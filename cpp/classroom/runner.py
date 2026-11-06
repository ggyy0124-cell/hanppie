"""Classroom runner: compile a C++ program against ``hanppie.h`` and run it.

Serves one robot and one program at a time over HTTP for teacher demos:

- ``POST /run`` takes a multipart form with the C++ source in ``code`` and
  optional standard input in ``input``, compiles the source with the Dev-C++
  options and streams the program output until it ends.
- ``POST /stop`` stops the running program; dropping the ``/run`` connection
  does the same.
- ``GET /status`` reports whether a program is running.

A stopped program first gets SIGTERM, which the SDK turns into a robot stop,
and is killed after a grace period. Only the Python standard library is used.
"""

from __future__ import annotations

import argparse
import json
import os
import select
import signal
import socket
import subprocess
import tempfile
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from email import policy
from email.parser import BytesParser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

__all__ = ["Limits", "Runner", "main", "make_server"]

COMPILE_OPTIONS = ("-O2", "-std=c++11")
DISCONNECTED = "client disconnected"
FORM_HINT = "send a multipart form: code=@main.cpp, optionally input=@input.txt"


@dataclass(frozen=True)
class Limits:
    run_seconds: float = 180.0
    stop_grace_seconds: float = 2.0
    compile_seconds: float = 60.0
    output_bytes: int = 1024 * 1024
    request_bytes: int = 256 * 1024


class Runner:
    """Compiles and runs one program at a time."""

    def __init__(self, header: Path, limits: Limits, compiler: str = "g++") -> None:
        self.header = header
        self.limits = limits
        self.compiler = compiler
        self._busy = threading.Lock()
        self._stop = threading.Event()

    def acquire(self) -> bool:
        if not self._busy.acquire(blocking=False):
            return False
        self._stop.clear()
        return True

    def release(self) -> None:
        self._busy.release()

    def running(self) -> bool:
        return self._busy.locked()

    def stop(self) -> bool:
        if not self.running():
            return False
        self._stop.set()
        return True

    def compile(self, source: bytes, work: Path) -> str | None:
        """Build ``work/program``; return the compiler output on failure."""
        (work / "main.cpp").write_bytes(source)
        command = [
            self.compiler,
            *COMPILE_OPTIONS,
            "-I",
            str(self.header.parent),
            "main.cpp",
            "-o",
            "program",
        ]
        try:
            result = subprocess.run(
                command,
                cwd=work,
                stdin=subprocess.DEVNULL,
                capture_output=True,
                timeout=self.limits.compile_seconds,
                check=False,
            )
        except subprocess.TimeoutExpired:
            return f"compile timed out after {self.limits.compile_seconds:g} s\n"
        if result.returncode == 0:
            return None
        return (result.stdout + result.stderr).decode(errors="replace")

    def execute(
        self,
        work: Path,
        stdin: bytes,
        emit: Callable[[bytes], None],
        client: socket.socket | None,
    ) -> str:
        """Run ``work/program`` on ``stdin``, pass its output to ``emit`` and return how it ended."""
        (work / "input.txt").write_bytes(stdin)
        # Line-buffered stdio so output arrives while the program runs, not at exit.
        with (work / "input.txt").open("rb") as input_file:
            process = subprocess.Popen(
                ["stdbuf", "-oL", "-eL", str(work / "program")],
                cwd=work,
                stdin=input_file,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        assert process.stdout is not None
        with process.stdout:
            output = _Output(process.stdout.fileno(), emit, self.limits.output_bytes)
            deadline = time.monotonic() + self.limits.run_seconds
            reason = self._watch(output, client, deadline)
            if reason is None:
                try:
                    code = process.wait(timeout=max(0.0, deadline - time.monotonic()))
                except subprocess.TimeoutExpired:
                    reason = self._timed_out()
                else:
                    return _exit_reason(code)
            self._terminate(process)
            if reason != DISCONNECTED:
                output.drain(self.limits.stop_grace_seconds)  # what it printed while stopping
            return reason

    def _watch(self, output: _Output, client: socket.socket | None, deadline: float) -> str | None:
        """Forward output until it closes (None) or the run must end (the reason)."""
        watched = [output.fd] if client is None else [output.fd, client.fileno()]
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return self._timed_out()
            if self._stop.is_set():
                return "stopped"
            ready, _, _ = select.select(watched, [], [], min(remaining, 0.1))
            if client is not None and client.fileno() in ready:
                if _closed(client):
                    return DISCONNECTED
                watched = [output.fd]  # pipelined bytes: stop watching, never read them
            if output.fd in ready:
                try:
                    if not output.forward():
                        return None
                except OSError:
                    return DISCONNECTED
                if output.full:
                    return f"output over {self.limits.output_bytes} bytes"

    def _timed_out(self) -> str:
        return f"timed out after {self.limits.run_seconds:g} s"

    def _terminate(self, process: subprocess.Popen[bytes]) -> None:
        _signal_group(process, signal.SIGTERM)
        try:
            process.wait(timeout=self.limits.stop_grace_seconds)
        except subprocess.TimeoutExpired:
            _signal_group(process, signal.SIGKILL)
            process.wait()


class _Output:
    """Program output forwarded to the client up to a byte limit."""

    def __init__(self, fd: int, emit: Callable[[bytes], None], limit: int) -> None:
        self.fd = fd
        self.emit = emit
        self.room = limit
        self.full = False

    def forward(self) -> bool:
        """Forward one read; False once the output is closed."""
        chunk = os.read(self.fd, 4096)
        if not chunk:
            return False
        if not self.full:
            self.emit(chunk[: self.room])
            self.full = len(chunk) > self.room
            self.room = max(0, self.room - len(chunk))
        return True

    def drain(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        try:
            while select.select([self.fd], [], [], max(0.0, deadline - time.monotonic()))[0]:
                if not self.forward():
                    return
        except OSError:
            pass


def _form_fields(content_type: str, body: bytes) -> dict[str, bytes] | None:
    """Fields of a multipart/form-data body, or None if it is not one."""
    head = f"Content-Type: {content_type}\r\n\r\n".encode()
    message = BytesParser(policy=policy.HTTP).parsebytes(head + body)
    if message.get_content_type() != "multipart/form-data":
        return None
    fields: dict[str, bytes] = {}
    for part in message.walk():
        name = part.get_param("name", header="content-disposition")
        payload = part.get_payload(decode=True)
        if isinstance(name, str) and isinstance(payload, bytes):
            fields[name] = payload
    return fields


def _closed(client: socket.socket) -> bool:
    try:
        return client.recv(1, socket.MSG_PEEK | socket.MSG_DONTWAIT) == b""
    except BlockingIOError:
        return False
    except OSError:
        return True


def _signal_group(process: subprocess.Popen[bytes], number: signal.Signals) -> None:
    try:
        os.killpg(process.pid, number)
    except ProcessLookupError:
        pass


def _exit_reason(code: int) -> str:
    if code < 0:
        return f"ended by signal {signal.Signals(-code).name}"
    return f"finished (exit code {code})"


class _Handler(BaseHTTPRequestHandler):
    server: _Server
    protocol_version = "HTTP/1.1"

    def do_GET(self) -> None:
        if self.path != "/status":
            self._reply(HTTPStatus.NOT_FOUND, {"error": "not found"})
            return
        state = "running" if self.server.runner.running() else "idle"
        self._reply(HTTPStatus.OK, {"state": state})

    def do_POST(self) -> None:
        if self.path == "/run":
            self._run()
        elif self.path == "/stop":
            self._reply(HTTPStatus.OK, {"stopped": self.server.runner.stop()})
        else:
            self._reply(HTTPStatus.NOT_FOUND, {"error": "not found"})

    def _run(self) -> None:
        runner = self.server.runner
        length = self.headers.get("Content-Length")
        if length is None or not length.isdigit():
            self._reply(HTTPStatus.LENGTH_REQUIRED, {"error": FORM_HINT})
            return
        if int(length) > runner.limits.request_bytes:
            self.close_connection = True
            self._reply(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, {"error": "request too large"})
            return
        body = self.rfile.read(int(length))
        fields = _form_fields(self.headers.get("Content-Type", ""), body)
        if fields is None or "code" not in fields:
            self._reply(HTTPStatus.BAD_REQUEST, {"error": FORM_HINT})
            return
        if not runner.acquire():
            self._reply(HTTPStatus.CONFLICT, {"error": "a program is already running"})
            return
        reason = ""
        # Released before the response ends, so a client can run again right away.
        try:
            with tempfile.TemporaryDirectory(prefix="hanppie-run-") as directory:
                work = Path(directory)
                error = runner.compile(fields["code"], work)
                if error is None:
                    self.close_connection = True
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "text/plain; charset=utf-8")
                    self.send_header("Transfer-Encoding", "chunked")
                    self.end_headers()
                    reason = runner.execute(
                        work, fields.get("input", b""), self._chunk, self.connection
                    )
        finally:
            runner.release()
        if error is not None:
            self._send(HTTPStatus.UNPROCESSABLE_ENTITY, "text/plain", error.encode())
            return
        try:
            self._chunk(f"\n[runner] {reason}\n".encode())
            self.wfile.write(b"0\r\n\r\n")
        except OSError:
            pass

    def _chunk(self, data: bytes) -> None:
        if data:
            self.wfile.write(f"{len(data):x}\r\n".encode() + data + b"\r\n")
            self.wfile.flush()

    def _reply(self, status: HTTPStatus, body: dict[str, object]) -> None:
        self._send(status, "application/json", json.dumps(body).encode() + b"\n")

    def _send(self, status: HTTPStatus, content_type: str, body: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", f"{content_type}; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class _Server(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], runner: Runner) -> None:
        self.runner = runner
        super().__init__(address, _Handler)


def make_server(host: str, port: int, runner: Runner) -> ThreadingHTTPServer:
    return _Server((host, port), runner)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--header", type=Path, required=True, help="path to hanppie.h")
    parser.add_argument("--host", default="127.0.0.1", help="listen address")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--compiler", default="g++")
    parser.add_argument("--run-seconds", type=float, default=Limits.run_seconds)
    args = parser.parse_args(argv)
    if not args.header.is_file():
        parser.error(f"header not found: {args.header}")
    runner = Runner(args.header.resolve(), Limits(run_seconds=args.run_seconds), args.compiler)
    server = make_server(args.host, args.port, runner)
    print(f"listening on http://{args.host}:{server.server_address[1]}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        # The program runs in its own session, so stop it before exiting.
        runner.stop()
        deadline = time.monotonic() + runner.limits.stop_grace_seconds + 1
        while runner.running() and time.monotonic() < deadline:
            time.sleep(0.05)
        server.server_close()


if __name__ == "__main__":
    main()
