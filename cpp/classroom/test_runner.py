"""Runner contract tests: form, input, compile errors, live output, stop, timeout and busy.

Programs are compiled against a stub ``hanppie.h``; the SDK's own SIGTERM stop is
covered by the C++ loopback tests. Run with
``python3 -m unittest discover -s cpp/classroom``.
"""

from __future__ import annotations

import http.client
import json
import socket
import tempfile
import threading
import time
import unittest
from pathlib import Path

from runner import Limits, Runner, make_server

__all__: list[str] = []

STUB_HEADER = """
#include <csignal>
#include <cstdio>
#include <unistd.h>
"""

# Prints "ready", then on SIGTERM prints "robot stopped" and exits like the SDK does.
STOPPABLE = r"""
#include "hanppie.h"
void onTerm(int) { std::printf("robot stopped\n"); std::fflush(stdout); _exit(0); }
int main() {
    std::signal(SIGTERM, onTerm);
    std::printf("ready\n");
    for (;;) pause();
}
"""

STUBBORN = r"""
#include "hanppie.h"
int main() {
    std::signal(SIGTERM, SIG_IGN);
    std::printf("ready\n");
    for (;;) pause();
}
"""


def form(fields: dict[str, bytes]) -> tuple[bytes, str]:
    """A multipart/form-data body, as ``curl -F`` sends it, and its content type."""
    boundary = "hanppie-test-boundary"
    body = b""
    for name, value in fields.items():
        body += (
            f'--{boundary}\r\nContent-Disposition: form-data; name="{name}"; '
            f'filename="{name}"\r\nContent-Type: application/octet-stream\r\n\r\n'
        ).encode()
        body += value + b"\r\n"
    body += f"--{boundary}--\r\n".encode()
    return body, f"multipart/form-data; boundary={boundary}"


class RunnerTest(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        header = Path(directory.name) / "hanppie.h"
        header.write_text(STUB_HEADER)
        self.start(Runner(header, Limits(run_seconds=10, stop_grace_seconds=0.5)))

    def start(self, runner: Runner, allowed_origins: frozenset[str] = frozenset()) -> None:
        self.runner = runner
        self.server = make_server("127.0.0.1", 0, runner, allowed_origins)
        self.port = self.server.server_address[1]
        thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)
        self.addCleanup(self.wait_idle)

    def connect(self) -> http.client.HTTPConnection:
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=15)
        self.addCleanup(connection.close)
        return connection

    def run_program(self, source: str, stdin: bytes | None = None) -> http.client.HTTPResponse:
        fields = {"code": source.encode()}
        if stdin is not None:
            fields["input"] = stdin
        body, content_type = form(fields)
        connection = self.connect()
        connection.request("POST", "/run", body=body, headers={"Content-Type": content_type})
        return connection.getresponse()

    def call(self, method: str, path: str) -> tuple[int, dict[str, object]]:
        connection = self.connect()
        connection.request(method, path)
        response = connection.getresponse()
        return response.status, json.loads(response.read())

    def wait_idle(self) -> None:
        deadline = time.monotonic() + 5
        while self.runner.running() and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertFalse(self.runner.running())

    def test_compile_error_returns_the_compiler_output(self) -> None:
        response = self.run_program('#include "hanppie.h"\nint main() { forward(1) }\n')
        self.assertEqual(response.status, 422)
        self.assertIn("main.cpp:2", response.read().decode())
        self.assertEqual(self.call("GET", "/status"), (200, {"state": "idle"}))

    def test_program_output_and_exit_code(self) -> None:
        response = self.run_program(
            '#include "hanppie.h"\nint main() { std::printf("hello\\n"); return 3; }\n'
        )
        self.assertEqual(response.status, 200)
        self.assertEqual(response.read().decode(), "hello\n\n[runner] finished (exit code 3)\n")

    def test_program_reads_the_submitted_input(self) -> None:
        source = (
            '#include "hanppie.h"\n#include <iostream>\n'
            "int main() { int a, b; if (std::cin >> a >> b) std::cout << a + b << std::endl;"
            ' else std::cout << "no input" << std::endl; }\n'
        )
        response = self.run_program(source, b"3 4\n")
        self.assertEqual(response.read().decode(), "7\n\n[runner] finished (exit code 0)\n")
        response = self.run_program(source)
        self.assertEqual(response.read().decode(), "no input\n\n[runner] finished (exit code 0)\n")

    def test_stop_sends_sigterm_while_output_streams(self) -> None:
        response = self.run_program(STOPPABLE)
        self.assertEqual(response.status, 200)
        self.assertEqual(response.readline(), b"ready\n")  # before the program ends
        self.assertEqual(self.call("GET", "/status"), (200, {"state": "running"}))
        self.assertEqual(self.run_program(STOPPABLE).status, 409)
        self.assertEqual(self.call("POST", "/stop"), (200, {"stopped": True}))
        self.assertEqual(response.read().decode(), "robot stopped\n\n[runner] stopped\n")
        self.assertEqual(self.call("POST", "/stop"), (200, {"stopped": False}))

    def test_program_ignoring_sigterm_is_killed(self) -> None:
        response = self.run_program(STUBBORN)
        self.assertEqual(response.readline(), b"ready\n")
        started = time.monotonic()
        self.call("POST", "/stop")
        self.assertEqual(response.read().decode(), "\n[runner] stopped\n")
        self.assertLess(time.monotonic() - started, 3)

    def test_timeout_stops_the_program(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        header = self.runner.header
        self.start(Runner(header, Limits(run_seconds=1, stop_grace_seconds=0.5)))
        response = self.run_program(STOPPABLE)
        self.assertEqual(
            response.read().decode(), "ready\nrobot stopped\n\n[runner] timed out after 1 s\n"
        )

    def test_dropping_the_connection_stops_the_program(self) -> None:
        body, content_type = form({"code": STOPPABLE.encode()})
        client = socket.create_connection(("127.0.0.1", self.port), timeout=15)
        head = f"POST /run HTTP/1.1\r\nHost: x\r\nContent-Type: {content_type}\r\n"
        client.sendall(f"{head}Content-Length: {len(body)}\r\n\r\n".encode() + body)
        received = b""
        while b"ready" not in received:
            received += client.recv(4096)
        self.assertTrue(self.runner.running())
        client.close()
        self.wait_idle()

    def test_malformed_requests_are_rejected(self) -> None:
        connection = self.connect()
        connection.putrequest("POST", "/run")
        connection.endheaders()
        self.assertEqual(connection.getresponse().status, 411)
        raw = self.connect()
        raw.request("POST", "/run", body=b"int main() {}")
        self.assertEqual(raw.getresponse().status, 400)
        body, content_type = form({"input": b"1"})
        missing = self.connect()
        missing.request("POST", "/run", body=body, headers={"Content-Type": content_type})
        self.assertEqual(missing.getresponse().status, 400)
        too_large = "x" * (Limits().request_bytes + 1)
        self.assertEqual(self.run_program(too_large).status, 413)

    def test_browsers_are_rejected_unless_allowed(self) -> None:
        for origin in ("http://attacker.example", "null"):
            with self.subTest(origin=origin):
                connection = self.connect()
                connection.request("POST", "/stop", headers={"Origin": origin})
                response = connection.getresponse()
                self.assertEqual(response.status, 403)
                self.assertIsNone(response.getheader("Access-Control-Allow-Origin"))
        body, content_type = form({"code": STOPPABLE.encode()})
        connection = self.connect()
        headers = {"Content-Type": content_type, "Origin": "null"}
        connection.request("POST", "/run", body=body, headers=headers)
        self.assertEqual(connection.getresponse().status, 403)
        self.assertFalse(self.runner.running())

    def test_allowed_origins_get_cors_headers(self) -> None:
        self.start(self.runner, frozenset({"null"}))
        body, content_type = form({"code": b'#include <cstdio>\nint main() { std::puts("hi"); }'})
        connection = self.connect()
        headers = {"Content-Type": content_type, "Origin": "null"}
        connection.request("POST", "/run", body=body, headers=headers)
        response = connection.getresponse()
        self.assertEqual(response.status, 200)
        self.assertEqual(response.getheader("Access-Control-Allow-Origin"), "null")
        self.assertIn(b"hi", response.read())
        preflight = self.connect()
        preflight.request("OPTIONS", "/run", headers={"Origin": "null"})
        response = preflight.getresponse()
        self.assertEqual(response.status, 204)
        self.assertEqual(response.getheader("Access-Control-Allow-Private-Network"), "true")
        other = self.connect()
        other.request("POST", "/stop", headers={"Origin": "http://attacker.example"})
        self.assertEqual(other.getresponse().status, 403)


if __name__ == "__main__":
    unittest.main()
