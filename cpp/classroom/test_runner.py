"""Runner contract tests: compile errors, live output, stop, timeout, busy and disconnect.

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


class RunnerTest(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        header = Path(directory.name) / "hanppie.h"
        header.write_text(STUB_HEADER)
        self.start(Runner(header, Limits(run_seconds=10, stop_grace_seconds=0.5)))

    def start(self, runner: Runner) -> None:
        self.runner = runner
        self.server = make_server("127.0.0.1", 0, runner)
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

    def run_program(self, source: str) -> http.client.HTTPResponse:
        connection = self.connect()
        connection.request("POST", "/run", body=source.encode())
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
        body = STOPPABLE.encode()
        client = socket.create_connection(("127.0.0.1", self.port), timeout=15)
        client.sendall(
            b"POST /run HTTP/1.1\r\nHost: x\r\nContent-Length: %d\r\n\r\n" % len(body) + body
        )
        received = b""
        while b"ready" not in received:
            received += client.recv(4096)
        self.assertTrue(self.runner.running())
        client.close()
        self.wait_idle()

    def test_requests_without_or_with_oversized_source_are_rejected(self) -> None:
        connection = self.connect()
        connection.putrequest("POST", "/run")
        connection.endheaders()
        self.assertEqual(connection.getresponse().status, 411)
        too_large = "x" * (Limits().source_bytes + 1)
        self.assertEqual(self.run_program(too_large).status, 413)


if __name__ == "__main__":
    unittest.main()
