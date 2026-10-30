"""Test-only process fixture; all runtime behavior comes from installed C++ binaries."""
import json
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
from ament_index_python.packages import get_package_prefix


class NativeProcess:
    def __init__(self, executable, parameters=None, namespace=None):
        self.log = tempfile.TemporaryFile(mode='w+')
        self.port = None
        values = dict(parameters or {})
        if executable == 'socket_control':
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', 0))
                self.port = probe.getsockname()[1]
            values['socket_port'] = self.port
        binary = Path(get_package_prefix('tb4_multi_bringup')) / 'lib/tb4_multi_bringup' / executable
        argv = [str(binary), '--ros-args']
        if namespace:
            argv += ['-r', '__ns:=' + namespace]
        for key, value in values.items():
            argv += ['-p', key + ':=' + json.dumps(value, allow_nan=False)]
        self.process = subprocess.Popen(argv, stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    def request(self, request):
        with socket.create_connection(('127.0.0.1', self.port), timeout=3) as connection:
            connection.sendall((json.dumps(request) + '\n').encode())
            return json.loads(connection.makefile('rb').readline(8193))

    def status(self):
        return self.request({'command': 'status'})

    def wait_ready(self, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                self.log.seek(0)
                raise AssertionError(self.log.read())
            try:
                return self.status()
            except (OSError, ValueError):
                time.sleep(.05)
        raise AssertionError('Native gateway did not start')

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGTERM)
            try:
                self.process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
                raise AssertionError('Native node did not shut down')
        self.log.close()
