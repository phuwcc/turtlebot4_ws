"""Exercise both Python clients with fragmented or invalid loopback responses."""
import json
from pathlib import Path
import socket
import sys
import threading
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from socket_client import send
from tb4_marco_bridge.cli import request


class SocketProtocolTest(unittest.TestCase):
    def exchange(self, client, chunks):
        received, errors = [], []
        with socket.socket() as server:
            server.bind(('127.0.0.1', 0))
            server.listen(1)
            server.settimeout(3)

            def serve():
                try:
                    with server.accept()[0] as connection:
                        connection.settimeout(3)
                        received.append(connection.makefile('rb').readline(65537))
                        for chunk in chunks:
                            connection.sendall(chunk)
                except Exception as error:
                    errors.append(error)

            thread = threading.Thread(target=serve, daemon=True)
            thread.start()
            try:
                return client({'command': 'status'}, port=server.getsockname()[1])
            finally:
                thread.join(timeout=4)
                self.assertFalse(thread.is_alive())
                self.assertEqual(errors, [])
                self.assertEqual(json.loads(received[0]), {'command': 'status'})
                self.assertTrue(received[0].endswith(b'\n'))

    def test_fragmented_response(self):
        for client in (send, request):
            with self.subTest(client=client.__module__):
                result = self.exchange(client, [b'{"ok":', b'true,"robots":{}}', b'\n'])
                self.assertTrue(result['ok'])

    def test_truncated_or_nonobject_response_is_rejected(self):
        for client in (send, request):
            for payload in (b'{"ok":true}', b'[]\n', b'bad-json\n'):
                with self.subTest(client=client.__module__, payload=payload):
                    with self.assertRaises((ValueError, RuntimeError)):
                        self.exchange(client, [payload])

    def test_oversized_request_rejected_before_connect(self):
        for client in (send, request):
            with patch('socket.create_connection') as connect:
                with self.assertRaises(ValueError):
                    client({'command': 'preview', 'padding': 'x' * 65536}, port=8765)
                connect.assert_not_called()


if __name__ == '__main__':
    unittest.main()
