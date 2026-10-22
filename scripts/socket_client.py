#!/usr/bin/env python3
"""Send one JSON-lines task to the PC gateway (no ROS environment needed)."""
import argparse
import json
import os
import socket


def send(request, host='127.0.0.1', port=8765):
    with socket.create_connection((host, port), timeout=4) as connection:
        connection.sendall((json.dumps(request, allow_nan=False) + '\n').encode())
        with connection.makefile('rb') as stream:
            raw = stream.readline(8193)
        if not raw.endswith(b'\n') or len(raw) > 8192:
            raise ValueError('Invalid response from gateway')
        return json.loads(raw)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default=os.environ.get('TB4_SOCKET_HOST', '127.0.0.1'))
    parser.add_argument('--port', type=int, default=int(os.environ.get('TB4_SOCKET_PORT', '8765')))
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('status')
    sub.add_parser('menu')
    for command in ('goal', 'pose', 'cancel', 'select'):
        child = sub.add_parser(command)
        child.add_argument('robot', choices=('robot1', 'robot2', 'robot3'))
        if command in ('goal', 'pose'):
            for coordinate in ('x', 'y', 'yaw'):
                child.add_argument(coordinate, type=float)
    args = vars(parser.parse_args())
    host, port = args.pop('host'), args.pop('port')
    try:
        if args['command'] != 'menu':
            response = send(args, host, port)
            print(json.dumps(response, indent=2, ensure_ascii=False))
            return 0 if response.get('ok') else 1
        selected = 'robot1'
        while True:
            choice = input('Chọn robot [1/2/3], hủy goal [c], trạng thái [s], thoát [q]: ').strip()
            if choice == 'q':
                return 0
            if choice in ('1', '2', '3'):
                candidate = 'robot' + choice
                response = send({'command': 'select', 'robot': candidate}, host, port)
                if response.get('ok'):
                    selected = candidate
            elif choice == 'c':
                response = send({'command': 'cancel', 'robot': selected}, host, port)
            elif choice == 's':
                response = send({'command': 'status'}, host, port)
            else:
                continue
            print(json.dumps(response, ensure_ascii=False))
    except (OSError, ValueError) as error:
        print('Socket error: ' + str(error))
        return 1
    except (EOFError, KeyboardInterrupt):
        return 0


if __name__ == '__main__':
    raise SystemExit(main())
