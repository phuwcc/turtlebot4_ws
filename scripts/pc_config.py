#!/usr/bin/env python3
"""Configure laptop DDS with 1–3 validated Pi addresses, without editing shell code."""
import argparse
import ipaddress
import os
from pathlib import Path
import shlex
import tempfile


MAX_ROBOTS = 3


def validate_address(value):
    address = ipaddress.IPv4Address(value.strip())
    if address.is_unspecified or address.is_multicast or address.is_loopback or str(address) == '255.255.255.255':
        raise ValueError('Cần IP Wi-Fi/LAN của Pi, không dùng địa chỉ localhost/broadcast/multicast.')
    return str(address)


def make_environment(addresses):
    if not 1 <= len(addresses) <= MAX_ROBOTS:
        raise ValueError('Số robot phải từ 1 đến 3.')
    addresses = [validate_address(value) for value in addresses]
    if len(set(addresses)) != len(addresses):
        raise ValueError('Mỗi Pi phải có một IP khác nhau.')
    return {
        'TB4_ROBOT_COUNT': str(len(addresses)), 'ROS_DOMAIN_ID': '0',
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'ROS_DISCOVERY_SERVER': ';'.join(address + ':11811' for address in addresses) + ';',
        'ROS_SUPER_CLIENT': 'True', 'ROS_LOCALHOST_ONLY': '0',
    }


def load_environment(path):
    values = {}
    for line in path.read_text().splitlines():
        words = shlex.split(line, comments=True)
        if not words:
            continue
        if len(words) != 2 or words[0] != 'export' or '=' not in words[1]:
            raise ValueError('File môi trường không đúng định dạng. Chạy lại configure.')
        key, value = words[1].split('=', 1)
        values[key] = value
    count = int(values.get('TB4_ROBOT_COUNT', '0'))
    servers = values.get('ROS_DISCOVERY_SERVER', '').split(';')
    if servers[-1] == '':
        servers.pop()
    if len(servers) != count or not 1 <= count <= MAX_ROBOTS:
        raise ValueError('Thiếu số robot hoặc danh sách discovery sai. Chạy lại configure.')
    addresses = []
    for server in servers:
        address, port = server.rsplit(':', 1)
        if port != '11811':
            raise ValueError('Discovery port phải là 11811.')
        addresses.append(address)
    expected = make_environment(addresses)
    if values != expected:
        raise ValueError('Cấu hình ROS không khớp. Chạy lại configure.')
    return expected


def save_environment(path, addresses):
    values = make_environment(addresses)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            for key, value in values.items():
                stream.write('export ' + key + '=' + shlex.quote(value) + '\n')
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return values


def configure(path, input_fn=input):
    print('Nhập câu trả lời sau dấu nhắc; không sửa nội dung lệnh read.')
    while True:
        count = input_fn('Số robot chạy trong phiên này [1/2/3]: ').strip()
        if count in ('1', '2', '3'):
            count = int(count)
            break
        print('Nhập 1, 2 hoặc 3; không để trống.')
    addresses = []
    for index in range(count):
        while True:
            value = input_fn(f'IP Wi-Fi/LAN robot{index + 1} (server ID {index}): ')
            try:
                address = validate_address(value)
                if address in addresses:
                    raise ValueError('IP này đã dùng cho robot trước.')
                addresses.append(address)
                break
            except ValueError as error:
                print('IP không hợp lệ: ' + str(error) + ' Nhập lại.')
    values = save_environment(path, addresses)
    print(f'Đã lưu {count} robot vào {path}')
    print('ROS_DISCOVERY_SERVER=' + values['ROS_DISCOVERY_SERVER'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--path', type=Path, default=Path.home() / '.config/turtlebot4/pc_env.sh')
    args = parser.parse_args()
    try:
        if args.check:
            load_environment(args.path)
        else:
            configure(args.path)
    except (OSError, ValueError, EOFError, KeyboardInterrupt) as error:
        print('Chưa lưu cấu hình mới: ' + str(error))
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
