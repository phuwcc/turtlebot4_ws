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


def robot_ids(value):
    ids = value.split(',') if isinstance(value, str) else list(value)
    if not ids or ids != sorted(set(ids)) or any(r not in ('robot1', 'robot2', 'robot3') for r in ids):
        raise ValueError('Chọn ID tăng dần, không trùng: robot1,robot2,robot3')
    return ids


def discovery_ids(value, ids):
    result = [int(v) for v in value.split(',')] if isinstance(value, str) else list(value)
    if len(result) != len(ids) or len(set(result)) != len(result) or any(type(i) is not int or not 0 <= i <= 2 for i in result):
        raise ValueError('Discovery server IDs phải từ 0 đến 2, không trùng và khớp số robot.')
    return result


def make_environment(addresses, selected=None, server_ids=None):
    if not 1 <= len(addresses) <= MAX_ROBOTS:
        raise ValueError('Số robot phải từ 1 đến 3.')
    addresses = [validate_address(value) for value in addresses]
    if len(set(addresses)) != len(addresses):
        raise ValueError('Mỗi Pi phải có một IP khác nhau.')
    ids = robot_ids(selected) if selected is not None else ['robot' + str(i + 1) for i in range(len(addresses))]
    if len(ids) != len(addresses):
        raise ValueError('Số IP không khớp số robot đã chọn.')
    indices = discovery_ids(server_ids, ids) if server_ids is not None else [int(r[-1]) - 1 for r in ids]
    slots = [''] * (max(indices) + 1)
    for index, address in zip(indices, addresses):
        slots[index] = address + ':11811'
    values = {
        'TB4_ROBOT_COUNT': str(len(addresses)), 'ROS_DOMAIN_ID': '0',
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'ROS_DISCOVERY_SERVER': ';'.join(slots) + ';',
        'ROS_SUPER_CLIENT': 'True', 'ROS_LOCALHOST_ONLY': '0',
    }
    if selected is not None:
        values['TB4_ROBOT_IDS'] = ','.join(ids)
    if server_ids is not None:
        values['TB4_DISCOVERY_IDS'] = ','.join(str(i) for i in indices)
    return values


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
    selected = values.get('TB4_ROBOT_IDS')
    ids = robot_ids(selected) if selected is not None else ['robot' + str(i + 1) for i in range(count)]
    server_ids = values.get('TB4_DISCOVERY_IDS')
    indices = discovery_ids(server_ids, ids) if server_ids is not None else [int(r[-1]) - 1 for r in ids]
    if not 1 <= count <= MAX_ROBOTS or len(ids) != count or len(servers) != max(indices) + 1:
        raise ValueError('Thiếu số robot hoặc danh sách discovery sai. Chạy lại configure.')
    by_index = {}
    for index, server in enumerate(servers):
        if index not in indices:
            if server:
                raise ValueError('Discovery server slot không chọn phải trống.')
            continue
        address, port = server.rsplit(':', 1)
        if port != '11811':
            raise ValueError('Discovery port phải là 11811.')
        by_index[index] = address
    expected = make_environment([by_index[i] for i in indices], selected, server_ids)
    if values != expected:
        raise ValueError('Cấu hình ROS không khớp. Chạy lại configure.')
    return expected


def save_environment(path, addresses, selected=None, server_ids=None):
    values = make_environment(addresses, selected, server_ids)
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


def configure(path, input_fn=input, selected=None, server_ids=None):
    print('Nhập câu trả lời sau dấu nhắc; không sửa nội dung lệnh read.')
    if selected is not None:
        ids = robot_ids(selected)
        count = len(ids)
    else:
        while True:
            count = input_fn('Số robot chạy trong phiên này [1/2/3]: ').strip()
            if count in ('1', '2', '3'):
                count = int(count)
                break
            print('Nhập 1, 2 hoặc 3; không để trống.')
        ids = ['robot' + str(i + 1) for i in range(count)]
    indices = discovery_ids(server_ids, ids) if server_ids is not None else [int(r[-1]) - 1 for r in ids]
    addresses = []
    for identity, index in zip(ids, indices):
        while True:
            value = input_fn(f'IP Wi-Fi/LAN {identity} (server ID {index}): ')
            try:
                address = validate_address(value)
                if address in addresses:
                    raise ValueError('IP này đã dùng cho robot trước.')
                addresses.append(address)
                break
            except ValueError as error:
                print('IP không hợp lệ: ' + str(error) + ' Nhập lại.')
    values = save_environment(path, addresses, selected, server_ids)
    print(f'Đã lưu {count} robot vào {path}')
    print('ROS_DISCOVERY_SERVER=' + values['ROS_DISCOVERY_SERVER'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--server-ids', help='Override discovery IDs thực tế, ví dụ 0 cho một Pi đang chạy server ID 0')
    parser.add_argument('--robots', help='Ví dụ robot2 hoặc robot1,robot3; mặc định chọn robot1..N')
    parser.add_argument('--path', type=Path, default=Path.home() / '.config/turtlebot4/pc_env.sh')
    args = parser.parse_args()
    try:
        if args.check:
            load_environment(args.path)
        else:
            configure(args.path, selected=args.robots, server_ids=args.server_ids)
    except (OSError, ValueError, EOFError, KeyboardInterrupt) as error:
        print('Chưa lưu cấu hình mới: ' + str(error))
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
