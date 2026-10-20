"""Pure configuration helpers shared by the fleet launch and its tests."""
from copy import deepcopy
import ipaddress


ROBOTS = ('robot1', 'robot2', 'robot3')


def active_robots(count):
    if type(count) is not int or not 1 <= count <= len(ROBOTS):
        raise ValueError('robot_count must be 1, 2 or 3')
    return ROBOTS[:count]


def validate_discovery(value, count):
    """Fail before starting DDS if a saved list contains empty/invalid addresses."""
    active_robots(count)
    if value is None:
        return  # Allow explicitly configured local/multicast discovery.
    servers = value.split(';')
    if servers[-1] == '':
        servers.pop()
    try:
        if len(servers) != count:
            raise ValueError('server count does not match robot_count')
        addresses = []
        for server in servers:
            address, port = server.rsplit(':', 1)
            parsed = ipaddress.IPv4Address(address)
            if parsed.is_unspecified or parsed.is_multicast or parsed.is_loopback or str(parsed) == '255.255.255.255':
                raise ValueError('expected a reachable Pi Wi-Fi/LAN address')
            if port != '11811':
                raise ValueError('expected port 11811')
            addresses.append(address)
        if len(set(addresses)) != count:
            raise ValueError('duplicate Pi addresses')
    except ValueError as error:
        raise ValueError('Invalid ROS_DISCOVERY_SERVER: ' + str(error) +
                         '. Run: bash scripts/laptop.sh configure') from error


def rviz_for_count(template, count):
    robots = active_robots(count)
    result = deepcopy(template)
    result['Visualization Manager']['Displays'] = [
        display for display in result['Visualization Manager']['Displays']
        if display.get('Name') not in ROBOTS or display['Name'] in robots
    ]
    return result
