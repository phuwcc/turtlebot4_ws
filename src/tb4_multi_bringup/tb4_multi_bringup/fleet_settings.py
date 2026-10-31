"""Pure configuration helpers shared by the fleet launch and its tests."""
from copy import deepcopy
import ipaddress
import math
from pathlib import Path

import yaml


ROBOTS = ('robot1', 'robot2', 'robot3')


def startup_poses(filename, count, selected=None):
    """Require an explicit map pose for every active robot when auto-init is used."""
    robots = active_robots(count, selected)
    if not filename:
        return {}
    with Path(filename).expanduser().open() as stream:
        poses = yaml.safe_load(stream)
    if not isinstance(poses, dict) or set(poses) - set(ROBOTS):
        raise ValueError('Initial poses YAML must contain robot1/robot2/robot3 keys')
    result = {}
    for robot in robots:
        pose = poses.get(robot)
        if not isinstance(pose, dict) or set(pose) != {'x', 'y', 'yaw'}:
            raise ValueError(robot + ' requires x, y and yaw in initial poses YAML')
        values = [pose[key] for key in ('x', 'y', 'yaw')]
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
            raise ValueError(robot + ' initial pose must contain finite numbers')
        result[robot] = dict(zip(('initial_x', 'initial_y', 'initial_yaw'), map(str, values)))
    return result


def active_robots(count, selected=None):
    if type(count) is not int or not 1 <= count <= len(ROBOTS):
        raise ValueError('robot_count must be 1, 2 or 3')
    if not selected:
        return ROBOTS[:count]
    robots = tuple(selected.split(',')) if isinstance(selected, str) else tuple(selected)
    if (len(robots) != count or len(set(robots)) != count
            or any(robot not in ROBOTS for robot in robots)
            or robots != tuple(sorted(robots))):
        raise ValueError('robot_ids must list robot_count unique, ordered IDs: robot1,robot2,robot3')
    return robots


def validate_discovery(value, count, selected=None, server_ids=None):
    """Preserve discovery server ID slots, including deliberate gaps for inactive robots."""
    robots = active_robots(count, selected)
    if value is None:
        return
    indices = [int(v) for v in server_ids.split(',')] if server_ids else [int(r[-1]) - 1 for r in robots]
    if len(indices) != count or len(set(indices)) != count or any(not 0 <= i <= 2 for i in indices):
        raise ValueError('Invalid TB4_DISCOVERY_IDS')
    servers = value.split(';')
    if servers[-1] == '':
        servers.pop()
    try:
        if len(servers) != max(indices) + 1:
            raise ValueError('server ID slots do not match active robots')
        addresses = []
        for index, server in enumerate(servers):
            if index not in indices:
                if server:
                    raise ValueError('inactive discovery server slot must be empty')
                continue
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
                         '. Run: bash scripts/pc-configure.sh') from error


def rviz_for_count(template, count, selected=None):
    robots = active_robots(count, selected)
    result = deepcopy(template)
    result['Visualization Manager']['Displays'] = [
        display for display in result['Visualization Manager']['Displays']
        if display.get('Name') not in ROBOTS or display['Name'] in robots
    ]
    for display in result['Visualization Manager']['Displays']:
        if display.get('Name', '').startswith('Shared map ('):
            display['Name'] = 'Shared map (' + robots[0] + ')'
            display['Topic']['Value'] = '/tb4_view/' + robots[0] + '/map'
    return result


def localization_hints(filename, count, selected=None, anchor_yaw=0.):
    """Bounded initial priors, relative to the saved-map anchor, not fixed TFs."""
    if not filename:
        return {}
    with Path(filename).expanduser().open() as stream:
        hints = yaml.safe_load(stream)
    if not isinstance(hints, dict) or set(hints)-set(ROBOTS):
        raise ValueError('Localization hints must use robot1/robot2/robot3 keys')
    result = {}
    for robot in active_robots(count, selected):
        if robot not in hints or robot == 'robot1':
            continue
        item = hints[robot]
        if not isinstance(item, dict) or not {'x','y','yaw'} <= set(item):
            raise ValueError(robot+' hint requires x, y, yaw')
        if set(item)-{'x','y','yaw','relative_to','std_xy','std_yaw'}:
            raise ValueError('Unknown localization hint field')
        values = [item[k] for k in ('x','y','yaw')]
        std_xy, std_yaw = item.get('std_xy',.2), item.get('std_yaw',.2)
        if any(type(v) not in (int,float) or not math.isfinite(v) for v in values+[std_xy,std_yaw]):
            raise ValueError('Hint and uncertainty must be finite numbers')
        if not .05 <= std_xy <= 1. or not .05 <= std_yaw <= math.pi:
            raise ValueError('Hint uncertainty outside supported range')
        relative = item.get('relative_to','map')
        x,y,yaw = values
        if relative == 'robot1':
            x,y = math.cos(anchor_yaw)*x-math.sin(anchor_yaw)*y, math.sin(anchor_yaw)*x+math.cos(anchor_yaw)*y
            yaw += anchor_yaw
        elif relative != 'map':
            raise ValueError('relative_to must be map or robot1')
        result[robot] = {'initial_hint':'true','hint_x':str(x),'hint_y':str(y),'hint_yaw':str(yaw),
                         'hint_std_xy':str(std_xy),'hint_std_yaw':str(std_yaw)}
    return result
