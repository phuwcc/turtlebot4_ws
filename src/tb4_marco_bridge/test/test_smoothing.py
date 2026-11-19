"""Corner rounding keeps endpoints, stays within the deviation bound and the gateway segment limit."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tb4_marco_bridge.smoothing import smooth_points, _distance_to_polyline, _wrap  # noqa: E402


def grid_path(cells, step=.05):
    pts = [(x * step, y * step) for x, y in cells]
    out = []
    for i, p in enumerate(pts):
        q = pts[min(i + 1, len(pts) - 1)] if i < len(pts) - 1 else pts[i - 1]
        yaw = math.atan2(q[1] - p[1], q[0] - p[0]) if i < len(pts) - 1 else out[-1][2]
        out.append([p[0], p[1], yaw])
    return out


def check(points, deviation=.03):
    smooth = smooth_points(points, deviation)
    assert smooth[0][:2] == points[0][:2] and smooth[0][2] == points[0][2]
    assert smooth[-1] == list(points[-1])
    xy = [p[:2] for p in points]
    assert max(_distance_to_polyline(p, xy) for p in smooth) <= deviation + 1e-3
    assert max(math.dist(a[:2], b[:2]) for a, b in zip(smooth, smooth[1:])) <= .15
    return smooth


# L-shaped 90 degree corner and a 45 degree staircase.
l_path = grid_path([(i, 0) for i in range(10)] + [(9, j) for j in range(1, 10)])
smooth = check(l_path)
steps = [abs(_wrap(b[2] - a[2])) for a, b in zip(smooth[1:-1], smooth[2:-1])]
assert max(steps) < math.radians(45), 'corner heading must change gradually'
check(grid_path([(i, 0) for i in range(6)] + [(5 + i, i) for i in range(1, 6)] + [(10, 5 + j) for j in range(1, 6)]))

# Reversing (yaw opposite to travel) and in-place turns are left untouched.
reverse = [[0., 0., math.pi], [.05, 0., math.pi], [.1, 0., math.pi], [.1, .05, math.pi]]
assert smooth_points(reverse) is reverse
turn = [[0., 0., 0.], [0., 0., 1.], [.05, 0., 0.]]
assert smooth_points(turn) is turn
assert smooth_points(l_path, 0.) is l_path
print('smoothing ok')
