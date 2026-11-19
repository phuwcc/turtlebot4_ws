"""Corner-rounding for MARCO grid paths, applied after the unmodified MARCO core.

MARCO returns dense 8-connected grid polylines (45/90 degree corners). Each
interior corner is replaced by a circular fillet whose distance from the
original polyline never exceeds ``max_deviation``; the result is resampled
with tangent headings so the controller can turn while driving. Start and end
poses are kept exactly. The caller still checks tracking against the original
(reserved) MARCO path, so the fillet only spends part of that corridor.
Paths with reversing segments or in-place turns are returned unchanged.
"""
import math


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _distance_to_polyline(point, polyline):
    best = float('inf')
    for a, b in zip(polyline, polyline[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        denom = dx * dx + dy * dy
        t = max(0., min(1., ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / denom)) if denom else 0.
        best = min(best, math.hypot(point[0] - a[0] - t * dx, point[1] - a[1] - t * dy))
    return best


def smooth_points(points, max_deviation=.03, spacing=.04):
    """Return [[x, y, yaw], ...] with rounded corners, or ``points`` unchanged."""
    if max_deviation <= 0 or len(points) < 3:
        return points
    xy = [tuple(p[:2]) for p in points]
    # Reversing or pure-rotation steps carry intent in the yaw; keep them verbatim.
    for i in range(1, len(points) - 1):
        a, b = xy[i - 1], xy[i]
        if math.dist(a, b) < 1e-6:
            return points
        if abs(_wrap(points[i][2] - math.atan2(b[1] - a[1], b[0] - a[0]))) > math.pi / 2:
            return points
    if math.dist(xy[-2], xy[-1]) < 1e-6:
        return points
    # Merge collinear runs so fillets can use the full straight length.
    vertices = [xy[0]]
    for i in range(1, len(xy) - 1):
        h1 = math.atan2(xy[i][1] - vertices[-1][1], xy[i][0] - vertices[-1][0])
        h2 = math.atan2(xy[i + 1][1] - xy[i][1], xy[i + 1][0] - xy[i][0])
        if abs(_wrap(h2 - h1)) > 1e-3:
            vertices.append(xy[i])
    vertices.append(xy[-1])
    if len(vertices) < 3:
        return points
    lengths = [math.dist(a, b) for a, b in zip(vertices, vertices[1:])]
    # Tangent length per corner: bounded by deviation and by half of each adjacent segment.
    tangents = [0.]
    for i in range(1, len(vertices) - 1):
        a, b, c = vertices[i - 1], vertices[i], vertices[i + 1]
        turn = abs(_wrap(math.atan2(c[1] - b[1], c[0] - b[0]) - math.atan2(b[1] - a[1], b[0] - a[0])))
        if turn < 1e-3 or turn > math.radians(135):
            tangents.append(0.); continue
        half = turn / 2
        radius = max_deviation / (1 / math.cos(half) - 1)
        tangents.append(min(radius * math.tan(half), lengths[i - 1] / 2, lengths[i] / 2))
    tangents.append(0.)
    # Build the curve as a dense list of (x, y) samples.
    curve = [vertices[0]]
    for i in range(1, len(vertices) - 1):
        a, b, c = vertices[i - 1], vertices[i], vertices[i + 1]
        t = tangents[i]
        u1 = ((b[0] - a[0]) / lengths[i - 1], (b[1] - a[1]) / lengths[i - 1])
        u2 = ((c[0] - b[0]) / lengths[i], (c[1] - b[1]) / lengths[i])
        if t <= 1e-6:
            curve.append(b); continue
        p1 = (b[0] - u1[0] * t, b[1] - u1[1] * t)
        p2 = (b[0] + u2[0] * t, b[1] + u2[1] * t)
        # Quadratic Bezier with control point at the corner: tangent at both
        # ends, and it stays inside the corner closer than the circular fillet.
        steps = max(4, int(math.ceil(2 * t / (spacing / 2))))
        for k in range(steps + 1):
            s = k / steps
            curve.append(((1 - s) ** 2 * p1[0] + 2 * (1 - s) * s * b[0] + s * s * p2[0],
                          (1 - s) ** 2 * p1[1] + 2 * (1 - s) * s * b[1] + s * s * p2[1]))
    curve.append(vertices[-1])
    # Resample by arc length.
    out = [curve[0]]
    carry = 0.
    for a, b in zip(curve, curve[1:]):
        seg = math.dist(a, b)
        if seg < 1e-9:
            continue
        pos = spacing - carry
        while pos < seg - 1e-9:
            out.append((a[0] + (b[0] - a[0]) * pos / seg, a[1] + (b[1] - a[1]) * pos / seg))
            pos += spacing
        carry = seg - (pos - spacing)
    if math.dist(out[-1], curve[-1]) > 1e-6:
        if math.dist(out[-1], curve[-1]) < spacing / 3 and len(out) > 1:
            out[-1] = curve[-1]
        else:
            out.append(curve[-1])
    if any(_distance_to_polyline(p, xy) > max_deviation + 1e-3 for p in out):
        return points
    result = [[out[0][0], out[0][1], points[0][2]]]
    for i in range(1, len(out) - 1):
        a, b = out[i - 1], out[i + 1]
        result.append([out[i][0], out[i][1], math.atan2(b[1] - a[1], b[0] - a[0])])
    result.append([out[-1][0], out[-1][1], points[-1][2]])
    return result
