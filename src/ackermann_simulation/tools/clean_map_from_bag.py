#!/usr/bin/env python3
"""Erase stale obstacles from a map, using a bag recorded in that map.

    clean_map_from_bag.py <bag_dir> <out_prefix>

Writes <out_prefix>.pgm / .yaml plus <out_prefix>_diff.pgm showing what was removed.

WHY THIS EXISTS
---------------
A static_layer obstacle is PERMANENT. The global costmap combines layers with
combination_method 1 (Maximum), so the obstacle layer's raytrace clearing can never
pull a cell below what the static layer asserts. An object that was present while
mapping and has since been removed therefore blocks the planner forever, and shows up
as "GridBased: failed to create plan, Starting point in lethal space!" or as goals that
are simply unreachable with no visible obstacle.

Measured on bag 20260827_073550: 2110 of the map's 4085 occupied cells - 52% - were
cells the LiDAR looked straight through, repeatedly, without ever getting a return from
them. Two of that run's four failed goals were adjacent to one such phantom, and one of
them ((3.15, 1.59)) was 1.80 m from the nearest REAL obstacle.

HOW A CELL IS JUDGED STALE
--------------------------
For every scan, every beam is Bresenham-traced from the sensor to its return point. A
map cell lying strictly BEFORE the return has been seen through: the beam reached
something further away, so nothing occupies that cell now. A cell is erased only when

    seen_through >= MIN_SEEN      (enough independent looks)
    hits <= HIT_RATIO*seen_through (essentially never returned from)

⚠️ THE THRESHOLDS ARE DELIBERATELY CONSERVATIVE. Erasing a real wall is a safety
defect, and the asymmetry of the two errors is not close: leaving a phantom costs a
failed goal, removing a real obstacle costs a collision. Cells that are merely unseen
are NEVER touched - absence of evidence is not evidence of absence, and a wall the
vehicle never looked at must stay.

This does not replace re-mapping. It repairs a specific, verifiable defect in an
existing map from evidence already recorded.
"""
import math
import os
import sys
from collections import Counter

MIN_SEEN = 50        # independent look-throughs before a cell may be erased
HIT_RATIO = 0.02     # and it must have returned on <=2% of those
LIDAR_X = 0.13       # lidar_joint x offset from base_link (base.xacro)


def read_bag(bagdir, topics):
    """Typed messages out of an mcap/sqlite3 bag, without needing ros2bag CLI."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    storage = 'mcap' if any(f.endswith('.mcap') for f in os.listdir(bagdir)) else 'sqlite3'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bagdir, storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    out = []
    while reader.has_next():
        topic, data, ts = reader.read_next()
        out.append((topic, ts, deserialize_message(data, get_message(types[topic]))))
    return out


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    if len(sys.argv) < 3:
        sys.stderr.write(__doc__)
        return 2
    bag, out = sys.argv[1], sys.argv[2]
    msgs = read_bag(bag, ['/map', '/scan', '/tf'])
    t0 = min(m[1] for m in msgs)
    mp = next((m for t, ts, m in msgs if t == '/map'), None)
    if mp is None:
        print('bag has no /map - record it next time'); return 1
    info = mp.info
    res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
    occ = {i for i, v in enumerate(mp.data) if v >= 65}

    m2o = o2b = None
    poses = []
    for t, ts, m in msgs:
        if t != '/tf':
            continue
        for tr in m.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                m2o = tr.transform
            if tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_footprint':
                o2b = tr.transform
        if m2o and o2b:
            a = yaw_of(m2o.rotation)
            poses.append(((ts - t0) / 1e9,
                          m2o.translation.x + math.cos(a) * o2b.translation.x
                          - math.sin(a) * o2b.translation.y,
                          m2o.translation.y + math.sin(a) * o2b.translation.x
                          + math.cos(a) * o2b.translation.y,
                          a + yaw_of(o2b.rotation)))
    if not poses:
        print('bag has no map->odom->base_footprint chain'); return 1

    seen, hit = Counter(), Counter()
    scans = [(ts, m) for t, ts, m in msgs if t == '/scan']
    for ts, m in scans:
        t = (ts - t0) / 1e9
        p = min(poses, key=lambda q: abs(q[0] - t))
        lx = p[1] + math.cos(p[3]) * LIDAR_X
        ly = p[2] + math.sin(p[3]) * LIDAR_X
        c0, r0 = int((lx - ox) / res), int((ly - oy) / res)
        for i, rng in enumerate(m.ranges):
            if not (math.isfinite(rng) and m.range_min <= rng <= m.range_max):
                continue
            a = m.angle_min + i * m.angle_increment + p[3]
            c1 = int((lx + rng * math.cos(a) - ox) / res)
            r1 = int((ly + rng * math.sin(a) - oy) / res)
            dc, dr = abs(c1 - c0), abs(r1 - r0)
            sc, sr = (1 if c1 > c0 else -1), (1 if r1 > r0 else -1)
            err, c, r, n = dc - dr, c0, r0, 0
            while (c, r) != (c1, r1) and n < 600:
                if 0 <= c < info.width and 0 <= r < info.height:
                    idx = r * info.width + c
                    if idx in occ:
                        seen[idx] += 1
                e2 = 2 * err
                if e2 > -dr:
                    err -= dr; c += sc
                if e2 < dc:
                    err += dc; r += sr
                n += 1
            if 0 <= c1 < info.width and 0 <= r1 < info.height:
                idx = r1 * info.width + c1
                if idx in occ:
                    hit[idx] += 1

    stale = {i for i in occ
             if seen[i] >= MIN_SEEN and hit[i] <= HIT_RATIO * seen[i]}
    print(f'map {info.width}x{info.height} @ {res}   occupied {len(occ)}')
    print(f'scans used {len(scans)}   ERASED {len(stale)} cells '
          f'({100*len(stale)/max(len(occ),1):.0f}% of occupied)')
    print(f'  kept: {len(occ)-len(stale)} occupied cells '
          f'(seen-through < {MIN_SEEN}, or returned from > {HIT_RATIO:.0%} of looks)')

    # PGM convention: row 0 is the TOP, and map_server maps the BOTTOM-LEFT to origin,
    # so the occupancy row index must be flipped when writing.
    W, H = info.width, info.height
    px = bytearray(W * H)
    diff = bytearray(W * H)
    for r in range(H):
        for c in range(W):
            idx = r * W + c
            v = mp.data[idx]
            if idx in stale:
                out_v, d = 254, 0          # erased -> free; black in the diff
            elif v >= 65:
                out_v, d = 0, 254
            elif v < 0:
                out_v, d = 205, 205
            else:
                out_v, d = 254, 254
            px[(H - 1 - r) * W + c] = out_v
            diff[(H - 1 - r) * W + c] = d
    for name, buf in ((out + '.pgm', px), (out + '_diff.pgm', diff)):
        with open(name, 'wb') as f:
            f.write(b'P5\n# cleaned by clean_map_from_bag.py\n')
            f.write(f'{W} {H}\n255\n'.encode())
            f.write(bytes(buf))
    with open(out + '.yaml', 'w') as f:
        f.write(f"""# Stale obstacles erased by tools/clean_map_from_bag.py from a bag
# recorded in this map. {len(stale)} of {len(occ)} occupied cells were removed because the
# LiDAR repeatedly looked straight through them. See <prefix>_diff.pgm: black = erased.
image: {os.path.basename(out)}.pgm
mode: trinary
resolution: {res}
origin: [{ox}, {oy}, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""")
    print(f'wrote {out}.pgm / .yaml and {out}_diff.pgm')
    return 0


if __name__ == '__main__':
    sys.exit(main())
