#!/usr/bin/env python3
"""Generate worlds/arena.world AND maps/arena.pgm+yaml from ONE geometry definition.

    python3 tools/make_arena.py            # rewrites both, in place

WHY THIS IS A GENERATOR AND NOT TWO HAND-WRITTEN FILES
------------------------------------------------------
AMCL does not compare the map to the world, it compares the map to the SCAN. A map that
disagrees with the world it was recorded in fails in the least helpful way available: the
particle filter still converges, reports a small covariance, and is confidently wrong,
which then reads as a Nav2 tuning problem. ackermann_bringup/launch/amcl.launch.py
carries a comment about exactly this happening on the vehicle with a map of a different
building.

Keeping both artefacts downstream of the same OBSTACLES list makes that class of mistake
unrepresentable: move a wall here and the world and the map move together.

The map is computed ANALYTICALLY rather than recorded with SLAM. That is legitimate here
only because every obstacle below is a vertical prism standing on the ground and taller
than the scan plane (see MIN_HEIGHT), so the set of surfaces a horizontal scanner can see
is exactly the 2D footprint of the obstacle list. Add a table, a ramp, or anything whose
cross-section varies with height, and this stops being true - map that world with
slam_sim.launch.py instead.
"""
import os

# ---------------------------------------------------------------------------
# GEOMETRY - the single source of truth for both outputs.
# ---------------------------------------------------------------------------
# Interior free space, metres. Chosen for THIS vehicle rather than copied from a
# turtlebot world: nav2_amcl.yaml plans at minimum_turning_radius 1.20 m, so the car
# needs a 2.40 m circle to turn around and cannot recover by rotating in place the way a
# differential-drive robot can. The stock turtlebot3_house corridors are ~1.5 m wide,
# which is narrower than that circle - the vehicle can enter and then have no way out
# except reversing, and every test becomes a test of reverse recovery.
import sys as _sys
# --corridor builds a NARROW world instead of the open arena. It exists because the
# open arena cannot reproduce the vehicle's real failure: on hardware, every goal
# AHEAD of the vehicle succeeded and every goal requiring a TURN-AROUND failed
# (bag 20260827_073550, 4/4 forward vs 0/4 reversing). In an 12 m wide arena there is
# always room to swing round, so the manoeuvre is never actually tested. A corridor
# narrower than the vehicle's own 2.40 m turning circle forces the multi-point turn
# that fails on the vehicle.
CORRIDOR = '--corridor' in _sys.argv
HALF_X, HALF_Y = (9.0, 1.1) if CORRIDOR else (8.0, 6.0)

WALL_T = 0.15          # wall thickness
WALL_H = 1.0           # every obstacle is this tall
MIN_HEIGHT = 0.279     # scan plane = lidar_joint z in sensors.xacro. Assert-checked below.

# (kind, args...) in metres. 'box' is (cx, cy, sx, sy); 'cyl' is (cx, cy, r).
OBSTACLES = [
    # Perimeter. Inner faces land exactly on +/-HALF_X, +/-HALF_Y.
    ('box', 0.0, HALF_Y + WALL_T / 2, 2 * HALF_X + 2 * WALL_T, WALL_T),
    ('box', 0.0, -(HALF_Y + WALL_T / 2), 2 * HALF_X + 2 * WALL_T, WALL_T),
    ('box', HALF_X + WALL_T / 2, 0.0, WALL_T, 2 * HALF_Y + 2 * WALL_T),
    ('box', -(HALF_X + WALL_T / 2), 0.0, WALL_T, 2 * HALF_Y + 2 * WALL_T),

    # Interior. Two wall stubs and four pillars, placed to leave no gap narrower than
    # 2.4 m (the vehicle's own turning circle) - verified by the clearance report this
    # script prints. The stubs matter more than the pillars: a scanner in a bare
    # rectangle sees four parallel walls and gives AMCL almost nothing to fix its
    # position ALONG a wall, so the filter slides. The stubs break that symmetry.
    ('box', -2.0, 2.5, 4.0, WALL_T),
    ('box', 3.0, -2.0, WALL_T, 4.0),
    ('cyl', 4.0, 3.0, 0.35),
    ('cyl', -4.0, -3.0, 0.35),
    ('cyl', 5.5, -3.5, 0.35),
    ('cyl', -6.0, 4.0, 0.35),
]
if CORRIDOR:
    # Bare corridor, 2.2 m clear width. The vehicle is 0.38 x 0.22 m and plans at a
    # 1.20 m turning radius (2.40 m circle), so it CANNOT arc round inside this width
    # and must execute a multi-point turn. Nothing else in the world, so a failure
    # here is about the manoeuvre and not about obstacle avoidance.
    OBSTACLES = [
        ('box', 0.0, HALF_Y + WALL_T / 2, 2 * HALF_X + 2 * WALL_T, WALL_T),
        ('box', 0.0, -(HALF_Y + WALL_T / 2), 2 * HALF_X + 2 * WALL_T, WALL_T),
        ('box', HALF_X + WALL_T / 2, 0.0, WALL_T, 2 * HALF_Y + 2 * WALL_T),
        ('box', -(HALF_X + WALL_T / 2), 0.0, WALL_T, 2 * HALF_Y + 2 * WALL_T),
    ]

# Map raster. 1 m of margin past the outer wall face so the unknown border is visible.
RES = 0.05
ORIGIN_X, ORIGIN_Y = -(HALF_X + 1.0), -(HALF_Y + 1.0)
WIDTH = int(round(2 * (HALF_X + 1.0) / RES))
HEIGHT = int(round(2 * (HALF_Y + 1.0) / RES))

FREE, OCC, UNKNOWN = 254, 0, 205
SUBSAMPLE = 3          # samples per pixel per axis; a 0.15 m wall is 3 px, so a
                       # pixel-centre-only test would alias thin walls away.

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def inside(kind, args, x, y):
    if kind == 'box':
        cx, cy, sx, sy = args
        return abs(x - cx) <= sx / 2 and abs(y - cy) <= sy / 2
    cx, cy, r = args
    return (x - cx) ** 2 + (y - cy) ** 2 <= r * r


def write_world():
    parts = []
    for i, (kind, *args) in enumerate(OBSTACLES):
        if kind == 'box':
            cx, cy, sx, sy = args
            geom = f'<box><size>{sx:.4f} {sy:.4f} {WALL_H:.4f}</size></box>'
        else:
            cx, cy, r = args
            geom = f'<cylinder><radius>{r:.4f}</radius><length>{WALL_H:.4f}</length></cylinder>'
        parts.append(f"""    <model name='obstacle_{i}'>
      <static>true</static>
      <pose>{cx:.4f} {cy:.4f} {WALL_H / 2:.4f} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>{geom}</geometry>
        </collision>
        <visual name='visual'>
          <geometry>{geom}</geometry>
          <material>
            <ambient>0.42 0.45 0.5 1</ambient>
            <diffuse>0.55 0.58 0.62 1</diffuse>
          </material>
        </visual>
      </link>
    </model>""")

    world = f"""<?xml version='1.0'?>
<!-- GENERATED BY tools/make_arena.py - DO NOT EDIT BY HAND.
     maps/arena.pgm is generated from the same OBSTACLES list in that script, so editing
     this file alone silently desynchronises the map AMCL localises against. Change the
     geometry there and re-run it. -->
<sdf version='1.6'>
  <world name='arena'>

    <!-- PHYSICS. COPIED VERBATIM FROM turtlebot3_house.world, which is the world the
         previous simulation of this vehicle ran in. Do not "optimise" it.

         🔴 An earlier version of this file used 500 Hz with iters 16, on the reasoning
         that a world of ten static boxes does not need 150 solver iterations and that
         halving the step count halves gzserver's load. Both halves of that were wrong.
         The vehicle DRIFTED AT REST: standing still with no command, it slid and slowly
         rotated, about -0.0093 rad/s of yaw - which imu_scale.py then dutifully measured
         as a gyro bias and subtracted, hiding the cause while corrupting the EKF's only
         yaw source. It is visible in Gazebo as the car turning by itself.

         Why it happens: this vehicle is 0.45 kg on four 0.0325 m wheels, and the drive
         joints are VELOCITY-commanded, so a stopped vehicle is a kinematic constraint
         holding the wheels at zero while contact forces push the body. Resolving that
         without creep needs the contact solver to actually converge. At iters 16 it does
         not, and the residual leaks out as sliding.

         The parts that matter most here, none of which were present before:
           iters 150                     - convergence, as above
           use_dynamic_moi_rescaling 1   - rescales the moment of inertia of small bodies
                                           against their contacts. This model has
                                           0.001 kg steering links carrying ixx=0.01,
                                           an inertia ~1e5x too large for the mass; that
                                           is what this flag exists to keep stable
           cfm 1e-5 / erp 0.2            - a slightly soft, damped constraint instead of
                                           a rigid one that rings
           contact_surface_layer 0.01    - allows 1 cm of penetration before a contact
                                           force is generated, which stops small wheels
                                           chattering against the floor

         Physics was never the expensive part anyway: measured on this world, gzserver
         costs 19% of one core with the camera off and 52% with it at 1920x1080. The
         camera is where the CPU goes, and sim_camera:=false is the knob for it. -->
    <physics type='ode'>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <precon_iters>0</precon_iters>
          <sor>1.400000</sor>
          <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000.000000</contact_max_correcting_vel>
          <contact_surface_layer>0.01000</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <!-- GROUND TRUTH, and the ability to put the vehicle back at the start without
         restarting Gazebo. Publishes /gazebo/model_states and serves
         /gazebo/set_entity_state, which is what tools/nav_probe.py uses to measure
         localisation error against truth and to reset between runs. Simulation only, by
         definition - there is no equivalent on the vehicle, so nothing in the navigation
         path may ever consume it. -->
    <plugin name='gazebo_ros_state' filename='libgazebo_ros_state.so'>
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <update_rate>20</update_rate>
    </plugin>

    <!-- shadows off: shadow mapping is re-rendered every frame in gzclient and buys
         nothing for a lidar-driven test. -->
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.7 0.75 0.8 1</background>
      <shadows>false</shadows>
    </scene>

{chr(10).join(parts)}

  </world>
</sdf>
"""
    path = os.path.join(HERE, 'worlds', 'arena_corridor.world' if CORRIDOR else 'arena.world')
    with open(path, 'w') as f:
        f.write(world)
    return path


def write_map():
    px = bytearray([UNKNOWN]) * (WIDTH * HEIGHT)
    outer_x = HALF_X + WALL_T
    outer_y = HALF_Y + WALL_T
    step = RES / SUBSAMPLE
    off = [(-1 + 2 * k) * step / 2 for k in range(SUBSAMPLE)] if SUBSAMPLE > 1 else [0.0]

    for r in range(HEIGHT):
        for c in range(WIDTH):
            cx = ORIGIN_X + (c + 0.5) * RES
            cy = ORIGIN_Y + (HEIGHT - 1 - r + 0.5) * RES
            hit = False
            for dx in off:
                for dy in off:
                    x, y = cx + dx, cy + dy
                    if any(inside(k, a, x, y) for k, *a in OBSTACLES):
                        hit = True
                        break
                if hit:
                    break
            if hit:
                px[r * WIDTH + c] = OCC
            elif abs(cx) < outer_x and abs(cy) < outer_y:
                # Interior and not an obstacle: the scanner has line of sight to it from
                # anywhere in the arena, so it is observed-free, not unknown.
                px[r * WIDTH + c] = FREE

    pgm = os.path.join(HERE, 'maps', 'arena_corridor.pgm' if CORRIDOR else 'arena.pgm')
    with open(pgm, 'wb') as f:
        f.write(b'P5\n# generated by tools/make_arena.py from worlds/arena.world\n')
        f.write(f'{WIDTH} {HEIGHT}\n255\n'.encode())
        f.write(bytes(px))

    yaml = os.path.join(HERE, 'maps', 'arena_corridor.yaml' if CORRIDOR else 'arena.yaml')
    with open(yaml, 'w') as f:
        IMG = "arena_corridor.pgm" if CORRIDOR else "arena.pgm"
        f.write(f"""# GENERATED BY tools/make_arena.py - DO NOT EDIT BY HAND.
# Computed from the geometry of worlds/arena.world, not recorded with SLAM, so it matches
# that world exactly rather than to within a scan-matching residual.
# free_thresh IS 0.196, NOT the 0.25 used by ackermann_bringup/maps/*.yaml, and the
# difference is not cosmetic. nav2's map_server converts a pixel to occ = (255-value)/255
# and calls it FREE when occ < free_thresh. The conventional 'unknown' grey is 205, which
# gives occ = 0.196 exactly - so at free_thresh 0.25 every unknown cell is read as FREE.
# On this map that turns the entire region OUTSIDE the arena walls into free space: it
# renders as one uniform light rectangle in RViz with no wall boundary visible, and the
# global costmap will happily plan through the wall into it, because track_unknown_space
# has nothing left to mark as unknown.
# 0.196 makes 205 land between the thresholds, which is what 'trinary' means to do.
# nav2_bringup's own turtlebot3_world.yaml uses 0.196 for exactly this reason.
image: {IMG}
mode: trinary
resolution: {RES}
origin: [{ORIGIN_X}, {ORIGIN_Y}, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""")
    return pgm, yaml, px


def report(px):
    assert WALL_H > MIN_HEIGHT, 'obstacles must reach above the scan plane'
    free = sum(1 for b in px if b == FREE)
    occ = sum(1 for b in px if b == OCC)
    print(f'map {WIDTH}x{HEIGHT} px @ {RES} m  origin ({ORIGIN_X}, {ORIGIN_Y})')
    print(f'  free {free}  occupied {occ}  unknown {WIDTH * HEIGHT - free - occ}')

    # Narrowest gap between any two obstacles, and each obstacle's clearance to the
    # spawn point. Nav2 plans at minimum_turning_radius 1.20 m -> 2.40 m turning circle.
    def sep(a, b):
        ka, *aa = a
        kb, *bb = b
        # crude but sufficient: sample the boundary of each and take the min distance
        import math
        def pts(kind, args, n=180):
            if kind == 'cyl':
                cx, cy, r = args
                return [(cx + r * math.cos(2 * math.pi * i / n),
                         cy + r * math.sin(2 * math.pi * i / n)) for i in range(n)]
            cx, cy, sx, sy = args
            out = []
            for i in range(n):
                t = i / n * 4
                if t < 1:   out.append((cx - sx / 2 + sx * t, cy - sy / 2))
                elif t < 2: out.append((cx + sx / 2, cy - sy / 2 + sy * (t - 1)))
                elif t < 3: out.append((cx + sx / 2 - sx * (t - 2), cy + sy / 2))
                else:       out.append((cx - sx / 2, cy + sy / 2 - sy * (t - 3)))
            return out
        pa, pb = pts(ka, aa), pts(kb, bb)
        return min(math.dist(p, q) for p in pa for q in pb)

    # The first four entries are the perimeter; they meet at the corners by
    # construction, so a pair of them reports 0.00 m and would hide every real gap.
    # Only pairs involving at least one INTERIOR obstacle are drivable gaps.
    n_perimeter = 4
    pairs = [(sep(OBSTACLES[i], OBSTACLES[j]), i, j)
             for i in range(len(OBSTACLES)) for j in range(i + 1, len(OBSTACLES))
             if j >= n_perimeter]
    pairs.sort()
    print('  tightest drivable gaps (vehicle turning circle 2.40 m, body 0.38 x 0.22 m):')
    for d, i, j in pairs[:4]:
        print(f'    obstacle_{i} <-> obstacle_{j}: {d:.2f} m')
    spawn_clear = min(sep(('cyl', 0.0, 0.0, 0.001), o) for o in OBSTACLES)
    print(f'  clearance at the spawn point (0, 0): {spawn_clear:.2f} m')


if __name__ == '__main__':
    w = write_world()
    pgm, yml, px = write_map()
    report(px)
    print('wrote', w)
    print('wrote', pgm)
    print('wrote', yml)
