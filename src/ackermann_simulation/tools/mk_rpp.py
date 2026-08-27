import re, sys
src, dst = sys.argv[1], sys.argv[2]
lines = open(src).read().split('\n')
# find the FollowPath block (6-space indent key) and replace it wholesale
start = next(i for i, l in enumerate(lines) if l.strip() == 'FollowPath:')
indent = len(lines[start]) - len(lines[start].lstrip())
end = start + 1
while end < len(lines):
    l = lines[end]
    if l.strip() and (len(l) - len(l.lstrip())) <= indent:
        break
    end += 1
rpp = """      # Regulated Pure Pursuit, swapped in for MPPI to test whether the endgame stall is
      # specific to MPPI's sampling or is a property of the vehicle/path.
      # use_rotate_to_heading MUST be false: it commands an in-place rotation, which an
      # Ackermann vehicle cannot perform. RPP also rejects the combination of
      # use_rotate_to_heading and allow_reversing being true together, and cusped Smac
      # paths need allow_reversing.
      # regulated_linear_scaling_min_radius matches the planner's 1.20 m so the speed
      # regulation kicks in exactly where the vehicle is at its steering limit.
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.4
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      transform_tolerance: 0.2
      use_velocity_scaled_lookahead_dist: true
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 0.6
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: false
      regulated_linear_scaling_min_radius: 1.20
      regulated_linear_scaling_min_speed: 0.15
      use_rotate_to_heading: false
      allow_reversing: true
      max_robot_pose_search_dist: 10.0
      use_interpolation: true"""
out = lines[:start + 1] + rpp.split('\n') + lines[end:]
open(dst, 'w').write('\n'.join(out))
print(f'{dst}: replaced FollowPath lines {start+1}..{end} with RPP')
