# 2025 competition field simulation

This planar field is reconstructed from the automatic-control course figure in the
previous Fujian University Student Robot Competition rules. It is intended for
navigation integration and software testing; use the current organizer CAD before
building the physical course.

## Coordinate convention

- Frame: `odom`
- Unit: metre
- Origin: lower-left inside corner of the 9.4 m x 10.0 m arena
- Start pose: `(4.70, 0.50, 90 deg)`
- Shooting-zone goal: `(8.70, 4.25, 90 deg)`
- Target zone: 2.5 m x 2.5 m in the upper-right corner

The field geometry is stored in
`src/uav_planning/config/competition_field_2025.json`. The gray
`course_surfaces` are visual references. The `collision_segments` are the
boundaries published to EGO-Planner as a point cloud. The rising sections shown
in the rules are flattened in this 2D model.

## Run

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_bringup planar_simulation.launch.py
```

The simulator automatically sends the shooting-zone goal after three seconds.
In RViz, **2D Goal Pose** can be used to test another reachable point on the
automatic course. To override the default goal at launch:

```bash
ros2 launch uav_bringup planar_simulation.launch.py \
  goal_x:=8.70 goal_y:=4.25 goal_yaw:=1.5708
```
