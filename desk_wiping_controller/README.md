# desk_wiping_controller

This `ros2_control` controller makes a Franka Panda wipe a cross pattern on a
table while pressing down with a constant normal force. The same controller
runs in the MuJoCo simulation and on the real robot, with separate parameters
for each.

## Task sequence

After activation, the controller goes through these phases once:

| Phase        | What happens |
|--------------|--------------|
| `MOVE_ABOVE` | Moves from the current end-effector position to `center_xy`, keeping the initial height. |
| `DESCEND`    | Lowers the reference at `descent_speed` until the filtered contact force reaches `contact_threshold`. |
| `FORCE_RAMP` | Ramps the commanded normal force linearly from 0 to `desired_force` over `force_ramp_duration`. |
| `CROSS`      | Wipes a cross around the contact point: center → +x → center → +y → center → −y → center → −x → center, keeping `desired_force`. |
| `RETURN`     | Releases the force and moves back to the initial position. |
| `HOLD`       | Holds the initial pose. |

Point-to-point motions use a quintic (minimum-jerk) profile. The end effector
keeps the orientation it had at activation the whole time.

## Control law

Position and orientation errors produce a task wrench through a Cartesian
impedance law:

```
F = -K * e - D * (J * dq - v_desired)
```

During `FORCE_RAMP` and `CROSS`, the Z component of `F` comes from a force
controller on the filtered contact force `F_meas` instead:

```
F_z = -F_cmd - Kp_force * (F_cmd - F_meas) - Kd_force * v_z
```

The joint torque command is

```
tau = J^T * F + N * (Kp_nullspace * (q_preferred - q) - Kd_nullspace * dq) + coriolis
```

where `N` is the nullspace projector built from a damped pseudoinverse of `J`.
The command has no gravity term because the real robot compensates gravity
itself, and MuJoCo's `gravcomp` does the same in simulation.

The contact force is the Z component of `O_F_ext_hat_K` minus its value at
activation, multiplied by `force_sign` and low-pass filtered. Because of that
offset, **activate the controller while the end effector is not touching
anything**.

## Running in simulation

```bash
ros2 launch desk_wiping_controller desk_wiping_sim.launch.py
```

This starts MuJoCo with the desk-wiping scene
(`franka_description/mujoco/franka/desk_wiping_scene.xml`) and loads the
controller without activating it. To start the task, activate it from a second
terminal:

```bash
ros2 control switch_controllers --activate desk_wiping_controller
```

You can also activate it from the rqt GUI
(`ros2 run rqt_controller_manager rqt_controller_manager`). To start RViz as
well, add `use_rviz:=true` to the launch command.

## Running on the real robot

```bash
ros2 launch desk_wiping_controller franka_desk_wiping.launch.py robot_ip:=<robot-ip>
ros2 control switch_controllers --activate desk_wiping_controller
```

The launch file starts `franka_control2` with the joint state broadcaster and
the Franka robot state broadcaster, and loads the controller without activating
it. It also accepts `load_gripper:=true`, `use_rviz:=true` and
`use_fake_hardware:=true`.

Activate the controller only after the robot is unlocked, the table is in place
and the workspace is clear.

## Parameters

The values are in `config/desk_wiping_sim_controllers.yaml` for the simulation
and `config/desk_wiping_real_controllers.yaml` for the real robot.

| Parameter | Sim | Real | Description |
|-----------|-----|------|-------------|
| `arm_id` | `panda` | `panda` | Prefix of the joint and robot model interfaces. |
| `center_xy` | `[0.65, 0.0]` | `[0.65, 0.0]` | Center of the cross in the robot base frame [m]. |
| `forward_distance` | 0.10 | 0.10 | Length of the cross arms along x [m]. |
| `lateral_distance` | 0.10 | 0.10 | Length of the cross arms along y [m]. |
| `move_duration` | 4.0 | 4.0 | Duration of `MOVE_ABOVE` [s]. |
| `cross_segment_duration` | 2.0 | 2.0 | Duration of each of the 8 cross segments [s]. |
| `return_duration` | 4.0 | 4.0 | Duration of `RETURN` [s]. |
| `descent_speed` | 0.01 | 0.02 | Reference speed during `DESCEND` [m/s]. |
| `Kp_xy`, `Kd_xy` | 300, 34.64 | 300, 34.64 | Translational stiffness / damping in x and y. |
| `Kp_z`, `Kd_z` | 40, 12.65 | 40, 12.65 | Translational stiffness / damping in z (outside contact). |
| `Kp_orientation`, `Kd_orientation` | 20, 8.94 | 20, 8.94 | Rotational stiffness / damping. |
| `Kp_nullspace`, `Kd_nullspace` | 10, 6.32 | 10, 6.32 | Nullspace joint stiffness / damping. |
| `pseudoinverse_damping` | 0.05 | 0.05 | Damping of the Jacobian pseudoinverse. |
| `preferred_joint_position` | Panda ready pose | Panda ready pose | Target of the nullspace impedance [rad]. |
| `force_sign` | 1.0 | -1.0 | Makes the measured contact force positive when pressing down. |
| `desired_force` | 1.0 | 6.0 | Normal force while wiping [N]. |
| `contact_threshold` | 0.25 | 3.0 | Force that ends `DESCEND` [N]. |
| `force_ramp_duration` | 1.5 | 1.5 | Duration of `FORCE_RAMP` [s]. |
| `Kp_force`, `Kd_force` | 1.0, 10.0 | 1.0, 10.0 | Gains of the Z force controller. |
| `force_filter_time_constant` | 0.05 | 0.05 | Time constant of the force low-pass filter [s]. |

## Debugging

The controller logs the filtered contact force every 0.5 s at `DEBUG` level.
To see it, enable debug logging for the `desk_wiping_controller` logger.
