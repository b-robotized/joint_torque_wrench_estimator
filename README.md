# joint_torque_wrench_estimator

Model-based Cartesian wrench estimator for a serial arm, from measured joint
torques and a URDF. A `ros2_control` broadcaster that exposes its estimate
through the standard `ForceTorqueSensor` state interfaces, so consumers use it
exactly as they would a real wrist F/T sensor.

For robots without an F/T sensor but with joint torque, motor current, or a
controller-provided disturbance torque.

Each cycle it estimates the external joint torque `τ_ext`, then maps it to a
Cartesian wrench at the tool with a damped pseudo-inverse of `Jᵀ`:

```
F = (J Jᵀ + α I)⁻¹ J τ_ext
```

`τ_ext` comes either from a momentum observer over the model dynamics (when the
input is the full joint torque) or straight from the input (when the robot
controller already reports a disturbance torque). See `torque_is_external`.

## Inputs

**State interfaces**, per joint of the chain `base_link` → `tip_link`:

| interface | notes |
|---|---|
| `<joint>/position` | |
| `<joint>/velocity` | only used when `torque_is_external` is false |
| `<joint>/effort` | the torque source; override with `torque_state_interfaces` |

No command interfaces are claimed.

**`robot_description`** from the controller manager, read once at configure.
The joint list, masses, inertias and Jacobians all derive from the URDF chain
between `base_link` and `tip_link`; every other joint is locked at its neutral
value. Kinematics are always used; masses and inertias only matter when
`torque_is_external` is false.

## Outputs

**Exported state interfaces**, in the standard FTS layout, prefixed with the
controller's instance name (a `ros2_control` requirement for chainable
controllers — the instance name *is* the sensor name downstream):

```
<controller_name>/force.x   force.y   force.z
<controller_name>/torque.x  torque.y  torque.z
```

**Topics**

| topic | type | contents |
|---|---|---|
| `~/wrench` | `geometry_msgs/WrenchStamped` | the estimated wrench |
| `~/payload` | `geometry_msgs/InertiaStamped` | mass and CoG the wrench implies |

Both publish every update cycle through a real-time publisher with `trylock`,
so samples may be dropped under subscriber contention.

### `~/payload`

Interprets the wrench as a single gravity load — valid only while the robot is
stationary and touching nothing but its payload. Choosing when to sample is the
consumer's job; the topic publishes unconditionally.

- `inertia.m` = `|F| / ‖gravity_vector‖`
- `inertia.com` = `(F × τ) / |F|²`, the minimum-norm solution of `τ = r × F`.
  Only the component of the offset **perpendicular** to `F` is observable: a
  force along its own line of action produces no extra moment, so the offset
  parallel to gravity reads zero.
- inertia tensor is always **zero** — it cannot be identified without dynamic
  excitation.

## Parameters

Generated with `generate_parameter_library`; see
`src/wrench_estimator_broadcaster_parameters.yaml` for full descriptions and
validation.

| parameter | type | default | description |
|---|---|---|---|
| `torque_state_interfaces` | `string[]` | `[]` | State interfaces carrying the measured torque, one per joint in Pinocchio chain order. Empty means `<joint>/effort`. |
| `torque_scale` | `double[]` | `[1.0]` | Multiplies each measured torque, converting the source's units to Nm. Scalar or per-joint. May be negative where the source's sign convention differs from the URDF joint axis. |
| `torque_is_external` | `bool` | `false` | `false`: input is the FULL joint torque, so `τ_ext` is extracted here. `true`: input is ALREADY the external/disturbance torque and is used directly. |
| `base_link` | `string` | — | Root of the kinematic chain. Required. |
| `tip_link` | `string` | — | Tool frame: kinematic end of the chain, and the reporting frame. Required. |
| `pinv_damping` | `double` | `0.01` | `α` in `(J Jᵀ + αI)⁻¹`. Trades under-estimation for noise immunity where `J` is ill-conditioned. |
| `singularity_threshold` | `double` | `0.001` | Minimum `σ_min(J)`; below it the output holds its last value. |
| `gravity_vector` | `double[3]` | `[0, 0, -9.81]` | Gravity in `base_link` frame — **not** world, unless `base_link` is gravity-aligned. When `torque_is_external` is true only the magnitude is used. |
| `wrench_filter_cutoff_hz` | `double` | `10.0` | Output low-pass cutoff. `0` disables. |
| `joint_torque_filter_cutoff_hz` | `double` | `0.0` | Pre-filter on the measured torque. `0` disables. |
| `joint_velocity_filter_cutoff_hz` | `double` | `0.0` | Pre-filter on joint velocity. `0` disables. |
| `wrench_deadband` | `double[6]` | zeros | Per-axis deadband `[Fx Fy Fz Tx Ty Tz]`; output clamped to zero inside the band. |
| `observer_gain` | `double[]` | `[50.0]` | Momentum observer gain `K_O`. Scalar or per-joint. Unused when `torque_is_external` is true. |

Parameters are read at configure time. Changing them at runtime has no effect
until the controller is reconfigured.

## Conventions

**Reporting frame.** The wrench is always expressed at `tip_link`'s origin in
`tip_link`'s axes, on both the topic and the exported interfaces. There is no
override. Consumers needing another frame transform downstream — note that
tf2's `Wrench` transform only rotates and omits the `r × F` moment-arm term, so
changing the *point* requires a full spatial transform. Simpler: set `tip_link`
to the frame you want, and the Jacobian handles it.

**Sign.** The wrench is the one applied *by the environment on the robot* at the
tool.

**Joint ordering.** Always Pinocchio's order for the extracted chain. Never
re-sort by URDF declaration order or by name; `torque_state_interfaces` and
per-joint parameter vectors follow it. The order is logged at configure.

## Example

```yaml
wrench_estimator:
  ros__parameters:
    type: joint_torque_wrench_estimator/WrenchEstimatorBroadcaster
    base_link: "base_link"
    tip_link: "tool0"
    pinv_damping: 0.001
    wrench_filter_cutoff_hz: 2.0
```

With a torque source that is already external, on non-standard interfaces:

```yaml
    torque_state_interfaces:
      - "<gpio_name>/<interface>"    # one per joint, in Pinocchio chain order
      # ...
    torque_scale: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]   # source units -> Nm
    torque_is_external: true
```

The interface names and scale factors depend entirely on your hardware
component; check `ros2 control list_hardware_interfaces` for what it exports.

Feeding an already-external torque with `torque_is_external: false` subtracts
gravity twice and reports roughly the negative of the load.

## Build

```bash
colcon build --packages-select joint_torque_wrench_estimator
```

Requires `pinocchio`, `controller_interface`, `hardware_interface`,
`realtime_tools`, `geometry_msgs`, `generate_parameter_library`.

## Accuracy notes

- The damped pseudo-inverse **under-estimates**, and by a pose-dependent
  amount: measuring a known 20 kg load in simulation gave 0.2 % error at a
  well-conditioned pose and ~7 % at a poor one (`pinv_damping = 0.001`). Take
  measurements at one fixed pose so the bias becomes a constant you can
  calibrate out.
- `gravity_vector` is applied to the model at configure time. A wrong direction
  produces a constant multi-newton bias indistinguishable from a real load —
  unless `torque_is_external` is true, where only the magnitude is used.
- The estimate is only meaningful while stationary and out of contact; inertial
  and contact forces are indistinguishable from a payload.
