# Implementation Plan

## Purpose

Estimate the external Cartesian wrench acting on the tip of a serial kinematic
chain from the **measured joint torques** and a **dynamic model** of the robot.
Expose the estimate to the rest of `ros2_control` through the same state
interfaces a real 6-axis F/T sensor would publish, so downstream controllers
(admittance/impedance controllers, or a payload mass estimator) can consume it
without modification.

This is the "no wrist sensor" path: the wrench is inferred from the model
rather than measured.

## Architecture

```
┌────────────────── controller_manager (RT loop) ──────────────────┐
│                                                                  │
│  joint state interfaces                                          │
│    <joint>/position, <joint>/velocity, <joint>/effort            │
│         │                                                        │
│         ▼                                                        │
│  ┌──────────────────────────────────────────────────────┐        │
│  │ WrenchEstimatorBroadcaster                           │        │
│  │  - Pinocchio model (built from /robot_description)   │        │
│  │  - Momentum observer  →  τ_ext                       │        │
│  │  - Damped (Jᵀ)⁺       →  F̂_ext                       │        │
│  │  - LPF + deadband + singularity gate                 │        │
│  │                                                      │        │
│  │ Exports state interfaces (FTS layout):               │        │
│  │   <sensor_name>/force.x,  …, /torque.z               │        │
│  │ Publishes (debug):                                   │        │
│  │   geometry_msgs/WrenchStamped  on ~/wrench           │        │
│  └──────────────────────────────────────────────────────┘        │
│         │                                                        │
│         ▼                                                        │
│  any consumer of the FTS interfaces (e.g. payload estimation)     │
└──────────────────────────────────────────────────────────────────┘
```

The broadcaster is a `ChainableControllerInterface` (or plain
`ControllerInterface` that exports state interfaces). It claims **no command
interfaces**.

## Algorithm

### Inputs (per update)
- `q ∈ ℝⁿ` from `<joint>/position`
- `q̇ ∈ ℝⁿ` from `<joint>/velocity`
- `τ_meas ∈ ℝⁿ` from `<joint>/effort`

### Momentum observer (default algorithm)
Avoids numerical differentiation of `q̇`.

```
p     = M(q) · q̇                       (generalized momentum)
β(q,q̇) = g(q) − Cᵀ(q,q̇) · q̇             (Pinocchio: rnea with q̈=0 and a Coriolis term)
r̂_{k+1} = K_O · ( p − ∫₀ᵗ (τ_meas − β + r̂) dτ )
τ_ext  ≈ r̂
```

`r̂` is the residual; with diagonal positive gain `K_O` it acts as a first-order
observer on `τ_ext`. Reference: De Luca & Mattone, ICRA 2005.

### Cartesian mapping
```
J = J_tip(q)                                   (6 × n, geometric Jacobian)
F̂_ext = (J Jᵀ + α I₆)⁻¹ J · τ_ext              (damped right-pseudo-inverse of Jᵀ)
```
Equivalent and numerically tidy: `(Jᵀ)⁺_damped · τ_ext`.

### Post-processing
1. **Singularity gate.** Compute `λ_min(JJᵀ)` via a fixed-size
   `SelfAdjointEigenSolver`, take `σ_min(J) = √λ_min`. If
   `σ_min(J) < singularity_threshold`, hold the last filtered output (raw
   wrench is set to the LPF state, so the LPF and deadband freeze
   naturally). A throttled WARN identifies the event.
2. Exponential LPF at `wrench_filter_cutoff_hz`.
3. Per-axis deadband (`wrench_deadband`).

### Optional pre-filters
Both default off. Useful on real hardware, usually unnecessary in sim:
- `joint_velocity_filter_cutoff_hz`
- `joint_torque_filter_cutoff_hz`

### Alternative algorithm
`inverse_dynamics` mode: compute `τ_model = M q̈ + C q̇ + g` with `q̈` from
finite-differencing `q̇`. Provided for benchmarking / ablation. Not recommended
for online use because of `q̈` noise.

## Parameter Schema

```yaml
joint_torque_wrench_estimator:
  ros__parameters:

    # ─── Topology (required) ─────────────────────────────────────────
    base_link: ""             # root of the kinematic chain
    tip_link:  ""             # tip / wrench-application point

    # ─── Identity (how downstream consumers see us) ──────────────────
    # NOTE: ros2_control mandates that the prefix of a chainable controller's
    # exported state interfaces begins with the controller's own name. The
    # *controller instance name* (chosen at spawn time, e.g.
    # "wrench_estimator") therefore IS the FTS sensor name. There is no
    # sensor_name parameter.
    #
    # The reporting frame is ALWAYS tip_link — see "Frames & sign
    # conventions" below. There is no sensor_frame parameter.

    # ─── Physical ────────────────────────────────────────────────────
    gravity_vector: [0.0, 0.0, -9.81]  # in base_link frame.
                                       # NOT the world's [0,0,-9.81] unless
                                       # base_link is gravity-aligned.

    # ─── Estimator ───────────────────────────────────────────────────
    algorithm: "momentum_observer"     # or "inverse_dynamics"
    observer_gain: [50.0]              # K_O; scalar (broadcast) or n_joints
    pinv_damping: 1.0e-3               # α in (J Jᵀ + αI)⁻¹
    singularity_threshold: 1.0e-3      # min singular value of J

    # ─── Filtering / shaping ─────────────────────────────────────────
    wrench_filter_cutoff_hz: 10.0      # output LPF; 0.0 disables
    joint_velocity_filter_cutoff_hz: 0.0   # 0.0 disables
    joint_torque_filter_cutoff_hz:   0.0   # 0.0 disables
    wrench_deadband: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # per-axis [Fx Fy Fz Tx Ty Tz]
```

### Knob-by-knob rationale

| Knob | Why it's exposed |
|---|---|
| `base_link`, `tip_link` | Sole topology input; the kinematic chain, joint list, and dynamic parameters are derived from these via the URDF (read from `/robot_description`). Duplicating the joint list as a parameter would be a single-source-of-truth violation. `tip_link` also doubles as the *reporting frame* (see "Frames & sign conventions"). |
| `gravity_vector` | Pinocchio's gravity is set on the model in the **root frame**. If the robot base is rotated/mounted off-vertical, the world's `[0,0,-9.81]` is wrong in `base_link` frame. Get this wrong → constant tens-of-newtons bias on the estimate. |
| `algorithm` | Research flexibility: `momentum_observer` is the production default; `inverse_dynamics` exists for ablation studies. |
| `observer_gain` | The single most important tuning knob: trades response time against noise. Per-joint vector so individual joints can be tuned (e.g. wrist joints typically tolerate higher gains). |
| `pinv_damping` | Without damping the `(Jᵀ)⁺` blows up near singularities. Tunable because the "right" α depends on robot and task. |
| `singularity_threshold` | Decoupled from `pinv_damping`: damping keeps numbers finite, threshold tells you when to *trust* the result. |
| `wrench_filter_cutoff_hz` | Output low-pass. Required in practice; the raw estimate is noisy. |
| `joint_velocity_filter_cutoff_hz` | Pre-filter on `q̇`. Default off (most hardware/sim already filter). Useful on noisy real encoders. |
| `joint_torque_filter_cutoff_hz` | Pre-filter on `τ_meas`. Same logic. |
| `wrench_deadband` | Suppresses model-mismatch chatter in free space; lets higher-level logic key cleanly on contact/no-contact. Per-axis because translation and rotation deadbands differ in magnitude. |

### Deliberately *not* exposed

- **Joint list / joint count** — derived from URDF chain `base_link → tip_link`.
- **URDF source path** — read from `/robot_description` parameter/topic (ros2_control convention).
- **Effort interface name override** — convention is `<joint>/effort`; if hardware breaks the convention that's a hardware-plugin concern.
- **Payload mass / inertia at runtime** — separate concern (payload estimator); not baked into this surface.
- **Publish rate** — equals controller_manager update rate; spectral content is governed by the LPF.
- **`sensor_frame` (reporting frame override).** The wrench is *always* expressed at `tip_link`, in `tip_link`'s axes. See "Frames & sign conventions" — frame conversion is a consumer concern.

## Frames & sign conventions

**Reporting frame.** The wrench is published *at* `tip_link`'s origin, expressed
in `tip_link`'s axes. Both the `WrenchStamped` topic (`header.frame_id =
tip_link`) and the exported FTS state interfaces refer to the same frame.
There is no per-consumer reporting-frame override; the contract is fixed.

**Sign.** The wrench is the **external wrench applied BY the environment ON
the robot tip**, expressed in `tip_link`'s axes. A tool hanging from the EE under gravity therefore
shows up as a force pointing *downward in world*, which translates to whichever
axis points "down" in the local `tip_link` frame.

**If a consumer needs a different frame:**

- *Pure rotation* (same point, different axes — e.g. "give me the wrench in
  base_link axes"): tf2's `doTransform(WrenchStamped&, ...)` handles this.
- *Different point* (e.g. "wrench at the TCP, 12 cm below `tip_link`"): tf2's
  built-in Wrench transform only **rotates**; it does *not* apply the
  spatial moment-arm term `r × F`. Doing it that way silently gives a
  rotated-but-not-translated wrench, which is wrong. The fix is one of:
  - Pick `tip_link` to be the TCP frame itself if it's reachable by fixed
    joints from the kinematic end of the chain. The Jacobian then naturally
    refers to the TCP, and no spatial transform is needed downstream.
  - Or apply the full spatial wrench transform in the consumer (e.g. via
    Pinocchio's `SE3::act()` on a `pinocchio::Force`).

**Why this lives in the consumer, not the controller.** The estimator's job
is "joint torques → Cartesian wrench at the kinematic tip". The reporting
frame is interface-glue; baking it into the broadcaster would (a) expose a
configurable knob that has to be remembered at consumer-read time, and
(b) couple the estimator's complexity to a consumer concern. Single fixed
contract wins.

## State interfaces exported

Six, in the standard `semantic_components::ForceTorqueSensor` layout:

```
<controller_name>/force.x
<controller_name>/force.y
<controller_name>/force.z
<controller_name>/torque.x
<controller_name>/torque.y
<controller_name>/torque.z
```

`<controller_name>` is the instance name chosen at spawn time (e.g.
`wrench_estimator`). The prefix MUST start with the controller's own name
— this is a ros2_control invariant for chainable controllers.

This is the same shape `force_torque_sensor_broadcaster` produces from real
hardware, so any FTS consumer works unmodified (point its
`force_torque_sensor.name` at the controller instance name).

## Topics

- Published: `~/wrench` (`geometry_msgs/WrenchStamped`) — debug / RViz.
- Subscribed: none.

## Real-time discipline

- All Eigen / Pinocchio buffers allocated in `on_configure`.
- No allocation, no logging-by-string-formatting, no mutex blocking in `update()`.
- Publisher wrapped in `realtime_tools::RealtimePublisher` with `trylock`.

## Open items (to revisit)

- Whether to ship `algorithm: inverse_dynamics` in v0.1 or defer.
- Behaviour on singularity gate: hold last-good vs publish NaN vs publish zero
  (currently leaning "hold last-good, set a validity flag" — may need a
  custom message or status interface).
- Payload re-identification service (out of scope for v0.1).
