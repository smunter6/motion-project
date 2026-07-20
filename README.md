# motion-project

A learning project: motion planning / trajectory generation for a simple
Cartesian robot, built to eventually drive EtherCAT servo drives (via EtherCRAB
+ CiA 402) while supporting a hardware-free **simulation mode** with simple
visualization.

We're building it one deliberate step at a time.

## Current state — Step 1: one axis, trajectory generation

Right now the project contains a single crate, `motion-core`, holding the
I/O-free heart of the system. So far that's the **trapezoidal velocity profile**
for a single axis: the thing that turns "move from A to B" into the stream of
per-cycle position setpoints a servo drive wants in CSP mode.

Nothing here touches hardware, EtherCAT, or a GUI yet — that's by design.

### Run the terminal demo

From the workspace root:

```
cargo run -p motion-core --bin demo_axis
```

This samples a 0 → 100 mm move at our 250 Hz control-loop rate and prints
time / position / velocity / phase, so you can watch the trapezoid happen as
numbers before we ever draw it.

### Run the tests

```
cargo test
```

The tests assert the physically-obvious invariants: the move starts at A and
ends at B at rest, never exceeds max velocity, is symmetric, handles the
short-move (triangular) case, works in the negative direction, is continuous
across phase boundaries, and reports phases correctly.

## Architecture (where this is going)

The guiding idea is a clean seam between the **motion logic** (pure, testable,
portable) and the **backend** that actually moves things (sim now, real EtherCAT
drives later). Both backends implement the same trait, so the planner never
knows or cares which is behind it.

Planned workspace layout:

```
motion-project/
├── motion-core/       # I/O-free: trajectory, kinematics, motion types   <-- we are here
├── axis-backend/      # the trait seam: command axes, read feedback
├── backend-sim/       # software plant model implementing the seam
├── backend-ethercat/  # EtherCRAB + CiA 402 (when hardware arrives)
├── viz/               # simple 2D + target-vs-actual plots
└── app/               # wires planner + backend + viz into the run loop
```

### Roadmap

1. **[done]** One axis: trapezoidal trajectory generator + invariant tests.
2. Two axes + coordination (finish-together time-scaling).
3. The `AxisGroup` backend trait + a simple integrator-based sim backend.
4. The fixed-timestep run loop: planner → backend → feedback.
5. Simple visualization (egui): 2D axis view + target-vs-actual plots.
6. Richer sim: second-order lag, limits/faults; S-curve (jerk-limited) profiles.
7. (Hardware later) EtherCRAB + CiA 402 backend behind the same trait.

## Design notes

- **Absolute-time trajectory, dt-stepping plant.** The trajectory generator is a
  pure function of elapsed time (exact, testable, jitter-robust). The stateful
  dt-stepping model belongs to the sim/real plant layer, which we add later. The
  two meet at the setpoint each control cycle.
- **f64 in the core.** Real drives use integer encoder counts; that conversion
  is the backend's job at the hardware seam, not the planner's.
- **250 Hz** control rate to start (4 ms cycle).
