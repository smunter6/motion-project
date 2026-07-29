# Telemetry — externalising the record

Separate rolling plan, opened 2026-07-29. **Nothing is built.** A design
exploration promoted to a plan at the user's request, after the user raised
**Zenoh** as a candidate. Deliberately *not* part of the kinematics/limit plan
(`let-s-make-a-plan-humble-dawn.md`) — that one changes what the machine
*does*, this changes what we can *see*.

Sequencing: **after phase 2 (unified limit scheduling)**, and not merely as
priority ordering — half this phase's value is measuring the quantities phase 2
bounds (`v²κ`, joint rates, reachability margin). Built first it is
instrumentation with no consumer; built after it verifies phase 2's envelope
did what it claimed. One argument for partial inversion: see §9.

`CLAUDE.md` holds the architectural decisions this must respect. The
non-negotiable ones are restated in §11, because each is something a telemetry
layer typically breaks.

---

## 1. The framing fork, settled first

"Time series database" points at the monitoring lineage — Prometheus, Influx,
Timescale. Those serve dashboards and alerting over long horizons, assume you
read *aggregates*, and downsample on retention.

Wrong instrument for this project. Every hard-won bug so far was sample-exact
and transient:

- the `0.000s` ramp durations that exposed the cascade-ordering defect,
- the `v₀·T/2` overshoot that vanishes from rest, so every rest-to-rest test
  passed while redirects silently missed,
- an off-branch IK jump, which `backend-sim` shows only as a growing gap,
- following error during an accel transient.

A metrics store averages away the twelve cycles that mattered.

**Target is a flight recorder, not a metrics database:** lossless,
session-scoped, sample-exact, replayable. MCAP / ROS-bag / Parquet lineage.

A metrics rollup is worth having eventually — "is this machine degrading over
weeks" is real — but *derived from* the recorder, never the primary store.

**Prometheus is ruled out** on model grounds: pull-based scraping at 1–15 s is
structurally wrong for 250 Hz capture. Recorded because it is the reflex answer
to "time series".

---

## 2. The one hard constraint, and the architecture it forces

The loop has a 4 ms budget. Today `RecordingAxisGroup::exchange` takes a
`Mutex<History>` **on the control thread**. Fine right now — 0.04 ms/frame lock
wait, measured, consumer is a local `VecDeque`.

It stops being fine the instant the consumer is a file, a socket, or a broker:
cycle time then couples to disk latency, network hiccups, and someone else's
`fsync`.

**The `Mutex` is not the thing to optimise. The coupling is the thing to
remove.** The shape is forced, identical for any transport or storage choice:

1. **Control thread:** one bounded, non-blocking push into a fixed-capacity
   SPSC ring. No allocation, no I/O, no lock another thread can hold across a
   syscall.
2. **Writer ("pump") thread:** drains, encodes, batches, fans out. Everything
   expensive lives here.
3. **Drop on full, never block.**

Point 3 has a corollary that matters more than it sounds:

> **The record must be able to represent its own gaps.** Count drops and write
> an explicit discontinuity marker.

A silently-gapped trace is *worse than no trace*: you read the seam as a
physical velocity step and hunt a planner bug that doesn't exist. Same instinct
as fault-as-detector — don't mask.

---

## 3. Where Zenoh fits (and where it doesn't)

Good instinct, but it lands on a different layer than §1. Keep three separate:

| layer | question | candidates |
|---|---|---|
| **transport** | how do bytes leave the process (and the machine)? | **Zenoh**, TCP, UDS, shared mem |
| **encoding** | what do the bytes look like? | postcard, CDR, protobuf, CBOR |
| **storage / query** | what holds a session, how is it analysed? | MCAP, Parquet+DuckDB, SQLite, TSDB |

**Zenoh is a strong answer to transport and a non-answer to storage.** It
composes with MCAP/Parquet rather than competing.

### Genuinely good fit

- **Resolves "don't put a DB client in the control process" structurally.** The
  control process becomes a publisher that knows nothing about storage — the
  *same seam philosophy* as `AxisGroup`. That resonance is the strongest
  argument for it here.
- **The real topology is Pi → workstation.** EtherCAT testing happens on the
  Pi; there is no viz window there. Zenoh handles discovery and routing without
  inventing a socket protocol, and the same code works same-host and cross-host.
- **Pure Rust**, matching the EtherCRAB/`motion-core` stance. `zenoh-pico` (C,
  constrained devices) keeps the RP2350 aspiration coherent rather than
  orphaning telemetry there.
- **Hierarchical key expressions with wildcard subscription** map cleanly onto
  per-axis / per-group / per-event streams (§5).
- **Zero-copy shared memory same-host**, which makes a separate viz process
  practical (§8.4).
- **Storage plugins** can persist matching publications into a backend
  (filesystem, RocksDB, others) by config rather than custom writer code.
  Verify the current backend list and API against live docs before relying on
  a specific one.

### Cautions — read before committing

1. **Zenoh does not remove the ring buffer.** A `put` still serialises and may
   touch a lock or socket. Zenoh belongs on the *writer* side of the ring,
   never inside `exchange()`. The seductive failure mode is exactly "Zenoh is
   fast, just publish from the control thread."
2. **Congestion control must be `Drop`, never `Block`.** A blocking publisher
   in a 250 Hz loop converts network backpressure into missed cycles. Most
   dangerous default to get wrong, and it is a one-line config choice.
3. **WSL2 multicast discovery is a live risk.** Peer discovery conventionally
   uses UDP multicast, and WSL2's virtualised NAT networking is already why raw
   L2 EtherCAT can't work here. Expect explicit endpoint config for dev. Same
   class of environment gotcha as the `glow`-not-`wgpu` finding — budget time.
4. **Not an analysis engine.** No time-range analytics, no columnar scans. You
   still want MCAP or Parquet+DuckDB behind it.
5. **A bus is nondeterministic.** Ordering and timing jitter make it unfit as
   the source for golden-trace tests (§8.2). Keep a **file sink independent of
   Zenoh** so replay tests never depend on a broker.
6. **Version churn.** Long 0.x series with breaking changes before 1.0. Pin
   exact versions; expect the API to have moved relative to any tutorial.
7. **Payload-agnostic** — choosing Zenoh does not choose an encoding.

### Verdict

Adopt Zenoh as **transport for live/off-box streaming**, and keep a direct
**file sink for durable capture and replay**. Do not let Zenoh become the only
path to a record — the two have different reliability and determinism
requirements, and collapsing them costs the golden-trace payoff.

---

## 4. What to record beyond today

`Sample` captures `AxisSetpoint`/`AxisFeedback` wholesale — a good property
worth preserving, since new seam fields come along free. Four things missing.

### 4a. The right clock

`Sample.t` is `start.elapsed()` *when the recorder ran* — not the cycle's
logical time, and carrying scheduler jitter. Record three:

- **cycle index — primary key.** Exact, monotonic, jitter-free, comparable
  across runs. Required for replay determinism.
- wall clock, for correlating with external events and logs.
- the trajectory's own logical `t`, what every profile is a pure function of.

Today's single field is the least authoritative of the three.

### 4b. Loop timing health

Actual `dt`, overruns, time spent in the cycle's work. Invisible today. On the
Pi with real EtherCAT and DC sync this likely becomes the *most* important
channel — and it is the one thing sim can never tell you.

### 4c. App-level events — the part that isn't free

`CLAUDE.md` already records that the viz tap cannot see `app`'s
`Profile`/command bookkeeping. For an external record that is far more costly:
**without events the sample stream is uninterpretable.** You cannot distinguish
a commanded redirect from a plant anomaly.

Needed: move accepted/started/finished/aborted, queue promotions and drops,
group-cascade fires *with blame axis and reason*, IK branch chosen, DS402
transitions, fault raise/clear, `setlimits` changes.

Requires a **new hook in the control loop**, not just a decorator — the one
piece of real plumbing here. Low rate, high value.

### 4d. Session provenance

Config snapshot, git commit, binary version, backend identity (sim vs
EtherCAT), kinematic model and link lengths, and — specifically because
`setlimits` made them mutable — **the limits actually in effect**. Without it
two traces aren't comparable, and limits genuinely cannot be recovered from
source any more.

### 4e. Derived quantities (the phase-2 tie-in)

Arc length `s`, curvature, `v²κ`, joint rates, `sin(q2_eff)` margin. Makes the
known-unbounded quantities *measurable historically* — quantify the ≈3×
centripetal violation across runs instead of re-deriving by hand, then verify
phase 2's envelope.

Record raw **and** derived: if a definition later changes, having both is how
you notice.

---

## 5. Proposed architecture

New crate — `telemetry`. I/O and dependencies allowed. **Not `motion-core`**
(decision #2) and not `axis-backend`, which stays a pure seam definition.

```
control thread          pump thread                     consumers
─────────────────       ──────────────────────────      ─────────────────
RecordingAxisGroup ──┐
  (seam tap)         ├─► bounded SPSC ring ─► encode ─┬─► FileSink    (MCAP/Parquet)
control loop events ─┘   drop-on-full,                ├─► ZenohSink   (live/off-box)
                         counted + marked             └─► HistorySink (in-proc viz)
```

- **One producer, fan-out to N sinks, each with its own retention policy.** The
  key structural point: viz wants "last 60 s, decimated, in memory"; the file
  wants "everything, durable"; Zenoh wants "now, lossy, elsewhere". Same tap,
  three policies — do not make one serve all three.
- The existing `History` becomes *just another sink*, behaviour unchanged. That
  keeps this from becoming a viz rewrite.
- **Keep the decorator pattern**, so it composes: `Recording<Replay<Sim>>`.

### Key-expression namespace (Zenoh)

The public interface of the telemetry system; worth designing deliberately:

```
motion/<session>/meta                  provenance, once at start (§4d)
motion/<session>/loop                  cycle timing health (§4b)
motion/<session>/axis/<n>/exchange     setpoint+feedback per cycle
motion/<session>/group/<g>/path        s, curvature, v²κ, derived (§4e)
motion/<session>/event                 discrete app events (§4c)
```

Hierarchical so a subscriber takes `motion/*/axis/*/exchange`, or one axis, or
events only, without the publisher knowing who listens.

---

## 6. Volume

~6 `f64` plus status per axis per cycle ≈ 54 B. Four axes ≈ **216 B/cycle** →
≈ 54 KB/s → ≈ **190 MB/hour raw**.

This data is unusually compressible *because it is smooth by construction* —
the entire point of the profiles. Delta-of-delta on timestamps plus
XOR/Gorilla-style float encoding, then zstd: realistically 10–20×. Recording as
`f32` (while still *computing* in `f64` — decision #4 is about the core, not
the record) halves the input first.

Call it **10–20 MB/hour stored**. A non-issue on a host.

On the Pi the constraint is not space but **SD-card write endurance**, arguing
for rotate-and-ship, or writing to USB/network rather than the boot card.

---

## 7. Storage / format options

- **MCAP** — domain-correct, and the recommendation. Robotics standard for
  exactly this; self-describing schemas; *heterogeneous* streams (250 Hz
  samples and low-rate events in one file on one clock — precisely §4's
  structure); Foxglove for scrubbing with no viz work; Rust crate exists.
- **Append-only binary → Parquet, queried by DuckDB/Polars** — no server,
  excellent columnar analysis. The alternative if you'd rather live in the
  dataframe world.
- **SQLite** — single file, transactional, SQL, zero infra; fast enough with
  batched transactions. Weaker for columnar analytics.
- **QuestDB / ClickHouse / Timescale** — real servers, real ingest, good SQL.
  Earn their place with a second machine or months of history. Overkill for one
  arm on a bench, and they arrive with operational burden.
- **HDF5** — strong for numeric arrays and a Python/Matlab audience; weaker for
  mixed event streams.

**Tradeoff worth weighting:** today's design gets schema evolution free (seam
structs captured wholesale). A rigid DB schema takes that away and hands you
migrations. Self-describing formats (MCAP, Parquet) preserve more of it — an
argument about *maintenance*, not performance.

---

## 8. Payoffs that aren't obvious

1. **`ReplayAxisGroup`.** Once a record exists, implementing `AxisGroup` by
   playing recorded feedback back is nearly free — same seam, same decorator,
   composes as `Recording<Replay<Sim>>`.
2. **Golden-trace regression tests.** Attacks a gap `CLAUDE.md` already names:
   control-loop integration testing is slow and brittle with real `sleep()`s
   and `println!`-format assertions. A blessed trace plus deterministic replay
   asserts this commit's *commanded stream* matches sample-for-sample — far
   stronger than string matching, and it is the "extract the decision logic to
   run on fake time" idea with a concrete mechanism. Needs cycle-index keying
   (§4a) and a non-bus sink (§3.5).
3. **The only instrument on the Pi.** No viz window there. Following-error
   faults — the *designed* detector under commanded-not-feedback — become
   post-mortem analysable instead of a message that scrolled past.
4. **Viz could become a separate process** subscribing over Zenoh, dissolving
   the eframe/winit "GUI owns the main thread, so the loop is on a spawned
   thread" constraint by putting them in different processes. Speculative, *not*
   proposed for this phase, but it is what this unlocks.
5. **Makes phase 2 verifiable** rather than argued (§4e).

---

## 9. Phasing

Each step independently useful; stop after any.

- **T0 — decouple.** Bounded SPSC ring + pump thread + `Sink` trait; move
  `History` behind it as the first sink. No new dependency, no format choice,
  no Zenoh. **De-risks everything else**, and is valuable alone: it removes a
  blocking primitive from the control thread.
- **T1 — clock and gaps.** Cycle index as primary key, `dt`/overrun channel,
  drop counting with explicit discontinuity markers.
- **T2 — events + provenance.** The control-loop hook (§4c) and session header
  (§4d). The largest genuinely-new plumbing.
- **T3 — durable file sink.** MCAP (or Parquet). A session survives exit.
- **T4 — Zenoh sink.** Live/off-box, `CongestionControl::Drop`, explicit
  endpoints for WSL. Prove Pi → workstation.
- **T5 — replay + golden traces.** `ReplayAxisGroup`, blessed traces, and turn
  the un-automated phase-1 verifications (listed in the kinematics plan) into
  real tests.
- **T6 — optional metrics rollup.** Derived *from* T3 output, not parallel.

### Should a slice come first?

Honest argument against the stated sequencing: **T0–T1 before phase 2.** Phase
2 is a numerical scheduling problem whose failures will be subtle and transient
— exactly what today's instrumentation is worst at showing. A sample-exact
record *while* building the envelope may pay for itself immediately, and T0–T1
need no dependency and no format decision.

T2 onward genuinely should wait. **User's call.**

---

## 10. Open questions for the user

1. **Zenoh scope** — transport only (recommended), or lean on its storage
   plugins for persistence without a writer? The latter is less code, more
   coupling.
2. **Format** — MCAP (domain-correct, Foxglove free) or Parquet+DuckDB (stays
   in the dataframe world)? Mostly a question of which analysis environment you
   want to live in.
3. **Does viz move out of process** eventually (§8.4), or stay embedded? Only
   affects how much the sink abstraction must carry.
4. **Is the golden-trace payoff (§8.2) a goal or a side effect?** If a goal,
   determinism constrains T0–T1 *now* — cycle-index keying, no wall-clock
   dependence in the compared stream — rather than being retrofitted.
5. **Encoding** — `postcard` (Rust-native, `no_std`-friendly) or CDR (ROS 2 /
   Foxglove-native interop, if that ever matters)?

---

## 11. Constraints this must not violate

Restated because a telemetry layer is *typically* where each gets broken:

- **`motion-core` stays I/O-free and dependency-free.** No serialisation, no
  transport, no telemetry types. If a derived quantity is wanted in the record,
  it is computed where it already lives and captured at the seam.
- **`axis-backend` stays a pure seam definition.** The recorder is a *decorator
  over* the trait, never a change to it.
- **The recorder is an observer.** Nothing in it may feed back into planning —
  same reasoning as commanded-not-feedback: the moment a plan depends on the
  record, there is a path that cannot be reasoned about.
- **No allocation, no I/O, no blocking in the per-cycle hot path.**
- **Loop behaviour identical with recording on or off** — the property
  `RecordingAxisGroup` has today and must keep.
- **No DB client in the control process on hardware.** Ship bytes out; process
  elsewhere.
