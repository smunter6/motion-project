# Development plans

Design plans, one per feature. These record the *reasoning before* a piece of
work — the options weighed, the tradeoffs accepted, the questions left open for
the user to settle.

**They are not the authority on what the code does.** Once work ships, the
design rationale moves to `CLAUDE.md` and to the source; a shipped plan is kept
only for what it uniquely records — chiefly **where the plan and the
implementation diverged, and why**. Read the code for *what*; read `CLAUDE.md`
for *why it looks that way*; read these for *what we were thinking at the time*.

These lived in `~/.claude/plans/` until 2026-07-29, outside the repo and under
auto-generated names (`eager-singing-aurora.md` and the like). That location was
global rather than per-project, invisible to any search of the tree, and not
versioned alongside the code the plans describe — so a plan's claims about a
commit couldn't travel with that commit. Moved here and renamed for content.

## Live

| plan | status |
| --- | --- |
| [kinematics-limit-scheduling.md](kinematics-limit-scheduling.md) | Rolling. Phase 1 (kinematics seam + SCARA) **shipped** 2026-07-26; phase 2, **unified limit scheduling**, is planned in full and not started. Carries four open questions for the user, the largest being whether the speed envelope applies to `LinearMove` as well as `PathProfile`. This is roadmap item 7. |
| [telemetry-external-record.md](telemetry-external-record.md) | **Nothing built.** Externalising `app/src/recording.rs`'s in-memory tap into a durable, off-box record: a flight recorder (MCAP) rather than a metrics TSDB, with Zenoh as transport only. Sequenced after phase 2, though it argues its own first two steps may be worth pulling forward. Five open questions. |

## Shipped

Kept for their divergence records, not as documentation of current behaviour.

| plan | status |
| --- | --- |
| [buffermode-nonzero-start.md](buffermode-nonzero-start.md) | **Shipped.** Extending `TrapezoidalProfile` to start from a nonzero velocity, plus PLCopen `BufferMode` (`Aborting`/`Buffered`) and a real move queue. The user's explicit call here — extend `TrapezoidalProfile` rather than add a second type beside it, the way `StopRamp` sits — is a precedent `CLAUDE.md` still cites. |
| [axis-groups.md](axis-groups.md) | **Shipped.** Hard-coded axis groups (`axisGroup0`) and `LinearMove`, deliberately narrower than PLCopen's runtime axis-group model. Its "Context" section supersedes an earlier full `MC_MovePath` spline plan that had occupied the same file and was shelved at the time — that feature was subsequently built, so treat the shelving note as historical. |

## Convention

New plans belong here, named for their content. Plan mode writes to
`~/.claude/plans/` with a generated slug name by default; move it here and
rename it once the plan stabilises. Amend a plan in place as decisions land,
compressing shipped phases down to their divergences rather than leaving a
second copy of rationale that `CLAUDE.md` now owns.
