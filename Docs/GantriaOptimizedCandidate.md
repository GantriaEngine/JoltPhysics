# Opt-in physics candidate: qualification ledger

This document records the tested v5.6.0 candidate at
`d8c46588b23700d24ebdaeada0ff460d34b1f830`, based on upstream
`e77f175595e64cb44218cc9d9d56fc365ad0e36a`. The patch adds four
independent, opt-in capabilities:

* `SoftBodyCreationSettings::mVelocityReconstruction` can reconstruct velocity
  from damped velocity plus intended constraint corrections instead of
  subtracting rounded positions.
* `SoftBodyCreationSettings::mNumConstraintIterations` can run two, four, or
  eight retained-lambda edge/volume sweeps per temporal substep.
* `SoftBodyRigidAttachment` couples a movable soft vertex to a dynamic rigid
  local point through bounded, opposite impulses and torque.
* `PhysicsSettings::mUseFullUpdateDeltaForSleep` credits the full update
  interval to the sleep test on the final collision step.

The upstream position-difference reconstruction, one constraint sweep, and
sleep clock remain the defaults. No attachment is registered automatically.
Multi-sweep admission rejects dihedral, skin, LRA, and rod graphs before body
allocation; the default one-sweep path still accepts them. The attachment
helper retains its explicit ownership, body-ID lifetime, step and impulse
bounds described in `GantriaDynamicAttachmentFeasibility.md`.

## Tested source and scope

| Evidence | Exact source | Result | Scope |
| --- | --- | --- | --- |
| Jolt unit CI | `d8c46588` | Windows Release, Linux Release, and Linux ASAN/UBSAN passed (run `42478036231`) | Numerical modes, unsupported admission, sleep timing and wake, attachment impulse and lifetime fixtures |
| Native physical profiles | `d8c46588` | Linux and Windows each passed 24 positive cases, 2 expected energy-control rejections, 6 timing parity checks, and 2 float32 spring calibrations (run `42482803195`) | U2/C8/I5/Q2 opt-in candidate and same-binary Q1 controls at selected cases |
| Unified physical profiles | `d8c46588` | Linux and Windows each passed 45 paired, 3 extended and 2 negative-control cases; all 50 receipt gates passed (same run) | Finite contact, pin, geometry, energy and observer-parity fixtures |
| Cross-platform receipts | `d8c46588` | Named frame-state SHA-256 digests, native score dictionaries and unified quality summaries match between Linux and Windows | Exact tested cases and build identities |

The opt-in convergence profile uses positive compliance `1e-6`, 80 temporal
integrations per frame, two constraint sweeps, two world updates, eight
collision steps per update, and five soft-body iterations per collision step.
Calibration reports zero float32 position and velocity replay error. The
physical run includes the full-update sleep clock. Passing these fixtures is
evidence for this configuration and finite scene set, not general energy
conservation or stability for arbitrary constraints, timesteps or contacts.
The attachment helper has its own dynamic-load, moving-anchor, torque,
break, removal and sequence-reuse unit fixtures; the physical profiles are
not a replacement for those fixtures.

## Maintenance and compatibility

Relative to upstream `e77f1755`, the qualified source pin contains 19 commits
and changes 27 files (+1,499/−8 lines). Fifteen files under `Jolt/` account for
+487/−8 lines, including the two-line `Jolt.cmake` registration. The 14
runtime C++/header files account for +485/−8 lines; five unit-test files add
798 lines. These predicates are distinct from the newer-upstream rehearsal's
diff counts.

The creation-settings binary stream has no internal version marker. The fork
adds a four-byte constraint-sweep count and a one-byte velocity mode, shifting
subsequent fields by five bytes. Old and new binary creation-settings streams
are not mutually readable without an external versioned wrapper. Callers
should rebuild serialized settings or provide such a wrapper when migrating.

The separate upgrade rehearsal at `c50f06fdb7db27c1a527b97251411ba63d2867a0`
replays this patch set onto newer upstream
`0837f118d7248579789a94835407de2d56a93be9`. It needed one additional
manual header conflict after the earlier two-conflict foundation rehearsal,
and its Windows Release, Linux Release and Linux sanitizer unit CI passed (run
`42481427201`). That result checks source maintenance and unit behavior; the
v5.6.0 physical-profile results above do not transfer to the newer upstream
revision.
