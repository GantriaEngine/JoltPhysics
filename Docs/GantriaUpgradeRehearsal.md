# Gantria physics upgrade rehearsal

This branch rehearses carrying the opt-in soft body velocity reconstruction
change from Jolt v5.6.0 (`e77f175595e64cb44218cc9d9d56fc365ad0e36a`)
to upstream `0837f118d7248579789a94835407de2d56a93be9`. Its merge commit
`a0743cc3a5644ed7fa1a8ed2e880dca6b57383b0` has the upstream commit and
the maintained v5.6.0 branch commit
`109be79fa82be0b306c56e4b4357675ed7843767` as parents. The maintained
v5.6.0 branch remains separate.

Relative to the upstream commit, the merge changes 10 files with 490 inserted
and 3 deleted lines. Six runtime source files account for 73 inserted and 3
deleted lines: `Body.cpp`, `SoftBodyCreationSettings.h/.cpp`,
`SoftBodyMotionProperties.h/.cpp`, and `SoftBodyVertex.h`. One unit test file
adds 260 lines, one design document adds 53 lines, and two CI files add 104
lines. The merge does not modify Engine source.

Two source conflicts required decisions:

1. Upstream changed the LRA position assignment to use
   `MulRSqrtApproximate(max_distance, distance_squared)`. The rehearsal keeps
   that upstream scale for the position assignment and adds the alternative
   velocity's intended correction as `delta * (scale - 1)`. The v5.6.0 branch
   continues to use its original square-root division. This can change LRA
   trajectories, so v5.6.0 qualification does not certify this upgrade.
2. Upstream added soft body unit test helpers and cases at the start of
   `SoftBodyTests.cpp`, where the maintained branch also added tests. The
   rehearsal keeps the upstream helpers and cases followed by the new
   numerical tests.

The fork adds one `uint8` reconstruction mode immediately after
`mNumIterations` in `SoftBodyCreationSettings::SaveBinaryState` and
`RestoreBinaryState`. These unversioned binary creation-setting streams are
not mutually readable with the earlier layout: every following field moves
by one byte. The object stream also gains a named enum attribute represented
as `uint32`. Two `Vec3` values are added to each runtime soft body vertex for
the integrated velocity and constraint accumulator; neither is part of the
manual replay state stream, because each solver sub step resets them. Replay
therefore requires the body to be created with the same mode, and this
document makes no cross-version replay compatibility claim.

This is a source merge and review checkpoint. Build, unit tests, sanitizers,
and physical qualification against the upstream commit must pass before the
upgrade is treated as qualified.
