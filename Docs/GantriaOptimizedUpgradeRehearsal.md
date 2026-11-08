# Optimized candidate upgrade rehearsal

This branch carries the opt-in numerical reconstruction, retained-lambda
constraint sweeps, bounded rigid-to-soft attachment helper, and full-update
sleep clock from the v5.6.0 candidate
`d8c46588b23700d24ebdaeada0ff460d34b1f830` onto upstream
`0837f118d7248579789a94835407de2d56a93be9`. It starts from the earlier
foundation upgrade rehearsal `fe32a47ca3c1d8c0868c8659b489e607da866fbb`
and replays the eleven subsequent candidate commits in order. The v5.6.0
candidate and the earlier upgrade rehearsal remain separate.

The earlier foundation merge required two manual conflict resolutions,
documented in `GantriaUpgradeRehearsal.md`. This extension required **one
additional manual conflict**, in `Jolt/Physics/PhysicsSettings.h` while applying
the sleep-clock commit. Upstream removed `mDeterministicSimulation`; the
v5.6.0 sleep patch used that field as context for its insertion. The resolution
adds only `mUseFullUpdateDeltaForSleep = false` after the unchanged sleep
threshold and does not restore the removed upstream field. Other cherry-picks
applied without manual conflicts. `git range-diff` confirms the implementation
commits retain their patch content, with context-only differences in the unit
test source list and the sleep settings header.

Before this document, the complete branch diff from upstream changed 28 files
with 1,545 inserted and 9 deleted lines. Fourteen runtime files under
`Jolt/Physics/` account for 487 inserted and 9 deleted lines. The library build
list adds 2 lines; five unit-test files add 798 lines; six prior documents add
154 lines; and two CI files add 104 lines. These counts include the foundation
patch and all subsequent candidate changes, not upstream changes.

The public API additions are `PhysicsSettings::mUseFullUpdateDeltaForSleep`,
`SoftBodyCreationSettings::mNumConstraintIterations` and
`mVelocityReconstruction`, and the `SoftBodyRigidAttachment` helper with its
bounded settings and status. Defaults preserve upstream behavior. The
unversioned `SoftBodyCreationSettings` binary stream adds a `uint32` sweep
count and a one-byte velocity mode after `mNumIterations`, shifting subsequent
fields by five bytes. The old and new layouts are not mutually readable.

This is a source and unit-test maintenance rehearsal. Passing its CI does not
transfer the v5.6.0 native, unified, provider, or performance qualification to
the newer upstream commit.
