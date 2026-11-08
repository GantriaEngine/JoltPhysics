# Full update sleep clock (opt-in)

`PhysicsSettings::mUseFullUpdateDeltaForSleep` defaults to `false` to preserve
the upstream sleep timing. Set it to `true` before calling `PhysicsSystem::Update`
to measure the sleep interval using the full update delta time.

`PhysicsSystem::Update` divides its delta time by the number of collision steps.
`CheckSleepAndUpdateBounds` calls `Body::UpdateSleepStateInternal` only on the
last collision step. The original call passes one collision step's delta time,
so the sleep timer advances by `UpdateDt / CollisionSteps` for each complete
update. With this option enabled, the call passes the exact `UpdateDt` saved in
the update context. The three sleep-test points and their displacement limit
(`mPointVelocitySleepThreshold * mTimeBeforeSleep`) are unchanged. The clock
now covers the same elapsed interval as that displacement limit.

This option changes when active bodies become eligible to sleep if an update
contains more than one collision step. It does not change contact resolution,
body activation, or the configured velocity and time thresholds. Callers should
set physics settings between updates; concurrent settings changes are not
supported by `PhysicsSystem`.
