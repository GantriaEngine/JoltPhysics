# Soft body constraint delta velocity reconstruction

`SoftBodyCreationSettings::mVelocityReconstruction` selects the method used to
reconstruct vertex velocity after each soft body solver sub step. Its default,
`ESoftBodyVelocityReconstruction::PositionDifference`, retains Jolt's original
`(position - previous_position) / sub_step_delta_time` calculation. Applications
can opt individual bodies into `ConstraintDelta` before creating them. The mode
is stored in the body's motion properties and in creation settings object and
binary serialization.

In `ConstraintDelta` mode, integration stores each vertex's velocity after
gravity, force, and damping, then clears a per-vertex displacement accumulator.
Each pre-reconstruction position constraint adds its intended displacement.
Velocity reconstruction uses the stored integrated velocity plus the accumulated
displacement divided by the sub step duration. Edge and tetrahedral volume
constraints use their correction vectors directly. Dihedral bends and rod
stretch/shear constraints do the same. Skin back-stop and distance clamps, and
long-range attachment, accumulate their scale-based correction directly while
retaining their existing target position assignment. A kinematic skin target
uses the target minus the current position as its displacement. Rod bend/twist changes rod
orientation, not vertex position, and retains its existing angular velocity
update.

The alternative avoids interpreting float quantization of the predicted
position as velocity. The source case is a vertex at `x = 2f` with speed
`0.003f` and a sub step of `1 / (60 * 384)` seconds: one position ULP exceeds
the intended movement, so position-difference reconstruction inflates speed.
The edge and volume path was qualified in the originating numerical experiment
with positive compliance at two world updates, eight collision steps per
update, and twelve soft iterations per collision step. The other constraint
families have implementation coverage but have not received that same
long-horizon physical qualification. The mode does not make exact constraints
energetically stable in all configurations.

Unit tests exercise the default mode, a float quantization case, edge and
volume correction accumulation, dihedral bending, skin clamping, rod stretch,
long-range attachment, and reconstruction of creation settings from a body.

Contact position projection occurs after velocity reconstruction, as before,
and deliberately does not enter the current sub step's accumulator. The
subsequent contact impulse/friction/restitution velocity logic is unchanged.
Recentering occurs after the solver iteration and also does not enter the
accumulator. The accumulator resets every sub step, so replay state continues
to save the persistent position and velocity state; no new replay fields are
required. Binary creation settings written by this fork include the mode field,
and readers of the earlier unversioned binary layout cannot consume those new
streams.
