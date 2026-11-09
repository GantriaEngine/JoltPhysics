# Cloth contact order experiment

This branch starts at the qualified native `7e59df5f` source. All alternatives
are private environment gates. With no gate, the original structural and
contact operations execute in their original order. The benchmark fixture,
authored material, and Engine public API are separate from this source.

| `GANTRIA_CONTACT_ORDER` | Per temporal integration after qualified Q2 structural sweeps |
| --- | --- |
| unset (A) | Original final contact projection and velocity update. |
| `B` | A, then one complete structural pass if a vertex penetrated its contact plane. Velocity was updated before this pass, so B is an ordering diagnostic, not an accepted physical update. |
| `C` | Position-only contact projection, complete structural pass, then original final contact/velocity update. Repeat the contact/structure pair with `GANTRIA_CONTACT_PASSES=1..4`; default 1. |
| `D` | Unilateral hard-contact multiplier projection alternating with complete structural passes, then original final contact/velocity update. The multiplier lives for one temporal integration. Default 4 passes; `GANTRIA_CONTACT_PASSES=1..4` allows matched C/D pass counts. Dynamic rigid contacts retain A handling; D does not pretend to apply a coupled rigid position correction. |
| `E` | Refresh vertex contact planes from post-structure positions before A's final contact update. Existing colliding shape transforms remain fixed for that soft update. Any CCD marker leaves the old plane path intact. No out-of-plane tie-break is imposed. |
| `F` | D's unilateral contact/structure alternations, followed by a final multiplier projection. The one final velocity update treats retained normal support as active contact and uses its equivalent normal correction distance for the existing friction cap. `GANTRIA_CONTACT_PASSES=1..4` allows F1/F4 controls. Dynamic contacts retain A. |

`GANTRIA_CONTACT_KINEMATIC_RELATIVE=1` is a separate G velocity-response
switch, combinable with A or F. On kinematic contacts only, it calculates
friction, normal response, and restitution from cloth velocity relative to the
body's linear and angular point velocity, then applies the resulting velocity
change to the cloth. Static contacts retain the original arithmetic and dynamic
contacts retain their original coupled impulse path. Without the switch, the
qualified default response remains unchanged. The switch does not change
position projection or infer motion from teleported poses.

C and D continue the retained structural multipliers after Q2, without another
temporal integration. Their extra passes stop when the preceding contact phase
made no position or multiplier change. No-contact negative controls therefore
receive no added structural pass. E is a test of stale tangent-plane effects,
not a symmetry-breaking mechanism. Because the authored cloth and sphere are
both exactly in the XY plane, the tested radial normals and edge corrections
cannot select a Z sign. A winding-based sign would introduce an explicit
one-sided material policy and is outside this double-sided solver experiment.

C and D are not yet physical fixes: their intermediate position-only contact
steps remove penetration before the original final friction calculation. That
calculation limits friction from the final residual projection distance, so it
does not account for earlier contact correction or D's retained contact
multiplier. A lower strain or zero penetration result cannot be accepted until
normal impulse, friction, restitution, and external-work accounting are made
consistent and independently qualified.
F repairs this specific missing-support path for static/kinematic contacts,
but its multiplier/friction mapping remains experimental. It does not make
static pose jumps have a measurable actuator work term or solve the planar
deformation by itself. Full trace and contact/momentum gates decide acceptance.

`GANTRIA_CONTACT_STAGES=1` on a single 128×128 cloth logs
`[Physics:ContactStage]` CSV rows at precontact, intermediate contact,
poststructure, final contact, and refreshed-plane stages as applicable. Each
row contains signed axial extrema, the winning constraint indices, maximum
linearized plane penetration, finite stored position/velocity counts, stored
velocity kinetic energy, and position displacement since the previous stage.
The separate `[Physics:ContactVelocityImpulse]` row reports the summed
velocity-domain change in cloth particle momentum, summed dynamic-rigid
solver impulse, and cloth kinetic-energy delta during the final contact
velocity step. The kinematic column records the directly calculated dot product
of cloth velocity impulse and prescribed surface velocity, for either G setting.
It is only velocity-domain actuator work; position projection displacement is
**not** an impulse.
Linearized plane penetration is **not** sphere/triangle penetration. Stored
precontact velocity has not yet been reconstructed for that integration.
Static-pose actuator work and full energy closure are unavailable from these
native rows and must be marked incomplete by the independent physical trace.

These diagnostic scans materially distort runtime. Use uninstrumented, matched
quality runs to assess performance. Native unit smoke tests cover every mode,
finite contact state, and exact no-contact parity. The independent Engine
`ContactFixtureV1` and `ContactTraceV1` protocol supplies the broader symmetric,
asymmetric, dynamic, worker-count, penetration, energy, and replay gates.
