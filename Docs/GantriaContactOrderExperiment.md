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
| `H` | After A's vertex response, project closest cloth triangle points out of collected static/kinematic sphere leaves using barycentric inverse-mass weights. Dynamic sphere contacts remain A. H is a geometry-only diagnostic: face correction does not reconstruct cloth velocity, apply face friction, or react on the rigid body. |
| `I` | H's face projection followed by an actual barycentric face-point velocity impulse for static/kinematic spheres. Cancels approaching relative normal speed and caps tangential impulse using friction times the larger of the normal velocity impulse and projection-depth support impulse. Dynamic sphere contacts remain A. |

`GANTRIA_CONTACT_KINEMATIC_RELATIVE=1` is a separate G velocity-response
switch, combinable with A or F. On kinematic contacts only, it calculates
friction, normal response, and restitution from cloth velocity relative to the
body's linear and angular point velocity, then applies the resulting velocity
change to the cloth. Its normal response cancels approaching relative speed
only; a separating cloth vertex does not adhere to a receding surface. Static
contacts retain the original arithmetic and dynamic
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
F's intermediate positional contact projections do not enter the
`ConstraintDelta` velocity numerator. This preserves the baseline policy of
resolving contact velocity in the final response, while structural corrections
do enter that numerator. For a fixed per-vertex plane, the retained contact
projection is normal to the plane. G's unilateral response can preserve
separating relative speed, so F+G full-velocity traces must be qualified before
any physical adoption; these rows do not establish energy closure.
H tests the geometric gap in vertex-only sphere contact. It uses the face's
oriented normal when the sphere center is exactly on its closest face point.
That is an **explicit winding-based side policy** under exact coplanar
symmetry, not a derived double-sided physical solution. H is not a qualified
solver fix. `GANTRIA_CONTACT_FACE_DIAGNOSTIC=1` emits correction counts; the
log marks face velocity response and rigid reaction incomplete. It must be
compared with the independent full trace's triangle penetration, edge strain,
and energy gates before drawing any conclusion about geometry.
I adds velocity response to that same geometry probe. For barycentric weights
`b`, its inverse effective mass is `Σ(b²/m)`. The normal velocity impulse is
unilateral; the friction limit uses `μ max(J_normal, depth/(W dt))`, where the
second term is the position projection's impulse-equivalent normal support.
Each vertex receives its share `inverse_mass × b × J`. The computed opposite
reaction is logged for static/kinematic bodies but is not applied to their
prescribed motion. The logger also sums the actual cloth kinetic-energy change
and `J·v_surface` for kinematic contacts. This closes only the velocity impulse
account; geometry projection, elastic-energy change, and externally imposed
static pose jumps still require separate accounting. Exact center-on-face
contact retains H's explicit winding-based side tie, so I is also experimental
and not a double-sided physical solution by itself.

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
