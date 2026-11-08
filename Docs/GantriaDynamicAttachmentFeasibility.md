# Dynamic rigid-to-soft attachment feasibility

`SoftBodyRigidAttachment` is an optional Jolt helper for one movable soft-body vertex and one dynamic rigid body. It lives on the `gantria/dynamic-attachments` branch based on qualified `109be79f`; it does not change the qualified solver or the default Jolt path. It uses `PhysicsStepListener`, so it needs no callback from a game executable and no per-vertex solver mode query.

## Timing and ownership

`OnStep` runs before every collision step, after the previous step has finished and before rigid gravity and soft-body simulation. Jolt holds all body and constraint mutexes for this phase. The listener reads positions and writes only velocities. It does not create, remove, or reposition bodies. It uses `BodyInterfaceNoLock` in the callback, because taking ordinary locks inside this phase would deadlock. A full `BodyID` with sequence number is resolved on each step; a removed or reused body is rejected before any impulse.

The caller must own the helper, call `Connect` after adding both bodies, and call `Disconnect` or destroy it **between** `PhysicsSystem::Update` calls, before destroying the physics system. It must not edit the helper, read its status, or remove a body while an update is running. Only one listener may write either participating body at a time; independent listeners touching the same bodies can race because Jolt runs listeners in parallel. `Connect` activates both bodies. Broken, missing, and invalid attachments retain the listener registration but perform no further work until disconnected. No persistent attachment state is serialized with the bodies.

## Coupling law

The rigid anchor is given in center-of-mass local coordinates and follows the rigid body's orientation. The soft point is the selected live vertex in the soft body's recentered local frame. Each step computes relative anchor position and point velocity, then an effective inverse mass that includes the soft vertex inverse mass, rigid translation, and rigid angular inertia at the lever arm. The desired velocity change contains a bounded fraction of positional error plus relative-velocity damping. Compliance adds `compliance / dt²` to the inverse effective mass. The resulting impulse is capped, added to the soft vertex velocity in its local frame, and applied with opposite sign at the rigid world anchor. Jolt's rigid `AddImpulse` generates the corresponding torque. This transfers momentum in both directions without pinning the soft vertex or teleporting either body.

Settings limit stiffness to `(0, 0.5]`, damping to `[0, 1]`, impulse per collision step, break distance, step duration, and collision steps per update. A break-distance violation applies no impulse. A step-count violation stops future work after the configured limit. These bounds are local stability controls, not a proof that every geometry or timestep is stable. A caller should use a finite, physically appropriate impulse cap and inspect the fixture's displacement and energy over its operating range.

This is a **velocity-level** attachment. A soft body's own edge/volume/contact solves can move the vertex after the paired impulse, and rigid contact solves can change the rigid response. Consequently exact pairwise momentum at the attachment application does not imply exact momentum of the whole system after gravity, collision, or external forces. The helper does not add a position constraint to Jolt's island solver, and high-stiffness hard welding is outside this feasibility API.

## Fixtures

`SoftBodyRigidAttachmentTests.cpp` covers initial paired impulse and momentum, a 90-frame loaded edge pulling a dynamic rigid box, moving rigid point transferring velocity into the soft vertex, off-center torque, break-before-impulse, removed body and reused sequence rejection, step limit, and invalid settings/pinned vertex rejection. The full CI test matrix and a bounded native worker build are required before declaring the branch qualified. This file records source design and expected behavior; it is not a runtime qualification result.
