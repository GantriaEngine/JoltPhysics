// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2026 Gantria
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/PhysicsStepListener.h>
#include <Jolt/Math/Vec3.h>

JPH_NAMESPACE_BEGIN

class PhysicsSystem;

/// Opt-in, velocity-level coupling of one movable soft-body vertex to a point on a dynamic rigid body.
/// Own one instance per pair. Connect, Disconnect and destruction are only allowed between PhysicsSystem::Update calls.
/// No other step listener may write either body concurrently. The PhysicsSystem must outlive this object.
class JPH_EXPORT SoftBodyRigidAttachment final : public PhysicsStepListener
{
public:
	struct Settings
	{
		float		mStiffness = 0.1f;				///< Fraction of position error corrected per collision step, (0, 0.5].
		float		mVelocityDamping = 0.5f;		///< Fraction of relative point velocity removed, [0, 1].
		float		mCompliance = 0.0f;			///< Additional inverse effective mass, in m / N, >= 0.
		float		mMaxImpulse = 1.0f;			///< Per-step impulse limit in kg m / s, > 0.
		float		mBreakDistance = 2.0f;		///< Detach before applying an impulse if separation exceeds this, > 0.
		float		mMaxDeltaTime = 1.0f / 30.0f; ///< Largest supported collision step, > 0.
		uint32		mMaxStepsPerUpdate = 16;		///< Fail closed if an Update requests more collision steps, [1, 16].
	};

	enum class EStatus : uint8
	{
		Disconnected,
		Active,
		Broken,
		BodyMissing,
		StepLimitExceeded,
		InvalidState
	};

							~SoftBodyRigidAttachment() override;

	/// inRigidLocalPoint is relative to the rigid center of mass. Both IDs include their generation.
	/// All validation occurs before registering the listener or mutating either body.
	/// Disconnect an existing connection before reconnecting.
	bool					Connect(PhysicsSystem &inSystem, BodyID inSoftBodyID, uint inVertexIndex,
							BodyID inRigidBodyID, Vec3Arg inRigidLocalPoint, const Settings &inSettings);
	void					Disconnect();

	EStatus					GetStatus() const				{ return mStatus; }
	Vec3					GetLastImpulseOnSoftBody() const { return mLastImpulseOnSoftBody; }
	uint64					GetAppliedStepCount() const { return mAppliedStepCount; }

	void					OnStep(const PhysicsStepListenerContext &inContext) override;

private:
	PhysicsSystem *			mSystem = nullptr;
	BodyID					mSoftBodyID;
	BodyID					mRigidBodyID;
	uint					mVertexIndex = 0;
	Vec3					mRigidLocalPoint = Vec3::sZero();
	Settings				mSettings;
	EStatus					mStatus = EStatus::Disconnected;
	Vec3					mLastImpulseOnSoftBody = Vec3::sZero();
	uint64					mAppliedStepCount = 0;
	uint32					mStepsThisUpdate = 0;
};

JPH_NAMESPACE_END
