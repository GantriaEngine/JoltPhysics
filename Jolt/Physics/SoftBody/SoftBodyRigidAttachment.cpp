// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2026 Gantria
// SPDX-License-Identifier: MIT

#include <Jolt/Jolt.h>
#include <Jolt/Physics/SoftBody/SoftBodyRigidAttachment.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Core/Mutex.h>
#include <cmath>

JPH_NAMESPACE_BEGIN

namespace
{
	struct AttachmentClaim
	{
		PhysicsSystem *mSystem;
		BodyID mSoftBodyID;
		BodyID mRigidBodyID;
	};

	Mutex sAttachmentClaimsMutex;
	AttachmentClaim sAttachmentClaims[1024];
	uint32 sAttachmentClaimCount = 0;
}

SoftBodyRigidAttachment::~SoftBodyRigidAttachment()
{
	Disconnect();
}

bool SoftBodyRigidAttachment::Connect(PhysicsSystem &inSystem, BodyID inSoftBodyID, uint inVertexIndex,
	BodyID inRigidBodyID, Vec3Arg inRigidLocalPoint, const Settings &inSettings)
{
	if (mSystem != nullptr)
		return false;

	if (inSoftBodyID == inRigidBodyID
		|| !std::isfinite(inSettings.mStiffness) || inSettings.mStiffness <= 0.0f || inSettings.mStiffness > 0.5f
		|| !std::isfinite(inSettings.mVelocityDamping) || inSettings.mVelocityDamping < 0.0f || inSettings.mVelocityDamping > 1.0f
		|| !std::isfinite(inSettings.mCompliance) || inSettings.mCompliance < 0.0f || inSettings.mCompliance > 1000.0f
		|| !std::isfinite(inSettings.mMaxImpulse) || inSettings.mMaxImpulse <= 0.0f || inSettings.mMaxImpulse > 1000.0f
		|| !std::isfinite(inSettings.mBreakDistance) || inSettings.mBreakDistance <= 0.0f || inSettings.mBreakDistance > 10000.0f
		|| !std::isfinite(inSettings.mMaxDeltaTime) || inSettings.mMaxDeltaTime <= 0.0f || inSettings.mMaxDeltaTime > 1.0f / 30.0f
		|| inSettings.mMaxStepsPerUpdate == 0 || inSettings.mMaxStepsPerUpdate > 16
		|| !std::isfinite(inRigidLocalPoint.GetX()) || !std::isfinite(inRigidLocalPoint.GetY()) || !std::isfinite(inRigidLocalPoint.GetZ()))
		return false;

	{
		BodyLockRead soft_lock(inSystem.GetBodyLockInterface(), inSoftBodyID);
		if (!soft_lock.SucceededAndIsInBroadPhase())
			return false;

		const Body &soft_body = soft_lock.GetBody();
		if (!soft_body.IsSoftBody())
			return false;
		const SoftBodyMotionProperties *properties = static_cast<const SoftBodyMotionProperties *>(soft_body.GetMotionProperties());
		if (inVertexIndex >= properties->GetVertices().size() || properties->GetVertex(inVertexIndex).mInvMass <= 0.0f)
			return false;
	}
	{
		BodyLockRead rigid_lock(inSystem.GetBodyLockInterface(), inRigidBodyID);
		if (!rigid_lock.SucceededAndIsInBroadPhase())
			return false;
		const Body &rigid_body = rigid_lock.GetBody();
		if (!rigid_body.IsRigidBody() || !rigid_body.IsDynamic()
			|| rigid_body.GetMotionProperties()->GetAllowedDOFs() != EAllowedDOFs::All)
			return false;
	}
	{
		lock_guard lock(sAttachmentClaimsMutex);
		if (sAttachmentClaimCount >= 1024)
			return false;
		for (uint32 i = 0; i < sAttachmentClaimCount; ++i)
		{
			const AttachmentClaim &claim = sAttachmentClaims[i];
			if (claim.mSystem == &inSystem
				&& (claim.mSoftBodyID == inSoftBodyID || claim.mSoftBodyID == inRigidBodyID
					|| claim.mRigidBodyID == inSoftBodyID || claim.mRigidBodyID == inRigidBodyID))
				return false;
		}
		sAttachmentClaims[sAttachmentClaimCount++] = { &inSystem, inSoftBodyID, inRigidBodyID };
	}

	mSystem = &inSystem;
	mSoftBodyID = inSoftBodyID;
	mRigidBodyID = inRigidBodyID;
	mVertexIndex = inVertexIndex;
	mRigidLocalPoint = inRigidLocalPoint;
	mSettings = inSettings;
	mStatus = EStatus::Active;
	mLastImpulseOnSoftBody = Vec3::sZero();
	mAppliedStepCount = 0;
	mStepsThisUpdate = 0;
	inSystem.AddStepListener(this);
	inSystem.GetBodyInterface().ActivateBody(inSoftBodyID);
	inSystem.GetBodyInterface().ActivateBody(inRigidBodyID);
	return true;
}

void SoftBodyRigidAttachment::Disconnect()
{
	if (mSystem != nullptr)
	{
		mSystem->RemoveStepListener(this);
		lock_guard lock(sAttachmentClaimsMutex);
		for (uint32 i = 0; i < sAttachmentClaimCount; ++i)
			if (sAttachmentClaims[i].mSystem == mSystem && sAttachmentClaims[i].mSoftBodyID == mSoftBodyID
				&& sAttachmentClaims[i].mRigidBodyID == mRigidBodyID)
			{
				sAttachmentClaims[i] = sAttachmentClaims[--sAttachmentClaimCount];
				break;
			}
	}
	mSystem = nullptr;
	mStatus = EStatus::Disconnected;
	mLastImpulseOnSoftBody = Vec3::sZero();
}

void SoftBodyRigidAttachment::OnStep(const PhysicsStepListenerContext &inContext)
{
	if (mStatus != EStatus::Active)
		return;
	if (inContext.mIsFirstStep)
		mStepsThisUpdate = 0;
	if (++mStepsThisUpdate > mSettings.mMaxStepsPerUpdate
		|| !std::isfinite(inContext.mDeltaTime) || inContext.mDeltaTime <= 0.0f
		|| inContext.mDeltaTime > mSettings.mMaxDeltaTime)
	{
		mStatus = EStatus::StepLimitExceeded;
		return;
	}

	// The physics system has locked all bodies for this phase. Full BodyIDs reject removed/reused slots.
	const BodyLockInterfaceNoLock &body_lock = inContext.mPhysicsSystem->GetBodyLockInterfaceNoLock();
	Body *soft_body = body_lock.TryGetBody(mSoftBodyID);
	Body *rigid_body = body_lock.TryGetBody(mRigidBodyID);
	if (soft_body == nullptr || rigid_body == nullptr || !soft_body->IsInBroadPhase() || !rigid_body->IsInBroadPhase())
	{
		mStatus = EStatus::BodyMissing;
		return;
	}
	if (!soft_body->IsSoftBody() || !rigid_body->IsRigidBody() || !rigid_body->IsDynamic())
	{
		mStatus = EStatus::InvalidState;
		return;
	}

	SoftBodyMotionProperties *properties = static_cast<SoftBodyMotionProperties *>(soft_body->GetMotionProperties());
	if (mVertexIndex >= properties->GetVertices().size())
	{
		mStatus = EStatus::InvalidState;
		return;
	}
	SoftBodyVertex &vertex = properties->GetVertex(mVertexIndex);
	if (!std::isfinite(vertex.mInvMass) || vertex.mInvMass <= 0.0f)
	{
		mStatus = EStatus::InvalidState;
		return;
	}

	RVec3 soft_point = soft_body->GetCenterOfMassTransform() * vertex.mPosition;
	RVec3 rigid_point = rigid_body->GetCenterOfMassTransform() * mRigidLocalPoint;
	Vec3 separation = Vec3(rigid_point - soft_point);
	if (separation.LengthSq() > Square(mSettings.mBreakDistance))
	{
		mStatus = EStatus::Broken;
		return;
	}

	Vec3 relative_velocity = rigid_body->GetPointVelocity(rigid_point) - soft_body->GetRotation() * vertex.mVelocity;
	Vec3 target_velocity = mSettings.mStiffness * separation / inContext.mDeltaTime + mSettings.mVelocityDamping * relative_velocity;
	Vec3 lever_arm = Vec3(rigid_point - rigid_body->GetCenterOfMassPosition());
	Mat44 cross = Mat44::sCrossProduct(lever_arm);
	Mat44 inverse_effective_mass = cross.Multiply3x3(rigid_body->GetInverseInertia()).Multiply3x3RightTransposed(cross);
	inverse_effective_mass += Mat44::sScale(vertex.mInvMass + rigid_body->GetMotionProperties()->GetInverseMass() + mSettings.mCompliance / Square(inContext.mDeltaTime));
	Mat44 effective_mass;
	if (!effective_mass.SetInversed3x3(inverse_effective_mass))
	{
		mStatus = EStatus::InvalidState;
		return;
	}
	Vec3 impulse = effective_mass.Multiply3x3(target_velocity);
	if (!std::isfinite(impulse.GetX()) || !std::isfinite(impulse.GetY()) || !std::isfinite(impulse.GetZ()))
	{
		mStatus = EStatus::InvalidState;
		return;
	}
	float impulse_length_sq = impulse.LengthSq();
	if (!std::isfinite(impulse_length_sq))
	{
		mStatus = EStatus::InvalidState;
		return;
	}
	if (impulse_length_sq > Square(mSettings.mMaxImpulse))
		impulse *= mSettings.mMaxImpulse / std::sqrt(impulse_length_sq);

	Vec3 soft_velocity_delta = soft_body->GetRotation().Conjugated() * (vertex.mInvMass * impulse);
	Vec3 next_soft_velocity = vertex.mVelocity + soft_velocity_delta;
	if (!std::isfinite(next_soft_velocity.GetX()) || !std::isfinite(next_soft_velocity.GetY()) || !std::isfinite(next_soft_velocity.GetZ()))
	{
		mStatus = EStatus::InvalidState;
		return;
	}
	const MotionProperties *rigid_properties = rigid_body->GetMotionProperties();
	Vec3 next_rigid_linear_velocity;
	Vec3 next_rigid_angular_velocity;
	bool within_velocity_limits = false;
	for (uint attempt = 0; attempt < 16; ++attempt)
	{
		next_rigid_linear_velocity = rigid_body->GetLinearVelocity() - impulse * rigid_properties->GetInverseMass();
		next_rigid_angular_velocity = rigid_body->GetAngularVelocity() - rigid_body->GetInverseInertia().Multiply3x3(lever_arm.Cross(impulse));
		if (std::isfinite(next_rigid_linear_velocity.LengthSq()) && std::isfinite(next_rigid_angular_velocity.LengthSq())
			&& next_rigid_linear_velocity.LengthSq() <= Square(rigid_properties->GetMaxLinearVelocity())
			&& next_rigid_angular_velocity.LengthSq() <= Square(rigid_properties->GetMaxAngularVelocity()))
		{
			within_velocity_limits = true;
			break;
		}
		impulse *= 0.5f;
	}
	if (!within_velocity_limits)
	{
		mStatus = EStatus::InvalidState;
		return;
	}
	soft_velocity_delta = soft_body->GetRotation().Conjugated() * (vertex.mInvMass * impulse);
	vertex.mVelocity += soft_velocity_delta;
	inContext.mPhysicsSystem->GetBodyInterfaceNoLock().AddImpulse(mRigidBodyID, -impulse, rigid_point);
	inContext.mPhysicsSystem->GetBodyInterfaceNoLock().ActivateBody(mSoftBodyID);
	mLastImpulseOnSoftBody = impulse;
	++mAppliedStepCount;
}

JPH_NAMESPACE_END
