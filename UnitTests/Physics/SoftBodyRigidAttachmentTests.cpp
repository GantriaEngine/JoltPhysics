// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2026 Gantria
// SPDX-License-Identifier: MIT

#include "UnitTestFramework.h"
#include "PhysicsTestContext.h"
#include "Layers.h"
#include <Jolt/Physics/SoftBody/SoftBodyRigidAttachment.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>

TEST_SUITE("SoftBodyRigidAttachmentTests")
{
	static Body &CreateSoftTriangle(PhysicsTestContext &inContext, bool inAddEdge = false)
	{
		Ref<SoftBodySharedSettings> shared = new SoftBodySharedSettings;
		shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(0, 0, 0), Float3(0, 0, 0), 0.0f));
		shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(1, 0, 0)));
		shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(0, 1, 0), Float3(0, 0, 0), 0.0f));
		shared->AddFace(SoftBodySharedSettings::Face(0, 1, 2));
		if (inAddEdge)
		{
			shared->mEdgeConstraints.push_back(SoftBodySharedSettings::Edge(0, 1));
			shared->CalculateEdgeLengths();
		}
		shared->Optimize();
		SoftBodyCreationSettings settings(shared, RVec3::sZero(), Quat::sIdentity(), Layers::MOVING);
		settings.mVelocityReconstruction = ESoftBodyVelocityReconstruction::ConstraintDelta;
		settings.mLinearDamping = 0.0f;
		settings.mAllowSleeping = false;
		settings.mUpdatePosition = false;
		Body &body = *inContext.GetBodyInterface().CreateSoftBody(settings);
		inContext.GetBodyInterface().AddBody(body.GetID(), EActivation::Activate);
		return body;
	}

	static Body &CreateRigidBox(PhysicsTestContext &inContext, RVec3Arg inPosition)
	{
		Body &body = inContext.CreateBox(inPosition, Quat::sIdentity(), EMotionType::Dynamic,
			EMotionQuality::Discrete, Layers::MOVING, Vec3::sReplicate(0.1f));
		body.GetMotionProperties()->SetLinearDamping(0.0f);
		body.GetMotionProperties()->SetAngularDamping(0.0f);
		return body;
	}

	static SoftBodyRigidAttachment::Settings GetSettings()
	{
		SoftBodyRigidAttachment::Settings settings;
		settings.mBreakDistance = 4.0f;
		settings.mMaxImpulse = 0.2f;
		return settings;
	}

	TEST_CASE("TestPairedImpulseConservesMomentum")
	{
		PhysicsTestContext context;
		context.ZeroGravity();
		Body &soft = CreateSoftTriangle(context);
		Body &rigid = CreateRigidBox(context, RVec3(2, 0, 0));
		SoftBodyRigidAttachment attachment;
		CHECK(attachment.Connect(*context.GetSystem(), soft.GetID(), 1, rigid.GetID(), Vec3::sZero(), GetSettings()));
		context.SimulateSingleStep();
		Vec3 impulse = attachment.GetLastImpulseOnSoftBody();
		CHECK(impulse.GetX() > 0.0f);
		CHECK_APPROX_EQUAL(rigid.GetLinearVelocity(), -impulse * rigid.GetMotionProperties()->GetInverseMass(), 1.0e-5f);
		const SoftBodyVertex &vertex = static_cast<SoftBodyMotionProperties *>(soft.GetMotionProperties())->GetVertex(1);
		CHECK_APPROX_EQUAL(vertex.mVelocity, impulse * vertex.mInvMass, 1.0e-4f);
		CHECK(attachment.GetAppliedStepCount() == 1);
	}

	TEST_CASE("TestLoadedRopePullsDynamicRigidBody")
	{
		PhysicsTestContext context(1.0f / 60.0f, 2);
		context.ZeroGravity();
		Body &soft = CreateSoftTriangle(context, true);
		Body &rigid = CreateRigidBox(context, RVec3(2, 0, 0));
		SoftBodyRigidAttachment attachment;
		CHECK(attachment.Connect(*context.GetSystem(), soft.GetID(), 1, rigid.GetID(), Vec3::sZero(), GetSettings()));
		for (uint i = 0; i < 90; ++i)
			context.SimulateSingleStep();
		CHECK(attachment.GetStatus() == SoftBodyRigidAttachment::EStatus::Active);
		CHECK(attachment.GetAppliedStepCount() == 180);
		CHECK(rigid.GetCenterOfMassPosition().GetX() < 1.9);
		CHECK(rigid.GetCenterOfMassPosition().GetX() > -2.0);
	}

	TEST_CASE("TestMovingRigidTransfersVelocityToSoftVertex")
	{
		PhysicsTestContext context;
		context.ZeroGravity();
		Body &soft = CreateSoftTriangle(context);
		Body &rigid = CreateRigidBox(context, RVec3(2, 0, 0));
		rigid.SetLinearVelocity(Vec3(2, 0, 0));
		float initial_energy = 0.5f * 4.0f / rigid.GetMotionProperties()->GetInverseMass();
		SoftBodyRigidAttachment attachment;
		CHECK(attachment.Connect(*context.GetSystem(), soft.GetID(), 1, rigid.GetID(), Vec3(-1, 0, 0), GetSettings()));
		context.SimulateSingleStep();
		CHECK(attachment.GetLastImpulseOnSoftBody().GetX() > 0.0f);
		CHECK(rigid.GetLinearVelocity().GetX() < 2.0f);
		const SoftBodyVertex &vertex = static_cast<SoftBodyMotionProperties *>(soft.GetMotionProperties())->GetVertex(1);
		CHECK(vertex.mVelocity.GetX() > 0.0f);
		float final_energy = 0.5f * rigid.GetLinearVelocity().LengthSq() / rigid.GetMotionProperties()->GetInverseMass()
			+ 0.5f * vertex.mVelocity.LengthSq() / vertex.mInvMass;
		CHECK(final_energy < initial_energy);
	}

	TEST_CASE("TestOffCenterAnchorProducesOppositeTorque")
	{
		PhysicsTestContext context;
		context.ZeroGravity();
		Body &soft = CreateSoftTriangle(context);
		Body &rigid = CreateRigidBox(context, RVec3(2, -1, 0));
		SoftBodyRigidAttachment attachment;
		CHECK(attachment.Connect(*context.GetSystem(), soft.GetID(), 1, rigid.GetID(), Vec3(0, 1, 0), GetSettings()));
		context.SimulateSingleStep();
		CHECK(attachment.GetLastImpulseOnSoftBody().GetX() > 0.0f);
		CHECK(rigid.GetAngularVelocity().GetZ() > 0.0f);
		CHECK(rigid.GetLinearVelocity().GetX() < 0.0f);
	}

	TEST_CASE("TestBreakDistanceStopsWorkBeforeImpulse")
	{
		PhysicsTestContext context;
		context.ZeroGravity();
		Body &soft = CreateSoftTriangle(context);
		Body &rigid = CreateRigidBox(context, RVec3(3, 0, 0));
		SoftBodyRigidAttachment attachment;
		SoftBodyRigidAttachment::Settings settings = GetSettings();
		settings.mBreakDistance = 0.5f;
		CHECK(attachment.Connect(*context.GetSystem(), soft.GetID(), 1, rigid.GetID(), Vec3::sZero(), settings));
		context.SimulateSingleStep();
		CHECK(attachment.GetStatus() == SoftBodyRigidAttachment::EStatus::Broken);
		CHECK(attachment.GetAppliedStepCount() == 0);
		CHECK_APPROX_EQUAL(rigid.GetLinearVelocity(), Vec3::sZero(), 1.0e-6f);
	}

	TEST_CASE("TestRemovedBodyAndGenerationCannotReceiveImpulse")
	{
		PhysicsTestContext context;
		context.ZeroGravity();
		Body &soft = CreateSoftTriangle(context);
		Body &rigid = CreateRigidBox(context, RVec3(2, 0, 0));
		BodyID old_id = rigid.GetID();
		SoftBodyRigidAttachment attachment;
		CHECK(attachment.Connect(*context.GetSystem(), soft.GetID(), 1, old_id, Vec3::sZero(), GetSettings()));
		context.GetBodyInterface().RemoveBody(old_id);
		context.GetBodyInterface().DestroyBody(old_id);
		Body &replacement = CreateRigidBox(context, RVec3(2, 0, 0));
		if (replacement.GetID().GetIndex() == old_id.GetIndex())
			CHECK(replacement.GetID().GetSequenceNumber() != old_id.GetSequenceNumber());
		context.SimulateSingleStep();
		CHECK(attachment.GetStatus() == SoftBodyRigidAttachment::EStatus::BodyMissing);
		CHECK(attachment.GetAppliedStepCount() == 0);
		CHECK_APPROX_EQUAL(replacement.GetLinearVelocity(), Vec3::sZero(), 1.0e-6f);
	}

	TEST_CASE("TestStepBoundFailsClosed")
	{
		PhysicsTestContext context(1.0f / 60.0f, 4);
		context.ZeroGravity();
		Body &soft = CreateSoftTriangle(context);
		Body &rigid = CreateRigidBox(context, RVec3(2, 0, 0));
		SoftBodyRigidAttachment attachment;
		SoftBodyRigidAttachment::Settings settings = GetSettings();
		settings.mMaxStepsPerUpdate = 2;
		CHECK(attachment.Connect(*context.GetSystem(), soft.GetID(), 1, rigid.GetID(), Vec3::sZero(), settings));
		context.SimulateSingleStep();
		CHECK(attachment.GetStatus() == SoftBodyRigidAttachment::EStatus::StepLimitExceeded);
		CHECK(attachment.GetAppliedStepCount() == 2);
	}

	TEST_CASE("TestValidationRejectsPinnedVertexAndNonDynamicRigid")
	{
		PhysicsTestContext context;
		context.ZeroGravity();
		Body &soft = CreateSoftTriangle(context);
		Body &rigid = CreateRigidBox(context, RVec3(2, 0, 0));
		SoftBodyRigidAttachment attachment;
		CHECK(!attachment.Connect(*context.GetSystem(), soft.GetID(), 0, rigid.GetID(), Vec3::sZero(), GetSettings()));
		SoftBodyRigidAttachment::Settings settings = GetSettings();
		settings.mStiffness = 1.0f;
		CHECK(!attachment.Connect(*context.GetSystem(), soft.GetID(), 1, rigid.GetID(), Vec3::sZero(), settings));
		Body &static_rigid = context.CreateBox(RVec3(3, 0, 0), Quat::sIdentity(), EMotionType::Static,
			EMotionQuality::Discrete, Layers::NON_MOVING, Vec3::sReplicate(0.1f));
		CHECK(!attachment.Connect(*context.GetSystem(), soft.GetID(), 1, static_rigid.GetID(), Vec3::sZero(), GetSettings()));
		CHECK(attachment.GetStatus() == SoftBodyRigidAttachment::EStatus::Disconnected);
	}

	TEST_CASE("TestOverlappingHelpersRejectedUntilDisconnect")
	{
		PhysicsTestContext context;
		context.ZeroGravity();
		Body &soft = CreateSoftTriangle(context);
		Body &rigid = CreateRigidBox(context, RVec3(2, 0, 0));
		SoftBodyRigidAttachment first;
		SoftBodyRigidAttachment second;
		CHECK(first.Connect(*context.GetSystem(), soft.GetID(), 1, rigid.GetID(), Vec3::sZero(), GetSettings()));
		CHECK(!second.Connect(*context.GetSystem(), soft.GetID(), 1, rigid.GetID(), Vec3::sZero(), GetSettings()));
		CHECK(second.GetStatus() == SoftBodyRigidAttachment::EStatus::Disconnected);
		first.Disconnect();
		CHECK(second.Connect(*context.GetSystem(), soft.GetID(), 1, rigid.GetID(), Vec3::sZero(), GetSettings()));
	}
}
