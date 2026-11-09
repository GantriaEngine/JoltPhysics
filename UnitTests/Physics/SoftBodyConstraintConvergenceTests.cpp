// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2026 Gantria
// SPDX-License-Identifier: MIT

#include "UnitTestFramework.h"
#include "PhysicsTestContext.h"
#include "Layers.h"
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>

TEST_SUITE("SoftBodyConstraintConvergenceTests")
{
	static Ref<SoftBodySharedSettings> MakeChain()
	{
		Ref<SoftBodySharedSettings> shared = new SoftBodySharedSettings;
		shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(0, 0, 0), Float3(0, 0, 0), 0.0f));
		shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(1, 0, 0)));
		shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(2, 0, 0)));
		shared->mEdgeConstraints.push_back(SoftBodySharedSettings::Edge(0, 1, 0.001f));
		shared->mEdgeConstraints.push_back(SoftBodySharedSettings::Edge(1, 2, 0.001f));
		shared->CalculateEdgeLengths();
		shared->Optimize();
		return shared;
	}

	static Body &CreateSoft(PhysicsTestContext &inContext, const SoftBodySharedSettings *inShared, uint32 inSweeps)
	{
		SoftBodyCreationSettings settings(inShared, RVec3::sZero(), Quat::sIdentity(), Layers::MOVING);
		settings.mVelocityReconstruction = ESoftBodyVelocityReconstruction::ConstraintDelta;
		settings.mNumIterations = 1;
		settings.mNumConstraintIterations = inSweeps;
		settings.mLinearDamping = 0.0f;
		settings.mAllowSleeping = false;
		settings.mUpdatePosition = false;
		Body &body = *inContext.GetBodyInterface().CreateSoftBody(settings);
		inContext.GetBodyInterface().AddBody(body.GetID(), EActivation::Activate);
		CHECK(body.GetSoftBodyCreationSettings().mNumConstraintIterations == inSweeps);
		return body;
	}

	TEST_CASE("TestSingleSweepMatchesOriginalEdgeCorrection")
	{
		PhysicsTestContext context;
		context.ZeroGravity();
		Ref<SoftBodySharedSettings> shared = MakeChain();
		Body &body = CreateSoft(context, shared, 1);
		auto *motion = static_cast<SoftBodyMotionProperties *>(body.GetMotionProperties());
		CHECK(motion->GetNumConstraintIterations() == 1);
		motion->GetVertex(2).mPosition = Vec3(2.3f, 0, 0);
		float expected[3] = {0.0f, 1.0f, 2.3f};
		const float inverse_mass[3] = {0.0f, 1.0f, 1.0f};
		for (const auto &edge : shared->mEdgeConstraints)
		{
			const uint a = edge.mVertex[0], b = edge.mVertex[1];
			float delta = expected[b] - expected[a];
			float length = abs(delta);
			float denom = length * (inverse_mass[a] + inverse_mass[b]
				+ edge.mCompliance / Square(context.GetStepDeltaTime()));
			if (denom < 1.0e-12f) continue;
			float correction = delta * (length - edge.mRestLength) / denom;
			expected[a] += inverse_mass[a] * correction;
			expected[b] -= inverse_mass[b] * correction;
		}
		context.SimulateSingleStep();
		for (uint i = 0; i < 3; ++i)
			CHECK_APPROX_EQUAL(motion->GetVertex(i).mPosition.GetX(), expected[i], 1.0e-6f);
	}

	TEST_CASE("TestRetainedEdgeMultiplierMatchesFourSweepScalarReference")
	{
		PhysicsTestContext context;
		context.ZeroGravity();
		Ref<SoftBodySharedSettings> shared = MakeChain();
		Body &body = CreateSoft(context, shared, 4);
		auto *motion = static_cast<SoftBodyMotionProperties *>(body.GetMotionProperties());
		motion->GetVertex(2).mPosition = Vec3(2.3f, 0, 0);
		float expected[3] = {0.0f, 1.0f, 2.3f};
		const float inverse_mass[3] = {0.0f, 1.0f, 1.0f};
		float lambda[2] = {0.0f, 0.0f};
		for (uint sweep = 0; sweep < 4; ++sweep)
			for (uint i = 0; i < shared->mEdgeConstraints.size(); ++i)
			{
				const auto &edge = shared->mEdgeConstraints[i];
				const uint a = edge.mVertex[0], b = edge.mVertex[1];
				float delta = expected[b] - expected[a];
				float length = abs(delta);
				float alpha = edge.mCompliance / Square(context.GetStepDeltaTime());
				float dl = -(length - edge.mRestLength + alpha * lambda[i]) /
					(inverse_mass[a] + inverse_mass[b] + alpha);
				lambda[i] += dl;
				float correction = -delta * dl / length;
				expected[a] += inverse_mass[a] * correction;
				expected[b] -= inverse_mass[b] * correction;
			}
		context.SimulateSingleStep();
		for (uint i = 0; i < 3; ++i)
		{
			const auto &vertex = motion->GetVertex(i);
			CHECK_APPROX_EQUAL(vertex.mPosition.GetX(), expected[i], 1.0e-5f);
			CHECK_APPROX_EQUAL(vertex.mVelocity, vertex.mIntegratedVelocity + vertex.mConstraintDelta / context.GetStepDeltaTime(), 1.0e-5f);
		}
	}

	TEST_CASE("TestExtraSweepsDoNotMultiplyFreeIntegrationOrEnergy")
	{
		PhysicsTestContext context;
		context.ZeroGravity();
		Ref<SoftBodySharedSettings> shared = new SoftBodySharedSettings;
		shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(0, 0, 0), Float3(2, 0, 0)));
		shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(0, 1, 0), Float3(2, 0, 0)));
		shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(0, 0, 1), Float3(2, 0, 0)));
		shared->Optimize();
		Body &body = CreateSoft(context, shared, 8);
		auto *motion = static_cast<SoftBodyMotionProperties *>(body.GetMotionProperties());
		for (uint frame = 0; frame < 60; ++frame)
			context.SimulateSingleStep();
		CHECK_APPROX_EQUAL(motion->GetVertex(0).mPosition.GetX(), 2.0f, 2.0e-5f);
		CHECK_APPROX_EQUAL(motion->GetVertex(0).mVelocity.GetX(), 2.0f, 1.0e-6f);
	}

	TEST_CASE("TestVolumeAndEdgeGraphUsesAdditionalSweeps")
	{
		PhysicsTestContext one;
		PhysicsTestContext two;
		one.ZeroGravity();
		two.ZeroGravity();
		Ref<SoftBodySharedSettings> shared = SoftBodySharedSettings::sCreateCube(2, 1.0f);
		Body &one_body = CreateSoft(one, shared, 1);
		Body &two_body = CreateSoft(two, shared, 2);
		auto *one_motion = static_cast<SoftBodyMotionProperties *>(one_body.GetMotionProperties());
		auto *two_motion = static_cast<SoftBodyMotionProperties *>(two_body.GetMotionProperties());
		one_motion->GetVertex(0).mPosition += Vec3(0.25f, 0.1f, 0);
		two_motion->GetVertex(0).mPosition += Vec3(0.25f, 0.1f, 0);
		one.SimulateSingleStep();
		two.SimulateSingleStep();
		bool different = false;
		for (uint i = 0; i < shared->mVertices.size(); ++i)
		{
			const auto &a = one_motion->GetVertex(i);
			const auto &b = two_motion->GetVertex(i);
			different |= (a.mPosition - b.mPosition).LengthSq() > 1.0e-10f;
			CHECK(!b.mVelocity.IsNaN());
			CHECK_APPROX_EQUAL(b.mVelocity, b.mIntegratedVelocity + b.mConstraintDelta / two.GetStepDeltaTime(), 1.0e-4f);
		}
		CHECK(different);
	}

	TEST_CASE("TestUnsupportedFamiliesRejectBeforeBodyAllocation")
	{
		PhysicsTestContext context;
		Ref<SoftBodySharedSettings> shared = new SoftBodySharedSettings;
		shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(0, 0, 0)));
		SoftBodyCreationSettings settings(shared, RVec3::sZero(), Quat::sIdentity(), Layers::MOVING);
		settings.mVelocityReconstruction = ESoftBodyVelocityReconstruction::ConstraintDelta;
		settings.mNumConstraintIterations = 2;
		const uint before = context.GetSystem()->GetNumBodies();
		shared->mDihedralBendConstraints.emplace_back(0, 0, 0, 0);
		CHECK(context.GetBodyInterface().CreateSoftBody(settings) == nullptr);
		shared->mDihedralBendConstraints.clear();
		shared->mSkinnedConstraints.emplace_back(0, 0.0f, 0.0f, 0.0f);
		CHECK(context.GetBodyInterface().CreateSoftBody(settings) == nullptr);
		shared->mSkinnedConstraints.clear();
		shared->mLRAConstraints.emplace_back(0, 0, 1.0f);
		CHECK(context.GetBodyInterface().CreateSoftBody(settings) == nullptr);
		shared->mLRAConstraints.clear();
		shared->mRodStretchShearConstraints.emplace_back(0, 0);
		CHECK(context.GetBodyInterface().CreateSoftBody(settings) == nullptr);
		shared->mRodStretchShearConstraints.clear();
		shared->mRodBendTwistConstraints.emplace_back(0, 0);
		CHECK(context.GetBodyInterface().CreateSoftBody(settings) == nullptr);
		shared->mRodBendTwistConstraints.clear();
		settings.mVelocityReconstruction = ESoftBodyVelocityReconstruction::PositionDifference;
		CHECK(context.GetBodyInterface().CreateSoftBody(settings) == nullptr);
		settings.mVelocityReconstruction = ESoftBodyVelocityReconstruction::ConstraintDelta;
		settings.mNumConstraintIterations = 3;
		CHECK(context.GetBodyInterface().CreateSoftBody(settings) == nullptr);
		CHECK(context.GetSystem()->GetNumBodies() == before);
	}

	TEST_CASE("TestParallelGroupSweepCounterMatchesSingleWorker")
	{
		PhysicsTestContext serial(1.0f / 60.0f, 1, 1);
		PhysicsTestContext parallel(1.0f / 60.0f, 1, 4);
		serial.ZeroGravity();
		parallel.ZeroGravity();
		Ref<SoftBodySharedSettings> shared = new SoftBodySharedSettings;
		for (uint i = 0; i < 512; ++i)
		{
			float x = float(i % 32) * 3.0f;
			float y = float(i / 32) * 3.0f;
			shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(x, y, 0)));
			shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(x + 1.0f, y, 0)));
			shared->mEdgeConstraints.emplace_back(2 * i, 2 * i + 1, 0.001f);
		}
		shared->CalculateEdgeLengths();
		shared->Optimize();
		Body &serial_body = CreateSoft(serial, shared, 8);
		Body &parallel_body = CreateSoft(parallel, shared, 8);
		auto *a = static_cast<SoftBodyMotionProperties *>(serial_body.GetMotionProperties());
		auto *b = static_cast<SoftBodyMotionProperties *>(parallel_body.GetMotionProperties());
		a->GetVertex(1).mPosition += Vec3(0.2f, 0, 0);
		b->GetVertex(1).mPosition += Vec3(0.2f, 0, 0);
		for (uint frame = 0; frame < 3; ++frame)
		{
			serial.SimulateSingleStep();
			parallel.SimulateSingleStep();
		}
		for (uint i = 0; i < shared->mVertices.size(); ++i)
		{
			CHECK_APPROX_EQUAL(a->GetVertex(i).mPosition, b->GetVertex(i).mPosition, 1.0e-5f);
			CHECK_APPROX_EQUAL(a->GetVertex(i).mVelocity, b->GetVertex(i).mVelocity, 1.0e-5f);
		}
	}

	TEST_CASE("TestConnectedClothSweepsMatchAcrossWorkers")
	{
		PhysicsTestContext Serial(1.0f / 60.0f, 1, 1);
		PhysicsTestContext Parallel(1.0f / 60.0f, 1, 4);
		Serial.ZeroGravity();
		Parallel.ZeroGravity();
		Ref<SoftBodySharedSettings> Shared = new SoftBodySharedSettings;
		constexpr uint Resolution = 32;
		for (uint Y = 0; Y < Resolution; ++Y)
			for (uint X = 0; X < Resolution; ++X)
				Shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(float(X), float(Y), 0.0f), Float3(0, 0, 0), Y == 0 ? 0.0f : 1.0f));
		for (uint Y = 0; Y < Resolution; ++Y)
			for (uint X = 0; X < Resolution; ++X)
			{
				const uint Index = Y * Resolution + X;
				if (X + 1 < Resolution)
					Shared->mEdgeConstraints.emplace_back(Index, Index + 1, 0.001f);
				if (Y + 1 < Resolution)
					Shared->mEdgeConstraints.emplace_back(Index, Index + Resolution, 0.001f);
				if (X + 1 < Resolution && Y + 1 < Resolution)
					Shared->mEdgeConstraints.emplace_back(Index, Index + Resolution + 1, 0.001f);
			}
		Shared->CalculateEdgeLengths();
		Shared->Optimize();
		CHECK(Shared->mUpdateGroups.size() > 2);
		Body &SerialBody = CreateSoft(Serial, Shared, 2);
		Body &ParallelBody = CreateSoft(Parallel, Shared, 2);
		auto *SerialMotion = static_cast<SoftBodyMotionProperties *>(SerialBody.GetMotionProperties());
		auto *ParallelMotion = static_cast<SoftBodyMotionProperties *>(ParallelBody.GetMotionProperties());
		const uint Disturbed = 12 * Resolution + 12;
		SerialMotion->GetVertex(Disturbed).mPosition += Vec3(0.25f, 0.1f, 0.0f);
		ParallelMotion->GetVertex(Disturbed).mPosition += Vec3(0.25f, 0.1f, 0.0f);
		for (uint Frame = 0; Frame < 3; ++Frame)
		{
			Serial.SimulateSingleStep();
			Parallel.SimulateSingleStep();
		}
		for (uint Index = 0; Index < Shared->mVertices.size(); ++Index)
		{
			CHECK_APPROX_EQUAL(SerialMotion->GetVertex(Index).mPosition, ParallelMotion->GetVertex(Index).mPosition, 1.0e-5f);
			CHECK_APPROX_EQUAL(SerialMotion->GetVertex(Index).mVelocity, ParallelMotion->GetVertex(Index).mVelocity, 1.0e-5f);
		}
	}
}
