// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2026 Gantria
// SPDX-License-Identifier: MIT

#include "UnitTestFramework.h"
#include "PhysicsTestContext.h"
#include "Layers.h"
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Core/StreamWrapper.h>
#include <cmath>
#include <cstdlib>
#include <sstream>

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

	static Ref<SoftBodySharedSettings> MakeGroupingCloth()
	{
		Ref<SoftBodySharedSettings> Shared = new SoftBodySharedSettings;
		constexpr uint Resolution = 20;
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
		Shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(0.0f, 0.0f, 1.0f)));
		Shared->mVolumeConstraints.emplace_back(0, 1, Resolution, Resolution * Resolution, 0.001f);
		Shared->CalculateEdgeLengths();
		Shared->CalculateVolumeConstraintVolumes();
		return Shared;
	}

	static std::string SaveSettings(const SoftBodySharedSettings &inSettings)
	{
		std::stringstream Stream;
		StreamOutWrapper Wrapper(Stream);
		inSettings.SaveBinaryState(Wrapper);
		return Stream.str();
	}

	struct ScopedContactOrder
	{
		explicit ScopedContactOrder(const char *inOrder)
		{
			const char *Previous = std::getenv("GANTRIA_CONTACT_ORDER");
			mHadPrevious = Previous != nullptr;
			if (mHadPrevious)
				mPrevious = Previous;
			mSetSucceeded = Set(inOrder);
		}

		~ScopedContactOrder()
		{
			Set(mHadPrevious ? mPrevious.c_str() : nullptr);
		}

		static bool Set(const char *inOrder)
		{
#ifdef _WIN32
			return _putenv_s("GANTRIA_CONTACT_ORDER", inOrder != nullptr ? inOrder : "") == 0;
#else
			return inOrder != nullptr ? setenv("GANTRIA_CONTACT_ORDER", inOrder, 1) == 0
				: unsetenv("GANTRIA_CONTACT_ORDER") == 0;
#endif
		}

		bool mSetSucceeded = false;
		bool mHadPrevious = false;
		std::string mPrevious;
	};

	struct ScopedKinematicRelativeContact
	{
		explicit ScopedKinematicRelativeContact(bool inEnabled)
		{
			const char *Previous = std::getenv("GANTRIA_CONTACT_KINEMATIC_RELATIVE");
			mHadPrevious = Previous != nullptr;
			if (mHadPrevious)
				mPrevious = Previous;
			mSetSucceeded = Set(inEnabled ? "1" : nullptr);
		}

		~ScopedKinematicRelativeContact()
		{
			Set(mHadPrevious ? mPrevious.c_str() : nullptr);
		}

		static bool Set(const char *inValue)
		{
#ifdef _WIN32
			return _putenv_s("GANTRIA_CONTACT_KINEMATIC_RELATIVE", inValue != nullptr ? inValue : "") == 0;
#else
			return inValue != nullptr ? setenv("GANTRIA_CONTACT_KINEMATIC_RELATIVE", inValue, 1) == 0
				: unsetenv("GANTRIA_CONTACT_KINEMATIC_RELATIVE") == 0;
#endif
		}

		bool mSetSucceeded = false;
		bool mHadPrevious = false;
		std::string mPrevious;
	};

	static Body &CreateSoft(PhysicsTestContext &inContext, const SoftBodySharedSettings *inShared, uint32 inSweeps, uint32 InTemporalIterations = 1)
	{
		SoftBodyCreationSettings settings(inShared, RVec3::sZero(), Quat::sIdentity(), Layers::MOVING);
		settings.mVelocityReconstruction = ESoftBodyVelocityReconstruction::ConstraintDelta;
		settings.mNumIterations = InTemporalIterations;
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
		Body &SerialBody = CreateSoft(Serial, Shared, 2, 2);
		Body &ParallelBody = CreateSoft(Parallel, Shared, 2, 2);
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
			CHECK(SerialMotion->GetVertex(Index).mPosition == ParallelMotion->GetVertex(Index).mPosition);
			CHECK(SerialMotion->GetVertex(Index).mVelocity == ParallelMotion->GetVertex(Index).mVelocity);
			CHECK(SerialMotion->GetVertex(Index).mConstraintDelta == ParallelMotion->GetVertex(Index).mConstraintDelta);
		}
	}

	TEST_CASE("TestSkippedEdgeFirstSweepClearsPreviousMultiplier")
	{
		PhysicsTestContext History;
		PhysicsTestContext Fresh;
		History.ZeroGravity();
		Fresh.ZeroGravity();
		Ref<SoftBodySharedSettings> Shared = new SoftBodySharedSettings;
		Shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(0, 0, 0), Float3(0, 0, 0), 0.0f));
		Shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(1, 0, 0)));
		Shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(2, 0, 0), Float3(0, 0, 0), 0.0f));
		Shared->mEdgeConstraints.emplace_back(0, 1, 0.001f);
		Shared->mEdgeConstraints.emplace_back(1, 2, 0.0f);
		Shared->CalculateEdgeLengths();
		Shared->Optimize();
		Body &HistoryBody = CreateSoft(History, Shared, 2);
		Body &FreshBody = CreateSoft(Fresh, Shared, 2);
		auto *HistoryMotion = static_cast<SoftBodyMotionProperties *>(HistoryBody.GetMotionProperties());
		auto *FreshMotion = static_cast<SoftBodyMotionProperties *>(FreshBody.GetMotionProperties());
		HistoryMotion->GetVertex(1).mPosition = Vec3(1.4f, 0, 0);
		History.SimulateSingleStep(); // Leaves a nonzero retained multiplier for the first edge.
		CHECK(HistoryMotion->GetVertex(1).mConstraintDelta.LengthSq() > 0.0f);
		HistoryMotion->GetVertex(1).mPosition = Vec3::sZero();
		HistoryMotion->GetVertex(1).mVelocity = Vec3::sZero();
		FreshMotion->GetVertex(1).mPosition = Vec3::sZero();
		FreshMotion->GetVertex(1).mVelocity = Vec3::sZero();
		History.SimulateSingleStep(); // First edge starts at zero length, then the second edge separates it.
		Fresh.SimulateSingleStep();
		CHECK(FreshMotion->GetVertex(1).mPosition.LengthSq() > 0.0f);
		for (uint Index = 0; Index < Shared->mVertices.size(); ++Index)
		{
			CHECK(HistoryMotion->GetVertex(Index).mPosition == FreshMotion->GetVertex(Index).mPosition);
			CHECK(HistoryMotion->GetVertex(Index).mVelocity == FreshMotion->GetVertex(Index).mVelocity);
			CHECK(HistoryMotion->GetVertex(Index).mConstraintDelta == FreshMotion->GetVertex(Index).mConstraintDelta);
		}
	}

	TEST_CASE("TestSkippedVolumeFirstSweepClearsPreviousMultiplier")
	{
		PhysicsTestContext History;
		PhysicsTestContext Fresh;
		History.ZeroGravity();
		Fresh.ZeroGravity();
		Ref<SoftBodySharedSettings> Shared = new SoftBodySharedSettings;
		Shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(0, 0, 0), Float3(0, 0, 0), 0.0f));
		Shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(0, 0.001f, 0)));
		Shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(0, 0, 0.001f)));
		Shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(1, 0, 0), Float3(0, 0, 0), 0.0f));
		Shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(0, 2, 0), Float3(0, 0, 0), 0.0f));
		Shared->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(0, 0, 2), Float3(0, 0, 0), 0.0f));
		Shared->mVolumeConstraints.emplace_back(0, 1, 2, 3, 1.0e-16f);
		Shared->mVolumeConstraints.back().mSixRestVolume = 1.0f;
		Shared->mEdgeConstraints.emplace_back(1, 4, 0.0001f);
		Shared->mEdgeConstraints.emplace_back(2, 5, 0.0001f);
		Shared->mEdgeConstraints[0].mRestLength = 1.0f;
		Shared->mEdgeConstraints[1].mRestLength = 1.0f;
		Shared->Optimize();
		Body &HistoryBody = CreateSoft(History, Shared, 2);
		Body &FreshBody = CreateSoft(Fresh, Shared, 2);
		auto *HistoryMotion = static_cast<SoftBodyMotionProperties *>(HistoryBody.GetMotionProperties());
		auto *FreshMotion = static_cast<SoftBodyMotionProperties *>(FreshBody.GetMotionProperties());
		History.SimulateSingleStep(); // The nearly collinear tetrahedron creates a retained multiplier.
		CHECK(HistoryMotion->GetVertex(1).mConstraintDelta.LengthSq() > 0.0f);
		for (uint Index : { 1u, 2u })
		{
			HistoryMotion->GetVertex(Index).mPosition = Vec3::sZero();
			HistoryMotion->GetVertex(Index).mVelocity = Vec3::sZero();
			FreshMotion->GetVertex(Index).mPosition = Vec3::sZero();
			FreshMotion->GetVertex(Index).mVelocity = Vec3::sZero();
		}
		History.SimulateSingleStep(); // The volume is degenerate before the edges separate its vertices.
		Fresh.SimulateSingleStep();
		CHECK(FreshMotion->GetVertex(1).mPosition.LengthSq() > 0.0f);
		CHECK(FreshMotion->GetVertex(2).mPosition.LengthSq() > 0.0f);
		for (uint Index = 0; Index < Shared->mVertices.size(); ++Index)
		{
			CHECK(HistoryMotion->GetVertex(Index).mPosition == FreshMotion->GetVertex(Index).mPosition);
			CHECK(HistoryMotion->GetVertex(Index).mVelocity == FreshMotion->GetVertex(Index).mVelocity);
			CHECK(HistoryMotion->GetVertex(Index).mConstraintDelta == FreshMotion->GetVertex(Index).mConstraintDelta);
		}
	}

	TEST_CASE("TestDefaultGroupSizeMatchesExplicit256")
	{
		Ref<SoftBodySharedSettings> Source = MakeGroupingCloth();
		Ref<SoftBodySharedSettings> Default = Source->Clone();
		Ref<SoftBodySharedSettings> Explicit = Source->Clone();
		SoftBodySharedSettings::OptimizationResults DefaultResults;
		SoftBodySharedSettings::OptimizationResults ExplicitResults;
		Default->Optimize(DefaultResults);
		CHECK(Explicit->Optimize(ExplicitResults, 256));
		CHECK(SaveSettings(*Default) == SaveSettings(*Explicit));
		CHECK(DefaultResults.mEdgeRemap.size() == ExplicitResults.mEdgeRemap.size());
		for (uint Index = 0; Index < DefaultResults.mEdgeRemap.size(); ++Index)
			CHECK(DefaultResults.mEdgeRemap[Index] == ExplicitResults.mEdgeRemap[Index]);
	}

	TEST_CASE("TestUnsupportedGroupSizeDoesNotChangeSettingsOrResults")
	{
		Ref<SoftBodySharedSettings> Shared = MakeGroupingCloth();
		SoftBodySharedSettings::OptimizationResults Results;
		Results.mEdgeRemap.push_back(77);
		Results.mVolumeRemap.push_back(9);
		const std::string Before = SaveSettings(*Shared);
		for (uint Batch : { 0u, 127u, 257u, 2048u })
		{
			CHECK_FALSE(Shared->Optimize(Results, Batch));
			CHECK(SaveSettings(*Shared) == Before);
			CHECK(Results.mEdgeRemap.size() == 1);
			CHECK(Results.mEdgeRemap[0] == 77);
			CHECK(Results.mVolumeRemap.size() == 1);
			CHECK(Results.mVolumeRemap[0] == 9);
		}
	}

	TEST_CASE("TestSupportedGroupSizesPreserveConstraintPermutation")
	{
		Ref<SoftBodySharedSettings> Source = MakeGroupingCloth();
		for (uint Batch : { 128u, 256u, 512u, 1024u })
		{
			Ref<SoftBodySharedSettings> Shared = Source->Clone();
			SoftBodySharedSettings::OptimizationResults Results;
			CHECK(Shared->Optimize(Results, Batch));
			CHECK(Shared->GetNumUpdateGroups() > 0);
			CHECK(Shared->GetNumNonParallelEdgeConstraints() <= Shared->mEdgeConstraints.size());
			CHECK(Shared->GetNumNonParallelVolumeConstraints() <= Shared->mVolumeConstraints.size());
			CHECK(Results.mEdgeRemap.size() == Source->mEdgeConstraints.size());
			CHECK(Results.mVolumeRemap.size() == Source->mVolumeConstraints.size());
			CHECK(Results.mVolumeRemap[0] == 0);
			Array<uint8> Seen(Results.mEdgeRemap.size(), 0);
			for (uint OldIndex = 0; OldIndex < Results.mEdgeRemap.size(); ++OldIndex)
			{
				const uint NewIndex = Results.mEdgeRemap[OldIndex];
				CHECK(NewIndex < Seen.size());
				if (NewIndex >= Seen.size())
					continue;
				CHECK(Seen[NewIndex] == 0);
				Seen[NewIndex] = 1;
				const auto &OldEdge = Source->mEdgeConstraints[OldIndex];
				const auto &NewEdge = Shared->mEdgeConstraints[NewIndex];
				CHECK(OldEdge.mVertex[0] == NewEdge.mVertex[0]);
				CHECK(OldEdge.mVertex[1] == NewEdge.mVertex[1]);
				CHECK(OldEdge.mRestLength == NewEdge.mRestLength);
				CHECK(OldEdge.mCompliance == NewEdge.mCompliance);
			}
		}
	}

	TEST_CASE("TestContactOrderModesLeaveNoContactGraphExact")
	{
		Ref<SoftBodySharedSettings> Shared = MakeChain();
		Vec3 BaselinePosition[3], BaselineVelocity[3];
		for (const char *Order : { "", "B", "C", "D", "E", "F" })
		{
			ScopedContactOrder Environment(*Order != '\0' ? Order : nullptr);
			CHECK(Environment.mSetSucceeded);
			PhysicsTestContext Context;
			Context.ZeroGravity();
			Body &BodyState = CreateSoft(Context, Shared, 2);
			auto *Motion = static_cast<SoftBodyMotionProperties *>(BodyState.GetMotionProperties());
			Motion->GetVertex(2).mPosition = Vec3(2.3f, 0, 0);
			Context.SimulateSingleStep();
			for (uint Index = 0; Index < 3; ++Index)
			{
				const auto &VertexState = Motion->GetVertex(Index);
				CHECK(!VertexState.mPosition.IsNaN());
				CHECK(!VertexState.mVelocity.IsNaN());
				if (*Order == '\0')
				{
					BaselinePosition[Index] = VertexState.mPosition;
					BaselineVelocity[Index] = VertexState.mVelocity;
				}
				else
				{
					CHECK(VertexState.mPosition == BaselinePosition[Index]);
					CHECK(VertexState.mVelocity == BaselineVelocity[Index]);
				}
			}
		}
	}

	TEST_CASE("TestContactOrderModesStayFiniteOnStaticSphere")
	{
		Ref<SoftBodySharedSettings> Shared = new SoftBodySharedSettings;
		for (uint Y = 0; Y < 3; ++Y)
			for (uint X = 0; X < 3; ++X)
				Shared->mVertices.emplace_back(Float3(float(X), float(Y), 0.0f), Float3(0, 0, 0), Y == 2 ? 0.0f : 1.0f);
		for (uint Y = 0; Y < 3; ++Y)
			for (uint X = 0; X < 3; ++X)
			{
				const uint Index = 3 * Y + X;
				if (X + 1 < 3)
					Shared->mEdgeConstraints.emplace_back(Index, Index + 1, 1.0e-6f);
				if (Y + 1 < 3)
					Shared->mEdgeConstraints.emplace_back(Index, Index + 3, 1.0e-6f);
			}
		Shared->CalculateEdgeLengths();
		Shared->Optimize();
		for (const char *Order : { "", "B", "C", "D", "E", "F" })
		{
			ScopedContactOrder Environment(*Order != '\0' ? Order : nullptr);
			CHECK(Environment.mSetSucceeded);
			PhysicsTestContext Context;
			Context.ZeroGravity();
			BodyCreationSettings SphereSettings(new SphereShape(0.75f), RVec3(2.5, 1, 0), Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING);
			Context.CreateBody(SphereSettings, EActivation::DontActivate);
			SoftBodyCreationSettings SoftSettings(Shared, RVec3::sZero(), Quat::sIdentity(), Layers::MOVING);
			SoftSettings.mVelocityReconstruction = ESoftBodyVelocityReconstruction::ConstraintDelta;
			SoftSettings.mNumIterations = 2;
			SoftSettings.mNumConstraintIterations = 2;
			SoftSettings.mLinearDamping = 0.0f;
			SoftSettings.mAllowSleeping = false;
			SoftSettings.mUpdatePosition = false;
			SoftSettings.mVertexRadius = 0.05f;
			Body &BodyState = *Context.GetBodyInterface().CreateSoftBody(SoftSettings);
			Context.GetBodyInterface().AddBody(BodyState.GetID(), EActivation::Activate);
			auto *Motion = static_cast<SoftBodyMotionProperties *>(BodyState.GetMotionProperties());
			Context.SimulateSingleStep();
			for (uint Index = 0; Index < 9; ++Index)
			{
				const auto &VertexState = Motion->GetVertex(Index);
				CHECK(std::isfinite(VertexState.mPosition.GetX()));
				CHECK(std::isfinite(VertexState.mPosition.GetY()));
				CHECK(std::isfinite(VertexState.mPosition.GetZ()));
				CHECK(std::isfinite(VertexState.mVelocity.GetX()));
				CHECK(std::isfinite(VertexState.mVelocity.GetY()));
				CHECK(std::isfinite(VertexState.mVelocity.GetZ()));
				if (*Order != 'B' && VertexState.mInvMass > 0.0f)
					CHECK((VertexState.mPosition - Vec3(2.5f, 1.0f, 0.0f)).Length() >= 0.8f - 1.0e-4f);
				if (Index / 3 == 2)
					CHECK(VertexState.mPosition.GetY() == 2.0f);
			}
			CHECK(Motion->GetVertex(5).mPosition.GetX() < 2.0f);
		}
	}

	TEST_CASE("TestKinematicContactUsesPrescribedSurfaceVelocityOnlyWhenEnabled")
	{
		Ref<SoftBodySharedSettings> Shared = new SoftBodySharedSettings;
		Shared->mVertices.emplace_back(Float3(0.0f, 0.0f, 0.0f));
		Shared->Optimize();
		for (const char *Order : { "", "F" })
		{
			ScopedContactOrder OrderEnvironment(*Order != '\0' ? Order : nullptr);
			CHECK(OrderEnvironment.mSetSucceeded);
			for (float SurfaceSpeed : { -1.0f, 1.0f })
			{
					Vec3 StaticOff, StaticOn, KinematicOff, KinematicOn;
					for (EMotionType MotionType : { EMotionType::Static, EMotionType::Kinematic })
						for (bool Relative : { false, true })
						{
							ScopedKinematicRelativeContact Environment(Relative);
							CHECK(Environment.mSetSucceeded);
							PhysicsTestContext Context;
							Context.ZeroGravity();
							BodyCreationSettings SphereSettings(new SphereShape(0.75f), RVec3(0.6, 0, 0), Quat::sIdentity(), MotionType,
								MotionType == EMotionType::Static ? Layers::NON_MOVING : Layers::MOVING);
							Body &Sphere = Context.CreateBody(SphereSettings, MotionType == EMotionType::Static ? EActivation::DontActivate : EActivation::Activate);
							if (MotionType == EMotionType::Kinematic)
								Sphere.SetLinearVelocity(Vec3(SurfaceSpeed, 0.0f, 0.0f));
							SoftBodyCreationSettings SoftSettings(Shared, RVec3::sZero(), Quat::sIdentity(), Layers::MOVING);
							SoftSettings.mVelocityReconstruction = ESoftBodyVelocityReconstruction::ConstraintDelta;
							SoftSettings.mNumIterations = 1;
							SoftSettings.mNumConstraintIterations = 2;
							SoftSettings.mLinearDamping = 0.0f;
							SoftSettings.mAllowSleeping = false;
							SoftSettings.mUpdatePosition = false;
							SoftSettings.mVertexRadius = 0.05f;
							Body &Soft = *Context.GetBodyInterface().CreateSoftBody(SoftSettings);
							Context.GetBodyInterface().AddBody(Soft.GetID(), EActivation::Activate);
							Context.SimulateSingleStep();
							const auto &VertexState = static_cast<SoftBodyMotionProperties *>(Soft.GetMotionProperties())->GetVertex(0);
							CHECK(!VertexState.mVelocity.IsNaN());
							// UpdateSoftBodyState resets mHasContact before publishing the
							// frame. Verify the actual contact projection geometrically.
							const RVec3 CenterOfMass = Sphere.GetCenterOfMassPosition();
							const Vec3 Center(float(CenterOfMass.GetX()), float(CenterOfMass.GetY()), float(CenterOfMass.GetZ()));
							CHECK((VertexState.mPosition - Center).Length() >= 0.8f - 1.0e-4f);
							CHECK(VertexState.mPosition.GetX() < -0.1f);
							if (MotionType == EMotionType::Static)
								(Relative ? StaticOn : StaticOff) = VertexState.mVelocity;
							else
								(Relative ? KinematicOn : KinematicOff) = VertexState.mVelocity;
						}
					CHECK(StaticOn == StaticOff);
					if (SurfaceSpeed < 0.0f)
						CHECK(KinematicOn.GetX() < KinematicOff.GetX() - 0.1f);
					else
						CHECK_APPROX_EQUAL(KinematicOn, KinematicOff, 1.0e-5f);
			}
		}
	}
}
