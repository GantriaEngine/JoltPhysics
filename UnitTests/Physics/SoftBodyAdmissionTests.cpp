// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2026 GantriaEngine contributors
// SPDX-License-Identifier: MIT

#include "UnitTestFramework.h"
#include "PhysicsTestContext.h"
#include "Layers.h"
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <cmath>

namespace {
    template <class T>
    bool IsPermutation(const T &inRemap)
    {
        Array<bool> seen(inRemap.size(), false);
        for (uint index : inRemap)
        {
            if (index >= inRemap.size() || seen[index])
                return false;
            seen[index] = true;
        }
        return true;
    }

    bool IsFinite(Vec3Arg inValue)
    {
        return std::isfinite(inValue.GetX()) && std::isfinite(inValue.GetY()) &&
            std::isfinite(inValue.GetZ());
    }
}

TEST_SUITE("SoftBodyAdmissionTests")
{
    TEST_CASE("TestOptimizeMixedGraphPreservesAllConstraintFamilies")
    {
        Ref<SoftBodySharedSettings> shared_settings = new SoftBodySharedSettings;
        shared_settings->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(0, 0, 0), Float3(0, 0, 0), 0.0f));
        shared_settings->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(1, 0, 0)));
        shared_settings->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(0, 1, 0)));
        shared_settings->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(0, 0, 1)));
        shared_settings->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(1, 1, 0)));
        shared_settings->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(1, 0, 1)));
        shared_settings->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(4, 0, 0)));
        shared_settings->mEdgeConstraints.push_back(SoftBodySharedSettings::Edge(0, 1));
        shared_settings->mEdgeConstraints.push_back(SoftBodySharedSettings::Edge(0, 1)); // Duplicate neighbor, distinct constraint.
        shared_settings->mEdgeConstraints.push_back(SoftBodySharedSettings::Edge(1, 2));
        shared_settings->mEdgeConstraints.push_back(SoftBodySharedSettings::Edge(2, 3));
        shared_settings->mEdgeConstraints.push_back(SoftBodySharedSettings::Edge(4, 5));
        shared_settings->mLRAConstraints.push_back(SoftBodySharedSettings::LRA(0, 5, 2.0f));
        shared_settings->mDihedralBendConstraints.push_back(SoftBodySharedSettings::DihedralBend(0, 1, 2, 3));
        shared_settings->mVolumeConstraints.push_back(SoftBodySharedSettings::Volume(0, 1, 2, 3));
        shared_settings->CalculateEdgeLengths();
        shared_settings->CalculateBendConstraintConstants();
        shared_settings->CalculateVolumeConstraintVolumes();

        SoftBodySharedSettings::OptimizationResults results;
        shared_settings->Optimize(results);
        CHECK(shared_settings->mEdgeConstraints.size() == 5);
        CHECK(shared_settings->mLRAConstraints.size() == 1);
        CHECK(shared_settings->mDihedralBendConstraints.size() == 1);
        CHECK(shared_settings->mVolumeConstraints.size() == 1);
        CHECK(IsPermutation(results.mEdgeRemap));
        CHECK(IsPermutation(results.mLRARemap));
        CHECK(IsPermutation(results.mDihedralBendRemap));
        CHECK(IsPermutation(results.mVolumeRemap));
        CHECK(results.mEdgeRemap.size() == 5);

        PhysicsTestContext context;
        context.ZeroGravity();
        SoftBodyCreationSettings settings(shared_settings, RVec3::sZero(), Quat::sIdentity(), Layers::MOVING);
        settings.mAllowSleeping = false;
        settings.mUpdatePosition = false;
        Body &first = *context.GetBodyInterface().CreateSoftBody(settings);
        Body &second = *context.GetBodyInterface().CreateSoftBody(settings);
        context.GetBodyInterface().AddBody(first.GetID(), EActivation::Activate);
        context.GetBodyInterface().AddBody(second.GetID(), EActivation::Activate);
        auto *first_motion = static_cast<SoftBodyMotionProperties *>(first.GetMotionProperties());
        auto *second_motion = static_cast<SoftBodyMotionProperties *>(second.GetMotionProperties());
        first_motion->GetVertex(1).mPosition += Vec3(0.1f, 0, 0);
        CHECK(second_motion->GetVertex(1).mPosition == Vec3(1, 0, 0));
        context.SimulateSingleStep();
        CHECK(IsFinite(first_motion->GetVertex(1).mPosition));
        CHECK(IsFinite(second_motion->GetVertex(1).mPosition));
        context.GetBodyInterface().RemoveBody(first.GetID());
        context.GetBodyInterface().DestroyBody(first.GetID());
        context.SimulateSingleStep();
        CHECK(IsFinite(second_motion->GetVertex(1).mPosition));
    }

    TEST_CASE("TestOptimizeRodAndUnpinnedIsolatedGraphs")
    {
        Ref<SoftBodySharedSettings> rods = new SoftBodySharedSettings;
        rods->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(0, 0, 0), Float3(0, 0, 0), 0.0f));
        rods->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(0, 0, 1)));
        rods->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(0, 0, 2)));
        rods->mRodStretchShearConstraints.push_back(SoftBodySharedSettings::RodStretchShear(0, 1));
        rods->mRodStretchShearConstraints.push_back(SoftBodySharedSettings::RodStretchShear(1, 2));
        rods->mRodBendTwistConstraints.push_back(SoftBodySharedSettings::RodBendTwist(0, 1));
        rods->CalculateRodProperties();
        SoftBodySharedSettings::OptimizationResults rod_results;
        rods->Optimize(rod_results);
        CHECK(IsPermutation(rod_results.mRodStretchShearConstraintRemap));
        CHECK(IsPermutation(rod_results.mRodBendTwistConstraintRemap));
        CHECK(rods->mRodStretchShearConstraints.size() == 2);
        CHECK(rods->mRodBendTwistConstraints.size() == 1);
        CHECK(rods->mRodBendTwistConstraints[0].mRod[0] < 2);
        CHECK(rods->mRodBendTwistConstraints[0].mRod[1] < 2);

        Ref<SoftBodySharedSettings> unpinned = new SoftBodySharedSettings;
        unpinned->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(0, 0, 0)));
        unpinned->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(1, 0, 0)));
        unpinned->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(5, 0, 0)));
        unpinned->mEdgeConstraints.push_back(SoftBodySharedSettings::Edge(0, 1));
        unpinned->mEdgeConstraints.push_back(SoftBodySharedSettings::Edge(0, 1));
        unpinned->CalculateEdgeLengths();
        SoftBodySharedSettings::OptimizationResults unpinned_results;
        unpinned->Optimize(unpinned_results);
        CHECK(IsPermutation(unpinned_results.mEdgeRemap));
        CHECK(unpinned->mVertices.size() == 3); // The isolated vertex is retained.
        CHECK(unpinned->mEdgeConstraints.size() == 2);
    }
}
