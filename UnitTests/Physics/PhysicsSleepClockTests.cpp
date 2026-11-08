// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include "UnitTestFramework.h"
#include "PhysicsTestContext.h"
#include "Layers.h"

TEST_SUITE("PhysicsSleepClockTests")
{
	static int SimulateUntilSleep(PhysicsTestContext &ioContext, Body &inBody, int inMaxFrames)
	{
		for (int frame = 1; frame <= inMaxFrames; ++frame)
		{
			CHECK(ioContext.SimulateSingleStep() == EPhysicsUpdateError::None);
			if (!inBody.IsActive())
				return frame;
		}

		return 0;
	}

	TEST_CASE("TestFullUpdateSleepClockAcrossCollisionSteps")
	{
		for (int collision_steps : { 1, 4, 8 })
		{
			PhysicsTestContext context(1.0f / 60.0f, collision_steps);
			context.ZeroGravity();
			context.CreateFloor();

			PhysicsSettings settings = context.GetSystem()->GetPhysicsSettings();
			settings.mUseFullUpdateDeltaForSleep = true;
			context.GetSystem()->SetPhysicsSettings(settings);

			// Start at rest on the floor so the sleep test sees no movement across its full interval.
			Body &body = context.CreateBox(RVec3(0, 0.5f, 0), Quat::sIdentity(), EMotionType::Dynamic, EMotionQuality::Discrete, Layers::MOVING, Vec3::sReplicate(0.5f));
			CHECK(body.IsActive());
			int sleep_frame = SimulateUntilSleep(context, body, 35);
			CHECK(sleep_frame >= 30);
			CHECK(sleep_frame <= 31);
			CHECK(float(sleep_frame) * context.GetDeltaTime() >= settings.mTimeBeforeSleep);
			CHECK(float(sleep_frame - 1) * context.GetDeltaTime() <= settings.mTimeBeforeSleep);

			// A sleeping body must still activate and respond to a later impulse.
			context.GetBodyInterface().AddImpulse(body.GetID(), Vec3::sAxisX());
			CHECK(body.IsActive());
			CHECK(context.SimulateSingleStep() == EPhysicsUpdateError::None);
			CHECK(body.IsActive());
			CHECK(body.GetLinearVelocity().GetX() > 0.0f);
		}
	}

	TEST_CASE("TestDefaultSleepClockPreserved")
	{
		for (int collision_steps : { 1, 4, 8 })
		{
			PhysicsTestContext context(1.0f / 60.0f, collision_steps);
			context.ZeroGravity();
			context.CreateFloor();
			CHECK_FALSE(context.GetSystem()->GetPhysicsSettings().mUseFullUpdateDeltaForSleep);

			Body &body = context.CreateBox(RVec3(0, 0.5f, 0), Quat::sIdentity(), EMotionType::Dynamic, EMotionQuality::Discrete, Layers::MOVING, Vec3::sReplicate(0.5f));
			int sleep_frame = SimulateUntilSleep(context, body, 32 * collision_steps);
			CHECK(sleep_frame >= 30 * collision_steps);
			CHECK(sleep_frame <= 31 * collision_steps);
		}
	}

	TEST_CASE("TestLoadedFloorSleepsAfterElapsedTime")
	{
		for (int collision_steps : { 1, 4, 8 })
		{
			PhysicsTestContext context(1.0f / 60.0f, collision_steps);
			context.CreateFloor();

			PhysicsSettings settings = context.GetSystem()->GetPhysicsSettings();
			settings.mUseFullUpdateDeltaForSleep = true;
			context.GetSystem()->SetPhysicsSettings(settings);

			Body &body = context.CreateBox(RVec3(0, 0.5f, 0), Quat::sIdentity(), EMotionType::Dynamic, EMotionQuality::Discrete, Layers::MOVING, Vec3::sReplicate(0.5f));
			body.SetAllowSleeping(false);
			CHECK(context.Simulate(2.0f) == EPhysicsUpdateError::None);
			CHECK(body.IsActive());
			body.SetAllowSleeping(true); // Reset the sleep timer after the initial contact settles.

			int sleep_frame = SimulateUntilSleep(context, body, 45);
			CHECK(sleep_frame >= 30);
			CHECK(sleep_frame <= 45);
		}
	}
}
