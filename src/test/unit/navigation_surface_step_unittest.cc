/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>

extern "C" {
#include "navigation/navigation_surface_step.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

class SurfaceStepTest : public ::testing::Test {
protected:
    surfaceStepDetector_t detector;
    const float threshold = 25.0f;

    void SetUp() override {
        surfaceStepDetectorReset(&detector);
    }

    // Feeds the same residual repeatedly, returning the last result
    float feed(float residual, int samples) {
        float step = 0.0f;
        for (int i = 0; i < samples; i++) {
            step = surfaceStepDetectorUpdate(&detector, residual, threshold);
        }
        return step;
    }
};

TEST_F(SurfaceStepTest, ReadingsWithinThresholdAreNotSteps)
{
    for (int i = 0; i < 20; i++) {
        EXPECT_EQ(0.0f, surfaceStepDetectorUpdate(&detector, (i % 2) ? 24.0f : -24.0f, threshold));
        EXPECT_FALSE(surfaceStepDetectorIsSuspect(&detector));
    }
}

TEST_F(SurfaceStepTest, SillIsConfirmedAfterConsecutiveReadings)
{
    // Ground 110 cm closer, as when a window sill passes under the aircraft
    for (int i = 1; i < SURFACE_STEP_CONFIRM_SAMPLES; i++) {
        EXPECT_EQ(0.0f, surfaceStepDetectorUpdate(&detector, -110.0f, threshold));
        EXPECT_TRUE(surfaceStepDetectorIsSuspect(&detector));
    }

    EXPECT_EQ(-110.0f, surfaceStepDetectorUpdate(&detector, -110.0f, threshold));
    EXPECT_FALSE(surfaceStepDetectorIsSuspect(&detector));
}

TEST_F(SurfaceStepTest, StepOffSillCancelsStepOnto)
{
    // Once the first step is taken the prediction sits on the sill, so leaving it reads as the opposite step
    const float onto = feed(-110.0f, SURFACE_STEP_CONFIRM_SAMPLES);
    const float off = feed(110.0f, SURFACE_STEP_CONFIRM_SAMPLES);

    EXPECT_EQ(0.0f, onto + off);
}

TEST_F(SurfaceStepTest, ShortGlitchIsRejected)
{
    feed(-110.0f, SURFACE_STEP_CONFIRM_SAMPLES - 1);
    EXPECT_TRUE(surfaceStepDetectorIsSuspect(&detector));

    EXPECT_EQ(0.0f, surfaceStepDetectorUpdate(&detector, 3.0f, threshold));
    EXPECT_FALSE(surfaceStepDetectorIsSuspect(&detector));

    // The count started over, so one more far reading is not enough
    EXPECT_EQ(0.0f, surfaceStepDetectorUpdate(&detector, -110.0f, threshold));
}

TEST_F(SurfaceStepTest, ReadingsOnBothSidesAreNotAStep)
{
    for (int i = 0; i < 20; i++) {
        EXPECT_EQ(0.0f, surfaceStepDetectorUpdate(&detector, (i % 2) ? 60.0f : -60.0f, threshold));
    }
}

TEST_F(SurfaceStepTest, ZeroThresholdDisablesDetection)
{
    for (int i = 0; i < 20; i++) {
        EXPECT_EQ(0.0f, surfaceStepDetectorUpdate(&detector, -300.0f, 0.0f));
        EXPECT_FALSE(surfaceStepDetectorIsSuspect(&detector));
    }
}

TEST_F(SurfaceStepTest, NanReadingIsNotAStep)
{
    feed(-110.0f, SURFACE_STEP_CONFIRM_SAMPLES - 1);

    EXPECT_EQ(0.0f, surfaceStepDetectorUpdate(&detector, NAN, threshold));
    EXPECT_FALSE(surfaceStepDetectorIsSuspect(&detector));
}

TEST_F(SurfaceStepTest, OffsetReleasesTowardZeroWithoutOvershoot)
{
    EXPECT_FLOAT_EQ(45.0f, surfaceStepOffsetRelease(50.0f, 5.0f));
    EXPECT_FLOAT_EQ(-45.0f, surfaceStepOffsetRelease(-50.0f, 5.0f));
    EXPECT_EQ(0.0f, surfaceStepOffsetRelease(3.0f, 5.0f));
    EXPECT_EQ(0.0f, surfaceStepOffsetRelease(-3.0f, 5.0f));
    EXPECT_FLOAT_EQ(50.0f, surfaceStepOffsetRelease(50.0f, 0.0f));
}
