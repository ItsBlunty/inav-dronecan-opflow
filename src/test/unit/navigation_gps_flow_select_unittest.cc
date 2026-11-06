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
#include "navigation/navigation_gps_flow_select.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

class GpsFlowSelectTest : public ::testing::Test {
protected:
    gpsFlowSelector_t selector;
    gpsFlowSelectorInput_t input;
    const float maxEph = 150.0f;
    timeMs_t now = 1000;

    void SetUp() override {
        gpsFlowSelectorReset(&selector);
        input.flowUsable = true;
        input.gpsAvailable = true;
        input.gpsEph = 80.0f;
        input.speedCompareValid = true;
        input.speedMismatch = 10.0f;
    }

    // Runs the selector at 10 ms steps for durationMs, returning the last decision
    bool run(timeMs_t durationMs) {
        bool rejected = false;
        for (timeMs_t step = 0; step < durationMs; step += 10) {
            now += 10;
            rejected = gpsFlowSelectorUpdate(&selector, &input, maxEph, now);
        }
        return rejected;
    }
};

TEST_F(GpsFlowSelectTest, GoodGpsIsUsed)
{
    EXPECT_FALSE(run(10000));
}

TEST_F(GpsFlowSelectTest, PoorAccuracyRejectsGpsAfterDelay)
{
    input.gpsEph = 400.0f;
    EXPECT_FALSE(run(GPS_FLOW_REJECT_DELAY_MS - 20));
    EXPECT_TRUE(run(40));
    EXPECT_EQ(GPS_FLOW_REASON_EPH, selector.reasons);
}

TEST_F(GpsFlowSelectTest, BriefAccuracySpikeIsIgnored)
{
    input.gpsEph = 400.0f;
    run(GPS_FLOW_REJECT_DELAY_MS / 2);
    input.gpsEph = 80.0f;
    EXPECT_FALSE(run(5000));
}

TEST_F(GpsFlowSelectTest, GpsReturnsOnlyAfterSustainedGoodAccuracy)
{
    input.gpsEph = 400.0f;
    ASSERT_TRUE(run(1000));

    // Under the threshold but not clearly good: stays on flow
    input.gpsEph = 120.0f;
    EXPECT_TRUE(run(10000));

    input.gpsEph = 80.0f;
    EXPECT_TRUE(run(GPS_FLOW_RECOVER_DELAY_MS - 20));
    EXPECT_FALSE(run(40));
}

TEST_F(GpsFlowSelectTest, AccuracyDipDuringRecoveryRestartsTheWait)
{
    input.gpsEph = 400.0f;
    ASSERT_TRUE(run(1000));

    input.gpsEph = 80.0f;
    run(GPS_FLOW_RECOVER_DELAY_MS - 500);
    input.gpsEph = 120.0f;
    run(100);
    input.gpsEph = 80.0f;
    EXPECT_TRUE(run(GPS_FLOW_RECOVER_DELAY_MS - 100));
    EXPECT_FALSE(run(200));
}

TEST_F(GpsFlowSelectTest, VelocityMismatchRejectsGpsWithGoodAccuracy)
{
    input.speedMismatch = 120.0f;
    EXPECT_FALSE(run(GPS_FLOW_MISMATCH_DELAY_MS - 20));
    EXPECT_TRUE(run(40));
    EXPECT_EQ(GPS_FLOW_REASON_MISMATCH, selector.reasons);

    // Agreement again: held off for the hold time, then the recovery wait
    input.speedMismatch = 10.0f;
    EXPECT_TRUE(run(GPS_FLOW_MISMATCH_HOLD_MS - 20));
    EXPECT_FALSE(run(GPS_FLOW_RECOVER_DELAY_MS));
}

TEST_F(GpsFlowSelectTest, MismatchIsIgnoredWhenComparisonInvalid)
{
    input.speedMismatch = 500.0f;
    input.speedCompareValid = false;
    EXPECT_FALSE(run(5000));
}

TEST_F(GpsFlowSelectTest, NoUsableFlowAlwaysUsesGps)
{
    input.gpsEph = 900.0f;
    input.speedMismatch = 500.0f;
    input.flowUsable = false;
    EXPECT_FALSE(run(5000));
}

TEST_F(GpsFlowSelectTest, ZeroThresholdDisablesSelector)
{
    input.gpsEph = 900.0f;
    input.speedMismatch = 500.0f;
    bool rejected = true;
    for (int i = 0; i < 500; i++) {
        now += 10;
        rejected = gpsFlowSelectorUpdate(&selector, &input, 0.0f, now);
    }
    EXPECT_FALSE(rejected);
}

TEST_F(GpsFlowSelectTest, GpsReacquiredOnFlowMustProveItself)
{
    input.gpsAvailable = false;
    EXPECT_TRUE(run(1000));
    EXPECT_EQ(GPS_FLOW_REASON_NO_GPS, selector.reasons);

    input.gpsAvailable = true;
    EXPECT_TRUE(run(GPS_FLOW_RECOVER_DELAY_MS - 20));
    EXPECT_FALSE(run(40));
}

TEST_F(GpsFlowSelectTest, NanAccuracyCountsAsBad)
{
    input.gpsEph = NAN;
    EXPECT_TRUE(run(1000));
}

TEST_F(GpsFlowSelectTest, TimerWrapIsHandled)
{
    now = 0xFFFFFFFFu - 200;
    input.gpsEph = 400.0f;
    EXPECT_TRUE(run(1000));
}
