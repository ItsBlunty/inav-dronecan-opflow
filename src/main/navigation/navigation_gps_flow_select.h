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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

/* Chooses between GPS and optical flow for horizontal position.
 *
 * INAV normally uses GPS whenever its reported accuracy is within inav_max_eph_epv (10 m by default),
 * which indoors or next to buildings means following a GPS position that wanders by metres while the
 * flow sensor knows better. While flow is usable, this rejects GPS on either of:
 *  - its reported horizontal accuracy (EPH) being worse than a threshold, or
 *  - GPS velocity disagreeing with flow velocity, which catches reflected signals that still report a
 *    good accuracy.
 * GPS is taken back only after its accuracy has been clearly good, and the velocities have agreed, for
 * a few seconds, so a doorway does not make it flip back and forth. */

#define GPS_FLOW_REJECT_DELAY_MS        500     // EPH beyond the threshold for this long rejects GPS
#define GPS_FLOW_RECOVER_DELAY_MS       3000    // EPH clearly good for this long takes GPS back
#define GPS_FLOW_RECOVER_EPH_RATIO      0.66f   // "Clearly good": EPH below this fraction of the threshold
#define GPS_FLOW_MISMATCH_CMS           50.0f   // GPS and flow velocity disagreeing by more than this...
#define GPS_FLOW_MISMATCH_DELAY_MS      1000    // ...for this long rejects GPS...
#define GPS_FLOW_MISMATCH_HOLD_MS       3000    // ...for at least this long after they agree again
#define GPS_FLOW_COMPARE_MAX_SPEED_CMS  300.0f  // Velocities are only compared below this (flow saturates fast and low)

typedef enum {
    GPS_FLOW_REASON_NONE        = 0,
    GPS_FLOW_REASON_EPH         = (1 << 0),
    GPS_FLOW_REASON_MISMATCH    = (1 << 1),
    GPS_FLOW_REASON_NO_GPS      = (1 << 2),
} gpsFlowRejectReason_e;

typedef struct gpsFlowSelectorInput_s {
    bool flowUsable;            // Flow can carry position on its own (healthy, lidar in range, dead reckoning allowed)
    bool gpsAvailable;          // GPS passes INAV's normal validity checks
    float gpsEph;               // GPS reported horizontal accuracy (cm)
    bool speedCompareValid;     // speedMismatch is meaningful this update
    float speedMismatch;        // Filtered magnitude of GPS minus flow velocity (cm/s)
} gpsFlowSelectorInput_t;

typedef struct gpsFlowSelector_s {
    bool gpsRejected;
    uint8_t reasons;            // gpsFlowRejectReason_e bits active this update

    bool ephBad;
    timeMs_t ephBadSinceMs;
    bool ephGood;
    timeMs_t ephGoodSinceMs;
    bool mismatch;
    timeMs_t mismatchSinceMs;
    bool mismatchHold;
    timeMs_t mismatchHoldUntilMs;
} gpsFlowSelector_t;

void gpsFlowSelectorReset(gpsFlowSelector_t *selector);

// Returns true while GPS should not be used for horizontal position. maxEph <= 0 disables the selector.
bool gpsFlowSelectorUpdate(gpsFlowSelector_t *selector, const gpsFlowSelectorInput_t *input, float maxEph, timeMs_t currentTimeMs);
