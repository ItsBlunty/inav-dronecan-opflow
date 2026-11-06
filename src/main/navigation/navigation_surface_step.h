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

/* Terrain step detection for surface (rangefinder) altitude hold.
 *
 * A rangefinder reading that jumps away from the inertial AGL prediction for several samples in a
 * row, all on the same side, means the ground under the aircraft changed height (a window sill, a
 * table edge, a raised floor) rather than the aircraft moving. Surface hold then keeps the
 * aircraft at the same height in the air instead of chasing the new ground. */

#define SURFACE_STEP_CONFIRM_SAMPLES    3       // Consecutive readings needed to accept a step
#define SURFACE_STEP_RELEASE_RATE_CMS   50.0f   // Rate the held offset fades once the pilot moves throttle

typedef struct surfaceStepDetector_s {
    uint8_t count;      // Consecutive readings beyond the threshold
    int8_t sign;        // Side of the prediction those readings were on
} surfaceStepDetector_t;

void surfaceStepDetectorReset(surfaceStepDetector_t *detector);

// Feeds one rangefinder reading as its residual against the AGL prediction (cm). Returns the size of
// a step confirmed by this reading (cm, negative when the ground came closer), or 0. threshold <= 0
// disables detection.
float surfaceStepDetectorUpdate(surfaceStepDetector_t *detector, float residual, float threshold);

// True while readings disagree with the prediction but no step has been confirmed yet: the AGL
// estimate should not be corrected from the rangefinder meanwhile
bool surfaceStepDetectorIsSuspect(const surfaceStepDetector_t *detector);

// Moves a held offset toward zero by at most maxChange (cm), without overshooting
float surfaceStepOffsetRelease(float offset, float maxChange);
