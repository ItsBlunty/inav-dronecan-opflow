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

#include "navigation/navigation_surface_step.h"

void surfaceStepDetectorReset(surfaceStepDetector_t *detector)
{
    detector->count = 0;
    detector->sign = 0;
}

float surfaceStepDetectorUpdate(surfaceStepDetector_t *detector, float residual, float threshold)
{
    // Also rejects NaN: the comparison is false
    if (!(threshold > 0.0f) || !(fabsf(residual) > threshold)) {
        surfaceStepDetectorReset(detector);
        return 0.0f;
    }

    const int8_t sign = (residual > 0.0f) ? 1 : -1;
    if (detector->count > 0 && sign != detector->sign) {
        // Readings on both sides of the prediction are noise, not a step
        detector->count = 0;
    }

    detector->sign = sign;
    detector->count++;

    if (detector->count >= SURFACE_STEP_CONFIRM_SAMPLES) {
        surfaceStepDetectorReset(detector);
        return residual;
    }

    return 0.0f;
}

bool surfaceStepDetectorIsSuspect(const surfaceStepDetector_t *detector)
{
    return detector->count > 0;
}

float surfaceStepOffsetRelease(float offset, float maxChange)
{
    if (!(maxChange > 0.0f)) {
        return offset;
    }

    if (offset > maxChange) {
        return offset - maxChange;
    }

    if (offset < -maxChange) {
        return offset + maxChange;
    }

    return 0.0f;
}
