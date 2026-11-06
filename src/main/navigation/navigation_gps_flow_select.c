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

#include <string.h>

#include "navigation/navigation_gps_flow_select.h"

void gpsFlowSelectorReset(gpsFlowSelector_t *selector)
{
    memset(selector, 0, sizeof(*selector));
}

static bool elapsed(timeMs_t currentTimeMs, timeMs_t sinceMs, timeMs_t durationMs)
{
    return (timeMs_t)(currentTimeMs - sinceMs) >= durationMs;
}

// Tracks since when a condition has held continuously
static void trackCondition(bool condition, bool *state, timeMs_t *sinceMs, timeMs_t currentTimeMs)
{
    if (condition && !*state) {
        *sinceMs = currentTimeMs;
    }
    *state = condition;
}

bool gpsFlowSelectorUpdate(gpsFlowSelector_t *selector, const gpsFlowSelectorInput_t *input, float maxEph, timeMs_t currentTimeMs)
{
    // Without usable flow there is nothing better than GPS, however poor it is
    if (!(maxEph > 0.0f) || !input->flowUsable) {
        gpsFlowSelectorReset(selector);
        return false;
    }

    if (!input->gpsAvailable) {
        // GPS that comes back while flow is carrying position has to prove itself before it is used
        selector->gpsRejected = true;
        selector->reasons = GPS_FLOW_REASON_NO_GPS;
        selector->ephBad = false;
        selector->ephGood = false;
        selector->mismatch = false;
        return true;
    }

    // NaN accuracy counts as bad
    trackCondition(!(input->gpsEph <= maxEph), &selector->ephBad, &selector->ephBadSinceMs, currentTimeMs);
    trackCondition(input->gpsEph < maxEph * GPS_FLOW_RECOVER_EPH_RATIO, &selector->ephGood, &selector->ephGoodSinceMs, currentTimeMs);

    const bool mismatch = input->speedCompareValid && input->speedMismatch > GPS_FLOW_MISMATCH_CMS;
    trackCondition(mismatch, &selector->mismatch, &selector->mismatchSinceMs, currentTimeMs);

    if (selector->mismatch && elapsed(currentTimeMs, selector->mismatchSinceMs, GPS_FLOW_MISMATCH_DELAY_MS)) {
        selector->mismatchHold = true;
        selector->mismatchHoldUntilMs = currentTimeMs + GPS_FLOW_MISMATCH_HOLD_MS;
    }
    // Signed: the hold ends at a time in the future, which unsigned subtraction would wrap past
    else if (selector->mismatchHold && (int32_t)(currentTimeMs - selector->mismatchHoldUntilMs) >= 0) {
        selector->mismatchHold = false;
    }

    selector->reasons = GPS_FLOW_REASON_NONE;
    if (selector->ephBad && elapsed(currentTimeMs, selector->ephBadSinceMs, GPS_FLOW_REJECT_DELAY_MS)) {
        selector->reasons |= GPS_FLOW_REASON_EPH;
    }
    if (selector->mismatchHold) {
        selector->reasons |= GPS_FLOW_REASON_MISMATCH;
    }

    if (!selector->gpsRejected) {
        selector->gpsRejected = (selector->reasons != GPS_FLOW_REASON_NONE);
    }
    else if (selector->reasons == GPS_FLOW_REASON_NONE && selector->ephGood &&
             elapsed(currentTimeMs, selector->ephGoodSinceMs, GPS_FLOW_RECOVER_DELAY_MS)) {
        selector->gpsRejected = false;
    }

    return selector->gpsRejected;
}
