/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_RANGEFINDER_DRONECAN)

#include <dronecan_msgs.h>

#include "build/debug.h"

#include "drivers/rangefinder/rangefinder_virtual.h"
#include "io/rangefinder.h"

static bool hasNewData = false;
static int32_t sensorData = RANGEFINDER_NO_NEW_DATA;

// Flow modules and standalone lidars can share one bus, so only one sensor is followed
static bool isSensorLocked = false;
static uint8_t lockedNodeId;
static uint8_t lockedSensorId;

static bool dronecanRangefinderDetect(void)
{
    // Messages can only be seen after init, so the sensor is assumed present
    return true;
}

static void dronecanRangefinderInit(void)
{
}

static void dronecanRangefinderUpdate(void)
{
}

static int32_t dronecanRangefinderGetDistance(void)
{
    if (hasNewData) {
        hasNewData = false;
        return sensorData;
    }

    return RANGEFINDER_NO_NEW_DATA;
}

static bool isFromLockedSensor(uint8_t sourceNodeId, uint8_t sensorId)
{
    if (!isSensorLocked) {
        lockedNodeId = sourceNodeId;
        lockedSensorId = sensorId;
        isSensorLocked = true;
    }

    return sourceNodeId == lockedNodeId && sensorId == lockedSensorId;
}

void dronecanRangefinderReceiveMeasurement(uint8_t sourceNodeId, const struct uavcan_equipment_range_sensor_Measurement * measurement)
{
    static uint32_t acceptedCount = 0;
    static uint32_t ignoredCount = 0;

    if (!isFromLockedSensor(sourceNodeId, measurement->sensor_id)) {
        DEBUG_SET(DEBUG_AGL, 7, ++ignoredCount);
        return;
    }

    // Range is float16 on the wire, so it can be NaN or +inf: the comparisons are false
    // for NaN and the upper bound keeps the cast defined
    const float rangeCm = measurement->range * 100.0f;
    const int32_t reportedRangeCm = (rangeCm >= 1.0f && rangeCm < (float)INT32_MAX) ? (int32_t)rangeCm : RANGEFINDER_OUT_OF_RANGE;

    DEBUG_SET(DEBUG_AGL, 4, measurement->reading_type);
    DEBUG_SET(DEBUG_AGL, 5, reportedRangeCm);
    DEBUG_SET(DEBUG_AGL, 6, ++acceptedCount);

    if (measurement->reading_type == UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE) {
        sensorData = reportedRangeCm;
    } else {
        // TOO_CLOSE, TOO_FAR and UNDEFINED all mean there is no usable distance. UNDEFINED is
        // what PX4 sends for an untrusted return: keeping the last reading would freeze the
        // altitude and, after the hardware timeout, mark a sensor that is still talking as failed
        sensorData = RANGEFINDER_OUT_OF_RANGE;
    }

    hasNewData = true;
}

virtualRangefinderVTable_t rangefinderDronecanVtable = {
    .detect = dronecanRangefinderDetect,
    .init = dronecanRangefinderInit,
    .update = dronecanRangefinderUpdate,
    .read = dronecanRangefinderGetDistance
};

#endif
