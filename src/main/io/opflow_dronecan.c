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
#include <math.h>

#include "platform.h"

#if defined(USE_OPFLOW_DRONECAN)

#include <dronecan_msgs.h>

#include "common/maths.h"

#include "drivers/opflow/opflow_virtual.h"
#include "io/opflow.h"

static bool hasNewData = false;
static opflowData_t sensorData = { 0 };

static bool dronecanOpflowDetect(void)
{
    // Messages can only be seen after init, so the sensor is assumed present
    return true;
}

static bool dronecanOpflowInit(void)
{
    return true;
}

static bool dronecanOpflowUpdate(opflowData_t * data)
{
    if (hasNewData) {
        hasNewData = false;
        *data = sensorData;
        return true;
    }

    return false;
}

void dronecanOpflowReceiveMeasurement(const struct com_hex_equipment_flow_Measurement * measurement)
{
    // opflowUpdate() divides by the interval, and anything longer than a second is far past
    // the opflow timeout anyway. Non-finite flow would propagate straight into the position estimate.
    if (!isfinite(measurement->integration_interval) ||
        measurement->integration_interval <= 0.0f || measurement->integration_interval > 1.0f ||
        !isfinite(measurement->flow_integral[0]) || !isfinite(measurement->flow_integral[1])) {
        return;
    }

    // Use the sensor's own integration window rather than the receive interval: CAN
    // arrival jitter would otherwise show up directly as flow rate noise
    const timeDelta_t deltaTimeUs = measurement->integration_interval * 1e6f;
    if (deltaTimeUs <= 0) {
        return;
    }

    // The sensor reports calibrated radians. opflow.c expects raw units that opflow_scale
    // converts to degrees, so pass degrees through and opflow_scale = 1.0 is exact
    sensorData.deltaTime = deltaTimeUs;
    // flow_integral is a right-handed rotation about each body axis (PX4/ArduPilot convention).
    // INAV's pitch rate has the opposite sign, so Y is negated here: a module mounted as its
    // arrow shows then needs align_opflow = DEFAULT (checked on an ARK Flow MR)
    sensorData.flowRateRaw[0] = RADIANS_TO_DEGREES(measurement->flow_integral[0]);
    sensorData.flowRateRaw[1] = -RADIANS_TO_DEGREES(measurement->flow_integral[1]);
    sensorData.flowRateRaw[2] = 0;
    sensorData.quality = (int)measurement->quality * 100 / 255;
    hasNewData = true;
}

virtualOpflowVTable_t opflowDronecanVtable = {
    .detect = dronecanOpflowDetect,
    .init = dronecanOpflowInit,
    .update = dronecanOpflowUpdate
};

#endif
