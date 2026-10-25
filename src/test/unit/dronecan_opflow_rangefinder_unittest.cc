/**
 * DroneCAN optical flow and rangefinder driver tests
 *
 * Each test encodes a message the way a peripheral would put it on the bus, decodes it with
 * the generated DSDL code, and checks what the driver hands to the opflow / rangefinder layer.
 */

#include <cstdint>
#include <cstring>
#include <cmath>
#include <limits>

extern "C" {
#include "platform.h"

#include "drivers/dronecan/libcanard/canard.h"
#include "com.hex.equipment.flow.Measurement.h"
#include "uavcan.equipment.range_sensor.Measurement.h"

#include "common/maths.h"
#include "io/opflow.h"
#include "io/rangefinder.h"
}

#include "gtest/gtest.h"

static CanardRxTransfer makeTransfer(uint8_t *buffer, uint32_t len)
{
    CanardRxTransfer transfer;
    memset(&transfer, 0, sizeof(transfer));
    transfer.payload_len = len;
    transfer.payload_head = buffer;
    return transfer;
}

// ===========================================================================
// Optical flow
// ===========================================================================

class DroneCANOpflowTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        // Drain anything a previous test left latched
        opflowData_t discard;
        opflowDronecanVtable.update(&discard);
        memset(&data, 0, sizeof(data));
    }

    void sendOverBus(float interval, float flowX, float flowY, uint8_t quality)
    {
        struct com_hex_equipment_flow_Measurement tx;
        memset(&tx, 0, sizeof(tx));
        tx.integration_interval = interval;
        tx.flow_integral[0] = flowX;
        tx.flow_integral[1] = flowY;
        tx.rate_gyro_integral[0] = flowX;
        tx.rate_gyro_integral[1] = flowY;
        tx.quality = quality;

        uint8_t buffer[COM_HEX_EQUIPMENT_FLOW_MEASUREMENT_MAX_SIZE];
        const uint32_t len = com_hex_equipment_flow_Measurement_encode(&tx, buffer);
        CanardRxTransfer transfer = makeTransfer(buffer, len);

        struct com_hex_equipment_flow_Measurement rx;
        ASSERT_FALSE(com_hex_equipment_flow_Measurement_decode(&transfer, &rx));
        dronecanOpflowReceiveMeasurement(&rx);
    }

    opflowData_t data;
};

TEST_F(DroneCANOpflowTest, DetectAndInitSucceed)
{
    EXPECT_TRUE(opflowDronecanVtable.detect());
    EXPECT_TRUE(opflowDronecanVtable.init());
}

TEST_F(DroneCANOpflowTest, NoDataBeforeFirstMessage)
{
    EXPECT_FALSE(opflowDronecanVtable.update(&data));
}

TEST_F(DroneCANOpflowTest, MeasurementIsConvertedToDegreesOverSensorInterval)
{
    sendOverBus(0.02f, 0.01f, -0.02f, 255);

    ASSERT_TRUE(opflowDronecanVtable.update(&data));
    EXPECT_EQ(data.deltaTime, 20000);
    EXPECT_NEAR(data.flowRateRaw[0], RADIANS_TO_DEGREES(0.01f), 1e-4f);
    EXPECT_NEAR(data.flowRateRaw[1], RADIANS_TO_DEGREES(-0.02f), 1e-4f);
    EXPECT_EQ(data.flowRateRaw[2], 0.0f);
    EXPECT_EQ(data.quality, 100);
}

TEST_F(DroneCANOpflowTest, EachMeasurementIsDeliveredOnce)
{
    sendOverBus(0.01f, 0.0f, 0.0f, 200);

    EXPECT_TRUE(opflowDronecanVtable.update(&data));
    EXPECT_FALSE(opflowDronecanVtable.update(&data));
}

TEST_F(DroneCANOpflowTest, QualityIsScaledToPercent)
{
    sendOverBus(0.01f, 0.0f, 0.0f, 0);
    ASSERT_TRUE(opflowDronecanVtable.update(&data));
    EXPECT_EQ(data.quality, 0);

    sendOverBus(0.01f, 0.0f, 0.0f, 128);
    ASSERT_TRUE(opflowDronecanVtable.update(&data));
    EXPECT_EQ(data.quality, 50);
}

// opflow.c computes rate = flowRateRaw * (1e6 / deltaTime) / opflow_scale, then converts
// degrees to radians. With opflow_scale = 1.0 that must equal the sensor's own LOS rate.
TEST_F(DroneCANOpflowTest, UnitScaleReproducesSensorRate)
{
    const float interval = 0.025f;
    const float flowIntegral = 0.05f;
    sendOverBus(interval, flowIntegral, 0.0f, 255);
    ASSERT_TRUE(opflowDronecanVtable.update(&data));

    const float opflowScale = 1.0f;
    const float rateRadPerSec = DEGREES_TO_RADIANS(data.flowRateRaw[0] * (1.0e6f / data.deltaTime) / opflowScale);
    EXPECT_NEAR(rateRadPerSec, flowIntegral / interval, 1e-4f);
}

TEST_F(DroneCANOpflowTest, InvalidIntervalsAreDropped)
{
    const float invalid[] = {
        0.0f, -0.01f, 1.5f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(), 1e-9f
    };

    for (float interval : invalid) {
        sendOverBus(interval, 0.01f, 0.01f, 255);
        EXPECT_FALSE(opflowDronecanVtable.update(&data)) << "interval " << interval;
    }
}

TEST_F(DroneCANOpflowTest, NonFiniteFlowIsDropped)
{
    const float invalid[] = {
        std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()
    };

    for (float flow : invalid) {
        sendOverBus(0.02f, flow, 0.0f, 255);
        EXPECT_FALSE(opflowDronecanVtable.update(&data)) << "flow X " << flow;

        sendOverBus(0.02f, 0.0f, flow, 255);
        EXPECT_FALSE(opflowDronecanVtable.update(&data)) << "flow Y " << flow;
    }
}

// ===========================================================================
// Rangefinder
// ===========================================================================

// The driver follows the first (node, sensor_id) it hears. Every test uses this sensor so the
// lock lands on it regardless of which test runs first.
static const uint8_t LOCKED_NODE_ID = 20;
static const uint8_t LOCKED_SENSOR_ID = 0;

class DroneCANRangefinderTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        rangefinderDronecanVtable.read();
    }

    void sendOverBus(uint8_t readingType, float rangeM,
                     uint8_t nodeId = LOCKED_NODE_ID, uint8_t sensorId = LOCKED_SENSOR_ID)
    {
        struct uavcan_equipment_range_sensor_Measurement tx;
        memset(&tx, 0, sizeof(tx));
        tx.sensor_id = sensorId;
        tx.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_LIDAR;
        tx.reading_type = readingType;
        tx.range = rangeM;

        uint8_t buffer[UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_MAX_SIZE];
        const uint32_t len = uavcan_equipment_range_sensor_Measurement_encode(&tx, buffer);
        CanardRxTransfer transfer = makeTransfer(buffer, len);

        struct uavcan_equipment_range_sensor_Measurement rx;
        ASSERT_FALSE(uavcan_equipment_range_sensor_Measurement_decode(&transfer, &rx));
        dronecanRangefinderReceiveMeasurement(nodeId, &rx);
    }
};

TEST_F(DroneCANRangefinderTest, DetectSucceeds)
{
    EXPECT_TRUE(rangefinderDronecanVtable.detect());
}

TEST_F(DroneCANRangefinderTest, NoDataBeforeFirstMessage)
{
    EXPECT_EQ(rangefinderDronecanVtable.read(), RANGEFINDER_NO_NEW_DATA);
}

TEST_F(DroneCANRangefinderTest, ValidRangeIsReportedInCentimetres)
{
    sendOverBus(UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE, 1.5f);

    EXPECT_EQ(rangefinderDronecanVtable.read(), 150);
    EXPECT_EQ(rangefinderDronecanVtable.read(), RANGEFINDER_NO_NEW_DATA);
}

TEST_F(DroneCANRangefinderTest, TooFarAndTooCloseAreOutOfRange)
{
    sendOverBus(UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_FAR, 40.0f);
    EXPECT_EQ(rangefinderDronecanVtable.read(), RANGEFINDER_OUT_OF_RANGE);

    sendOverBus(UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_CLOSE, 0.05f);
    EXPECT_EQ(rangefinderDronecanVtable.read(), RANGEFINDER_OUT_OF_RANGE);
}

TEST_F(DroneCANRangefinderTest, UndefinedReadingIsIgnored)
{
    sendOverBus(UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_UNDEFINED, 2.0f);

    EXPECT_EQ(rangefinderDronecanVtable.read(), RANGEFINDER_NO_NEW_DATA);
}

TEST_F(DroneCANRangefinderTest, NonPositiveOrNonFiniteValidRangeIsOutOfRange)
{
    const float invalid[] = {
        0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()
    };

    for (float range : invalid) {
        sendOverBus(UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE, range);
        EXPECT_EQ(rangefinderDronecanVtable.read(), RANGEFINDER_OUT_OF_RANGE) << "range " << range;
    }
}

TEST_F(DroneCANRangefinderTest, OtherSensorsOnTheBusAreIgnored)
{
    sendOverBus(UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE, 1.0f);
    EXPECT_EQ(rangefinderDronecanVtable.read(), 100);

    sendOverBus(UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE, 7.0f, LOCKED_NODE_ID + 1, LOCKED_SENSOR_ID);
    EXPECT_EQ(rangefinderDronecanVtable.read(), RANGEFINDER_NO_NEW_DATA);

    sendOverBus(UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE, 7.0f, LOCKED_NODE_ID, LOCKED_SENSOR_ID + 1);
    EXPECT_EQ(rangefinderDronecanVtable.read(), RANGEFINDER_NO_NEW_DATA);

    sendOverBus(UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE, 2.0f);
    EXPECT_EQ(rangefinderDronecanVtable.read(), 200);
}
