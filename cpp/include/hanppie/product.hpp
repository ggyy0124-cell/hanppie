// RoboMaster product and working-device messages carried by the App session.
// C++11 port of src/hanppie/product.py.
#ifndef HANPPIE_PRODUCT_HPP
#define HANPPIE_PRODUCT_HPP

#include <cstdint>
#include <set>
#include <vector>

#include "hanppie/protocol.hpp"

namespace hanppie {

enum class RobotModel { UNKNOWN, ROBOMASTER_S1, ROBOMASTER_EP };

const char* robotModelName(RobotModel model);

enum class RobotComponent : std::uint16_t {
    IMAGE_TRANSMISSION = 0x0100,
    CAMERA = 0x0104,
    CHASSIS = 0x0300,
    BATTERY = 0x030A,
    ESC_0 = 0x0314,
    ESC_1 = 0x0315,
    ESC_2 = 0x0316,
    ESC_3 = 0x0317,
    SERVO_1 = 0x0318,
    SERVO_2 = 0x0319,
    SERVO_3 = 0x031A,
    SERVO_4 = 0x031B,
    ARM = 0x031E,
    CLAW = 0x031F,
    GIMBAL = 0x0400,
    TOF_1 = 0x1201,
    TOF_2 = 0x1202,
    TOF_3 = 0x1203,
    TOF_4 = 0x1204,
    SENSOR_ADAPTER_1 = 0x1601,
    SENSOR_ADAPTER_2 = 0x1602,
    SENSOR_ADAPTER_3 = 0x1603,
    SENSOR_ADAPTER_4 = 0x1604,
    SENSOR_ADAPTER_5 = 0x1605,
    SENSOR_ADAPTER_6 = 0x1606,
    WATER_GUN = 0x1700,
    INFRARED_GUN = 0x1701,
    BACK_ARMOR = 0x1801,
    FRONT_ARMOR = 0x1802,
    LEFT_ARMOR = 0x1803,
    RIGHT_ARMOR = 0x1804,
    LEFT_HEAD_ARMOR = 0x1805,
    RIGHT_HEAD_ARMOR = 0x1806,
};

struct WorkingRobotDevice {
    std::uint16_t deviceId;
    std::vector<std::uint16_t> details;

    // Returns false when deviceId is not a known RobotComponent.
    bool component(RobotComponent& out) const;
    bool operator==(const WorkingRobotDevice& other) const;
};

struct RobotCapabilities {
    std::vector<WorkingRobotDevice> workingDevices;

    std::set<RobotComponent> components() const;
    std::set<std::uint16_t> unknownDeviceIds() const;
    bool operator==(const RobotCapabilities& other) const;
};

struct RobotProduct {
    RobotProduct() : model(RobotModel::UNKNOWN) {}

    RobotModel model;
    RobotCapabilities capabilities;

    bool operator==(const RobotProduct& other) const;
    bool operator!=(const RobotProduct& other) const { return !(*this == other); }
};

bool decodeModel(const protocol::DussFrame& frame, RobotModel& out);
bool decodeCapabilities(const protocol::DussFrame& frame, RobotCapabilities& out);
// Returns true and writes out when frame changes the current product.
bool updatedProduct(const RobotProduct& current, const protocol::DussFrame& frame,
                    RobotProduct& out);

}  // namespace hanppie

#endif  // HANPPIE_PRODUCT_HPP
