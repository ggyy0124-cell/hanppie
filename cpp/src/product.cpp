#include "hanppie/product.hpp"

namespace hanppie {

namespace {

const std::uint16_t KNOWN_COMPONENTS[] = {
    0x0100, 0x0104, 0x0300, 0x030A, 0x0314, 0x0315, 0x0316, 0x0317, 0x0318, 0x0319, 0x031A,
    0x031B, 0x031E, 0x031F, 0x0400, 0x1201, 0x1202, 0x1203, 0x1204, 0x1601, 0x1602, 0x1603,
    0x1604, 0x1605, 0x1606, 0x1700, 0x1701, 0x1801, 0x1802, 0x1803, 0x1804, 0x1805, 0x1806,
};

std::uint16_t readLe16(const Bytes& data, std::size_t offset) {
    return static_cast<std::uint16_t>(data[offset] | (data[offset + 1] << 8));
}

}  // namespace

const char* robotModelName(RobotModel model) {
    switch (model) {
        case RobotModel::ROBOMASTER_S1:
            return "robomaster_s1";
        case RobotModel::ROBOMASTER_EP:
            return "robomaster_ep";
        default:
            return "unknown";
    }
}

bool WorkingRobotDevice::component(RobotComponent& out) const {
    for (std::size_t i = 0; i < sizeof(KNOWN_COMPONENTS) / sizeof(KNOWN_COMPONENTS[0]); ++i) {
        if (KNOWN_COMPONENTS[i] == deviceId) {
            out = static_cast<RobotComponent>(deviceId);
            return true;
        }
    }
    return false;
}

bool WorkingRobotDevice::operator==(const WorkingRobotDevice& other) const {
    return deviceId == other.deviceId && details == other.details;
}

std::set<RobotComponent> RobotCapabilities::components() const {
    std::set<RobotComponent> result;
    for (std::size_t i = 0; i < workingDevices.size(); ++i) {
        RobotComponent component;
        if (workingDevices[i].component(component)) result.insert(component);
    }
    return result;
}

std::set<std::uint16_t> RobotCapabilities::unknownDeviceIds() const {
    std::set<std::uint16_t> result;
    for (std::size_t i = 0; i < workingDevices.size(); ++i) {
        RobotComponent component;
        if (!workingDevices[i].component(component)) result.insert(workingDevices[i].deviceId);
    }
    return result;
}

bool RobotCapabilities::operator==(const RobotCapabilities& other) const {
    return workingDevices == other.workingDevices;
}

bool RobotProduct::operator==(const RobotProduct& other) const {
    return model == other.model && capabilities == other.capabilities;
}

bool decodeModel(const protocol::DussFrame& frame, RobotModel& out) {
    if (!frame.valid || frame.attr != protocol::ATTR_RESP_NEED_ACK ||
        frame.cmdset != protocol::CMDSET_RM ||
        frame.cmdid != protocol::CMD_RM_PRODUCT_ATTRIBUTE_GET || frame.payload.size() < 2) {
        return false;
    }
    switch (frame.payload[1]) {
        case 1:
            out = RobotModel::ROBOMASTER_S1;
            break;
        case 2:
            out = RobotModel::ROBOMASTER_EP;
            break;
        default:
            out = RobotModel::UNKNOWN;
            break;
    }
    return true;
}

bool decodeCapabilities(const protocol::DussFrame& frame, RobotCapabilities& out) {
    if (!frame.valid || frame.attr != protocol::ATTR_NO_ACK ||
        frame.cmdset != protocol::CMDSET_RM ||
        frame.cmdid != protocol::CMD_RM_MODULE_STATUS_PUSH || frame.payload.empty()) {
        return false;
    }
    const Bytes& payload = frame.payload;
    std::size_t expected = payload[0];
    std::size_t cursor = 1;
    RobotCapabilities result;
    for (std::size_t n = 0; n < expected; ++n) {
        if (cursor + 3 > payload.size()) return false;
        WorkingRobotDevice device;
        device.deviceId = readLe16(payload, cursor);
        std::size_t detailCount = payload[cursor + 2];
        cursor += 3;
        if (cursor + detailCount * 2 > payload.size()) return false;
        for (std::size_t i = 0; i < detailCount; ++i) {
            device.details.push_back(readLe16(payload, cursor + i * 2));
        }
        cursor += detailCount * 2;
        result.workingDevices.push_back(device);
    }
    out = result;
    return true;
}

bool updatedProduct(const RobotProduct& current, const protocol::DussFrame& frame,
                    RobotProduct& out) {
    RobotModel model;
    RobotCapabilities capabilities;
    bool hasModel = decodeModel(frame, model);
    bool hasCapabilities = decodeCapabilities(frame, capabilities);
    if (!hasModel && !hasCapabilities) return false;
    RobotProduct updated = current;
    if (hasModel) updated.model = model;
    if (hasCapabilities) updated.capabilities = capabilities;
    if (updated == current) return false;
    out = updated;
    return true;
}

}  // namespace hanppie
