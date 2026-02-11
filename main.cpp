#include "stark-sdk.h"
#include "param.h"

#include "dds/Publisher.h"
#include "dds/Subscription.h"
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/common/thread/recurrent_thread.hpp>

#include <iostream>
#include <csignal>
#include <chrono>
#include <thread>
#include <algorithm>
#include <atomic>
#include <filesystem>
#include <mutex>

std::atomic<bool> running(true);
void signal_handler(int) { running = false; }

// Brainco Hand ID
constexpr uint8_t L_id = 0x7e;
constexpr uint8_t R_id = 0x7f;
constexpr uint32_t baudrate = 460800;

// Motor layout: combined 12-motor message (right 0-5, left 6-11)
constexpr int kMotorsPerHand = 6;
constexpr int kTotalMotors = 12;

// Shared DDS resources for the combined ee topic
struct SharedDDS {
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>> cmd_sub;
    std::unique_ptr<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>> state_pub;
};

// ------------------ Utility ------------------
std::vector<std::string> getAvailableSerialPorts() {
    std::vector<std::string> ports;
    for (const auto& entry : std::filesystem::directory_iterator("/dev")) {
        std::string path = entry.path().string();
        if (path.rfind("/dev/ttyUSB", 0) == 0) ports.push_back(path);
    }
    spdlog::info("Available Serial Ports: {}", fmt::join(ports, ", "));
    return ports;
}


// Hand connection struct
struct HandConnection {
    DeviceHandler* handle{nullptr};
    DeviceInfo* info{nullptr};
    std::string port;
};

// Try connecting a hand on a given port
HandConnection try_connect_hand(const std::string& port, uint8_t slave_id) {
    HandConnection ret;
    ret.port = port;

    DeviceHandler* handle = modbus_open(port.c_str(), baudrate);
    if (!handle) {
        spdlog::warn("Failed to open {} port", port);
        return ret;
    }

    DeviceInfo* info = modbus_get_device_info(handle, slave_id);
    if (!info) {
        spdlog::warn("Failed to get device info from {} port", port);
        modbus_close(handle);
        return ret;
    }

    modbus_set_finger_unit_mode(handle, slave_id, FINGER_UNIT_MODE_NORMALIZED);

    ret.handle = handle;
    ret.info = info;
    return ret;
}

// Find a hand from available ports given allowed SKUs
HandConnection find_hand(std::vector<std::string>& ports, uint8_t slave_id, const std::vector<SkuType>& allowed_skus, const std::string& hand_name) {
    for (const auto& port : ports) {
        spdlog::info("Trying port for {} hand from {} port", hand_name, port);
        auto conn = try_connect_hand(port, slave_id);
        if (!conn.handle) continue;

        if (std::find(allowed_skus.begin(), allowed_skus.end(), conn.info->sku_type) != allowed_skus.end()) {
            spdlog::info("{} hand bound to {} port", hand_name, port);
            ports.erase(std::remove(ports.begin(), ports.end(), port), ports.end());
            return conn;
        } else {
            modbus_close(conn.handle);
            spdlog::warn("Port {} is not {} hand (sku {}). Closed.", port, hand_name, (int)conn.info->sku_type);
        }
    }
    return {};
}

// ------------------ Hand Update Loop ------------------
// Each hand reads its 6-motor slice from the combined 12-motor command,
// inverts values (adapter: 1.0=open, hardware: 0=open), writes to hardware,
// then reads hardware state, inverts, and writes to its state slice.
void update_finger(DeviceHandler* handle, uint8_t slave_id,
                   SharedDDS* shared, int offset) {
    uint16_t positions[6], speeds[6];

    // Read commands from combined topic at this hand's offset
    // Invert: adapter convention 1.0=open → hardware 0=open (multiply inverted by 1000)
    for (int i = 0; i < kMotorsPerHand; ++i) {
        float cmd_val = std::clamp(shared->cmd_sub->msg_.cmds()[offset + i].q(), 0.f, 1.f);
        positions[i] = static_cast<uint16_t>((1.0f - cmd_val) * 1000.f);
        float spd_val = std::clamp(shared->cmd_sub->msg_.cmds()[offset + i].dq(), 0.f, 1.f);
        speeds[i]    = static_cast<uint16_t>(spd_val * 1000.f);
    }

    // Write commands to hardware
    modbus_set_finger_positions_and_speeds(handle, slave_id, positions, speeds, 6);

    // Read status from hardware
    auto status = modbus_get_motor_status(handle, slave_id);
    if (!status) return;

    // Update combined state at this hand's offset
    // Invert: hardware 0=open → adapter convention 1.0=open
    if (shared->state_pub->trylock()) {
        for (int i = 0; i < kMotorsPerHand; ++i) {
            shared->state_pub->msg_.states()[offset + i].q()       = 1.0f - (status->positions[i] / 1000.f);
            shared->state_pub->msg_.states()[offset + i].dq()      = status->speeds[i] / 1000.f;
            shared->state_pub->msg_.states()[offset + i].tau_est() = status->currents[i] / 1000.f;
        }
        shared->state_pub->unlockAndPublish();
    }
    free_motor_status_data(status);
}

// Worker thread for each hand
void hand_worker(DeviceHandler* handle, uint8_t slave_id,
                 SharedDDS* shared, int offset, const std::string& label) {
    spdlog::info("Starting worker for {} hand (offset {}, slave {})",
                 label, offset, (int)slave_id);

    while (running) {
        auto start_time = std::chrono::high_resolution_clock::now();
        update_finger(handle, slave_id, shared, offset);
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - start_time).count();
        int sleep_us = 10000 - static_cast<int>(elapsed_us); // 100Hz
        if (sleep_us > 0) std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
    }

    spdlog::info("Worker for {} hand exiting (closing handle)", label);
    if (handle) modbus_close(handle);
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);

    auto vm = param::helper(argc, argv);
    std::string ns = vm["namespace"].as<std::string>();
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network_interface"].as<std::string>());

    init_cfg(StarkHardwareType::STARK_HARDWARE_TYPE_REVO2_BASIC,
             StarkProtocolType::STARK_PROTOCOL_TYPE_MODBUS,
             LogLevel::LOG_LEVEL_ERROR, 1024);

    // Create shared DDS resources for combined 12-motor topic
    SharedDDS shared;
    shared.cmd_sub = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>>(
        "rt/" + ns + "/cmd");
    shared.cmd_sub->msg_.cmds().resize(kTotalMotors);
    // Initialize default speed for all fingers
    for (auto& finger : shared.cmd_sub->msg_.cmds()) finger.dq() = 1.;

    shared.state_pub = std::make_unique<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>>(
        "rt/" + ns + "/state");
    shared.state_pub->msg_.states().resize(kTotalMotors);

    spdlog::info("DDS topics: rt/{}/cmd, rt/{}/state ({} motors)", ns, ns, kTotalMotors);

    // Find hands
    std::vector<std::string> available_ports = getAvailableSerialPorts();
    if (available_ports.empty()) {
        spdlog::warn("No ttyUSB serial ports found.");
        return 0;
    }

    // Find right hand first (offset 0-5), then left (offset 6-11)
    HandConnection right_conn = find_hand(available_ports, R_id,
        {SkuType::SKU_TYPE_SMALL_RIGHT, SkuType::SKU_TYPE_MEDIUM_RIGHT}, "right");
    HandConnection left_conn  = find_hand(available_ports, L_id,
        {SkuType::SKU_TYPE_SMALL_LEFT, SkuType::SKU_TYPE_MEDIUM_LEFT}, "left");

    // Launch workers: right hand at offset 0, left hand at offset 6
    std::thread right_thread, left_thread;
    if (right_conn.handle) right_thread = std::thread(hand_worker, right_conn.handle, R_id, &shared, 0, "right");
    if (left_conn.handle)  left_thread  = std::thread(hand_worker, left_conn.handle,  L_id, &shared, kMotorsPerHand, "left");

    if (right_thread.joinable()) right_thread.join();
    if (left_thread.joinable())  left_thread.join();

    spdlog::info("exit.");
    return 0;
}
