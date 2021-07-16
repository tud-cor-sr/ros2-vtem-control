#include "modbus/modbus-tcp.h"
#include "VtemControl.hpp"

#include <cmath>
#include <stdexcept>


namespace {
    const int address_input_start = 45392;
    const int address_output_start = 40001;
    const int cpx_input_offset = 3;
    const int cpx_output_offset = 2;
    const int num_valves = 16;
}

vtem_control::VtemControl::VtemControl(const char *node, const char *service) {
    // Not connected.
    connected_ = false;

    // Resize buffer space.
    // Input buffer for each valve is of the format (actual, setpoint, diagnostic).
    input_buffer_.resize(2 * num_valves);
    output_buffer_.resize(num_valves);

    // Create Modbus context.
    ctx_ = modbus_new_tcp_pi(node, service);
}

vtem_control::VtemControl::~VtemControl() {
    // Close if not yet closed.
    if (connected_) {
        disconnect();
    }

    // Deallocate Modbus context.
    modbus_free(ctx_);
}

bool vtem_control::VtemControl::connect() {
    if (modbus_connect(ctx_) == -1) {
        fprintf(stderr, "Connection failed");//: %s\n", modbus_strerror(errno));
        modbus_free(ctx_);
        connected_ = false;
        return false;
    }
    connected_ = true;
    return true;
}

bool vtem_control::VtemControl::disconnect() {
    if (connected_) {
        modbus_close(ctx_);
        connected_ = false;
        return true;
    }

    return false;
}

int vtem_control::VtemControl::get_single_motion_app(int slot_idx) {
    const auto addr = address_input_start + cpx_input_offset + 3*slot_idx;
    uint16_t slot_status_bytes[2]; // this should read two bytes containing the slot status information
    
    if (modbus_read_registers(ctx_, addr, 1, &slot_status_bytes[0]) == -1) {
        throw std::runtime_error("Failed to read slot status register.");
    }

    /* Attention: I am not sure about this implementation */
    // extract first 6 bits from first byte
    bool bit[6];
    int motion_app_id;
    for(int i = 0; i < 6; i++) {
        bit[i] = ((slot_status_bytes[0] >> i) & 0x01);
        motion_app_id = (motion_app_id << 1) | (bit[i] - '0');
    }

    return motion_app_id;
}

bool vtem_control::VtemControl::set_single_motion_app(int slot_idx, int motion_app_id = 61) {
    const auto addr = address_output_start + cpx_output_offset + 3*slot_idx;

    // TODO: Adjust this code to only write the motion app id to the first 6 bits

    if (modbus_write_register(ctx_, addr, motion_app_id) == -1) {
        throw std::runtime_error("Failed to write register for setting motion app.");
    }

    return true;
}

bool vtem_control::VtemControl::set_all_motion_apps(int motion_app_id = 61) {
    for (auto slot_idx = 0; slot_idx < (num_valves / 2); slot_idx++) {
        if (!set_single_motion_app(slot_idx, motion_app_id)) {
            return false;
        }
    }
    return true;
}

bool vtem_control::VtemControl::activate_pressure_regulation(int slot_idx = -1) {
    if (slot_idx == -1) {
        return set_all_motion_apps(3);
    } else {
        return set_single_motion_app(3);
    }
}

bool vtem_control::VtemControl::deactivate_pressure_regulation(int slot_idx = -1) {
    if (slot_idx == -1) {
        return set_all_motion_apps(61);
    } else {
        return set_single_motion_app(61);
    }
}

void vtem_control::VtemControl::ensure_connection() const {
    if (!connected_) {
        throw std::runtime_error("Operation requires a connection.");
    }
}

void vtem_control::VtemControl::ensure_motion_app(int slot_idx, int des_motion_app_id) {
    if (get_single_motion_app(slot_idx) != des_motion_app_id) {
        throw std::runtime_error("Operation requires activating the suitable motion app");
    }
}

int vtem_control::VtemControl::get_slot_idx(const int valve_idx) const {
    int slot_idx = std::floor(valve_idx / 2); // index of slot [0-7]
    return slot_idx;
}

int vtem_control::VtemControl::get_single_pressure(const int valve_idx) {
    ensure_connection();

    int slot_idx = get_slot_idx(valve_idx);
    int slot_remain = valve_idx - 2*slot_idx; // either 0 or 1 for valve in slot
    ensure_motion_app(valve_idx, 3);

    const auto dest = &input_buffer_[valve_idx];
    const auto addr = address_input_start + cpx_input_offset + 3*slot_idx + 1 + slot_remain;

    if (modbus_read_registers(ctx_, addr, 1, dest) == -1) {
        throw std::runtime_error("Failed to read VPPM register.");
    }

    return *dest;
}

void vtem_control::VtemControl::set_single_pressure(const int valve_idx, const int pressure = 0) {
    ensure_connection();

    int slot_idx = get_slot_idx(valve_idx);
    int slot_remain = valve_idx - 2*slot_idx; // either 0 or 1 for valve in slot
    ensure_motion_app(valve_idx, 3);

    const auto addr = address_output_start + cpx_output_offset + 3*slot_idx + 1 +  slot_remain;

    if (modbus_write_register(ctx_, addr, pressure) == -1) {
        throw std::runtime_error("Failed to write VPPM register.");
    }
}

void vtem_control::VtemControl::get_all_pressures(std::vector<int> *output) {
    ensure_connection();

    for (auto valve_idx = 0; valve_idx < num_valves; valve_idx++) {
        get_single_pressure(valve_idx);
        output->at(valve_idx) = input_buffer_[valve_idx];
    }
}

void vtem_control::VtemControl::set_all_pressures(const std::vector<int> &pressures) {
    ensure_connection();

    for (auto valve_idx = 0; valve_idx < num_valves; valve_idx++) {
        set_single_pressure(valve_idx, pressures[valve_idx]);
    }
}