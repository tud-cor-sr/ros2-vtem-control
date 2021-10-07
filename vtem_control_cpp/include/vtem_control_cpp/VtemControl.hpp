#pragma once

#include <cstdint>
#include <vector>

#include "modbus/modbus.h"

namespace vtem_control {
    /**
     * @brief low-level control for the Festo valve array
     */
    class VtemControl {
    public:
        explicit VtemControl(const char *node, const char *service, int num_valves = 16);

        ~VtemControl();

        VtemControl &operator=(const VtemControl &) = delete;

        VtemControl(const VtemControl &) = delete;

        VtemControl() = default;

        bool connect();
        bool disconnect();
        bool is_connected();

        int get_slot_idx_from_valve_idx(int valve_idx) const;

        bool get_single_motion_app(int slot_idx, int &motion_app_id, int &valve_state);
        bool get_single_motion_app_adjusted(int slot_idx);

        bool set_single_motion_app(int slot_idx, int motion_app_id = 61, int app_control = 0);
        bool set_all_motion_apps(int motion_app_id = 61, int app_control = 0);

        bool acknowledge_errors(int slot_idx = -1);

        bool activate_pressure_regulation(int slot_idx = -1);
        bool deactivate_pressure_regulation(int slot_idx = -1);

        int get_single_pressure(int valve_idx);

        void set_single_pressure(int valve_idx, int pressure = 0);

        void get_all_pressures(std::vector<int> *output);

        void set_all_pressures(const std::vector<int> &pressures);

        int num_valves_;
        int num_slots_;

    private:
        bool ensure_connection() const;

        bool ensure_motion_app(int slot_idx, int des_motion_app_id, int des_valve_state, bool throw_exception = true);

        std::vector<int16_t> input_status_buffer_;
        std::vector<int16_t> input_value_buffer_;

        modbus_t *ctx_;

        bool connected_;
        bool set_motion_app_;
    };
}