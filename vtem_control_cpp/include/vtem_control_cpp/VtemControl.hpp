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
        explicit VtemControl(const char *node, const char *service);

        ~VtemControl();

        VtemControl &operator=(const VtemControl &) = delete;

        VtemControl(const VtemControl &) = delete;

        VtemControl() = default;

        bool connect();

        bool disconnect();

        int get_single_pressure(int valve_idx);

        void set_single_pressure(int valve_idx, int pressure);

        void get_all_pressures(std::vector<int> *output);

        void set_all_pressures(const std::vector<int> &pressures);

    private:
        void ensure_connection() const;

        void ensure_motion_app() const;

        int get_slot_idx(int valve_idx) const;

        std::vector<uint16_t> input_buffer_;
        std::vector<uint16_t> output_buffer_;

        modbus_t *ctx_;

        bool connected_;
        bool set_motion_app_;
    };
}