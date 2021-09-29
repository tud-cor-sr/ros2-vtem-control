#include "VtemControl.hpp"
#include <chrono>
#include <iostream>
#include <thread>

#define STEP_TEST true // send step signals to valve

/*
   Test program for actuating a single valve.
 */

int step_func(int i) {
    // step function to send to valve
    if (i % 1500 < 750) {
        return 300;
    } else {
        return 450;
    }
}

int main() {
    // Create VtemControl controller.
    vtem_control::VtemControl vtemControl("192.168.4.3", "502");

    unsigned int valveIdx = 0;          // test valves 0 to 15
    unsigned int slotIdx = vtemControl.get_slot_idx_from_valve_idx(valveIdx); 
    
    unsigned int commandedPressure = 100; // tested 0 to 450 mbar

    // Connect.
    if (!vtemControl.connect()) {
        std::cout << "Failed to connect to VTEM." << std::endl;
        return -1;
    }

    // acknowledge errors for all slots
    vtemControl.acknowledge_errors(-1);

    // Set motion app for all valves to 03 (proportional pressure regulation)
    if (!vtemControl.activate_pressure_regulation(slotIdx)) {
        std::cout << "Failed to activate pressure regulation." << std::endl;
        return -1;
    }

    vtemControl.set_single_pressure(valveIdx, commandedPressure);
    int actualPressure;

    int cycles = 1000;
    std::vector<double> x(cycles), actualPressures(cycles),
            commandedPressures(cycles); // for logging pressure profile
    for (int i = 0; i < cycles; i++) {
        // Wait 10 ms.
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (STEP_TEST) {
            commandedPressure = step_func(i);
        }
        commandedPressures.at(i) = commandedPressure;
        // Read pressure of valve 0.
        actualPressure = vtemControl.get_single_pressure(valveIdx);
        x.at(i) = i;
        actualPressures.at(i) = actualPressure;

        vtemControl.set_single_pressure(valveIdx, commandedPressure);

        if (i % 50 == 0) {
            std::cout << "Valve " << valveIdx << " actual pressure: " << actualPressure
                      << " mBar\tcommanded pressure: " << commandedPressure << " mBar" << std::endl;
        }
    }

    if (!vtemControl.deactivate_pressure_regulation(slotIdx)) {
        std::cout << "Failed to deactivate pressure regulation." << std::endl;
        return -1;
    }

    // Disconnect.
    vtemControl.disconnect();

    return 0;
}