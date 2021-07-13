#include "VtemControl.hpp"
#include <chrono>
#include <iostream>
#include <thread>

#define STEP_TEST false // send step signals to valve

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
    vtem_control::VtemControl vtemControl("192.168.1.101", "502");
    unsigned int valveNum = 0;          // test valves 0 to 15
    unsigned int commandPressure = 450; // tested 0 to 1000 mbar
    unsigned int endPressure = 0;

    // Connect.
    if (!vtemControl.connect()) {
        std::cout << "Failed to connect to VTEM." << std::endl;
        return -1;
    }

    // Set valve 0 to 1 bar.
    vtemControl.set_single_pressure(valveNum, commandPressure);
    int sensorvalue;
    double output;

    int cycles = 1000;
    std::vector<double> x(cycles), pressures(cycles),
            commandpressures(cycles); // for logging pressure profile
    for (int i = 0; i < cycles; i++) {
        // Wait 1 ms.
        // std::this_thread::sleep_for(std::chrono::millisecofalsends(1));

        if (STEP_TEST) {
            commandPressure = step_func(i);
        }
        commandpressures.at(i) = commandPressure;
        // Read pressure of valve 0.
        sensorvalue = vtemControl.get_single_pressure(valveNum);
        x.at(i) = i;
        pressures.at(i) = sensorvalue;
        output = commandPressure;

        vtemControl.set_single_pressure(valveNum, commandPressure);

        if (i % 50 == 0) {
            std::cout << "Valve " << valveNum << " sensor: " << sensorvalue
                      << " mbar\toutput: " << output << " mbar" << std::endl;
        }
    }

    // Set valve 0 to 0 bar (off).
    vtemControl.set_single_pressure(valveNum, endPressure);

    // Wait 100 ms.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Read pressure of valve 0.
    std::cout << "Valve " << valveNum << " :" << vtemControl.get_single_pressure(valveNum)
              << " mbar" << std::endl;

    // Disconnect.
    vtemControl.disconnect();

    return 0;
}