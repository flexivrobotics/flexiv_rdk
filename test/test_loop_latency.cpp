/**
 * @test test_loop_latency.cpp
 * A test to benchmark RDK's loop latency, including communication, computation, etc. The
 * workstation PC's serial port is used as reference, and the robot server's digital out port is
 * used as test target.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>

#include <iostream>
#include <thread>
#include <string.h>
#include <atomic>

// Standard input/output definitions
#include <stdio.h>
// UNIX standard function definitions
#include <unistd.h>
// File control definitions
#include <fcntl.h>
// Error number definitions
#include <errno.h>
// POSIX terminal control definitions
#include <termios.h>

namespace {
/** File descriptor of the serial port */
int g_fd = 0;

/** Atomic signal to stop scheduler tasks */
std::atomic<bool> g_stop_sched = {false};
}

// callback function for realtime periodic task
void PeriodicTask(flexiv::rdk::Robot& robot)
{
    // Loop counter
    static unsigned int loop_counter = 0;

    try {
        // Monitor fault on the connected robot
        if (robot.fault()) {
            throw std::runtime_error(
                "PeriodicTask: Fault occurred on the connected robot, exiting ...");
        }

        // send signal at 1Hz
        switch (loop_counter % 1000) {
            case 0: {
                spdlog::info(
                    "Sending benchmark signal to both workstation PC's serial "
                    "port and robot server's digital out port[0]");
                break;
            }
            case 1: {
                // signal robot server's digital out port
                robot.SetDigitalOutputs(std::vector<unsigned int> {0}, std::vector<bool> {true});

                // signal workstation PC's serial port
                auto n = write(g_fd, "0", 1);
                if (n < 0) {
                    spdlog::error("Failed to write to serial port");
                }

                break;
            }
            case 900: {
                // reset digital out after a few seconds
                robot.SetDigitalOutputs(std::vector<unsigned int> {0}, std::vector<bool> {false});
                break;
            }
            default:
                break;
        }
        loop_counter++;

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        g_stop_sched = true;
    }
}

void PrintHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot_sn] [serial_port_name]" << std::endl;
    std::cout << "    robot_sn: Serial number of the robot to connect. Remove any space, e.g. Rizon4s-123456" << std::endl;
    std::cout << "    serial_port_name: /dev/ttyS0 for COM1, /dev/ttyS1 for COM2, /dev/ttyUSB0 for USB-serial converter" << std::endl;
    std::cout << "Optional arguments: None" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

int main(int argc, char* argv[])
{
    // Parse Parameters
    //=============================================================================
    if (argc < 3 || flexiv::rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }

    // Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456
    std::string robot_sn = argv[1];

    // serial port name
    std::string serial_port = argv[2];

    try {
        // RDK Initialization
        //=============================================================================
        // Instantiate robot interface
        flexiv::rdk::Robot robot(robot_sn);

        // Clear fault on the connected robot if any
        if (robot.fault()) {
            spdlog::warn("Fault occurred on the connected robot, trying to clear ...");
            // Try to clear the fault
            if (!robot.ClearFault()) {
                spdlog::error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            spdlog::info("Fault on the connected robot is cleared");
        }

        // enable the robot, make sure the E-stop is released before enabling
        spdlog::info("Enabling robot ...");
        robot.Enable();

        // Wait for the robot to become operational
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        spdlog::info("Robot is now operational");

        // Benchmark Signal
        //=============================================================================
        // get workstation PC's serial port ready,
        g_fd = open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL | O_CLOEXEC);

        if (g_fd == -1) {
            spdlog::error("Unable to open serial port [{}]", serial_port);
        }

        // print messages
        spdlog::warn("Benchmark signal will be sent every 1 second");

        // Periodic Tasks
        //=============================================================================
        flexiv::rdk::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.AddTask(
            std::bind(PeriodicTask, std::ref(robot)), "HP periodic", 1, scheduler.max_priority());
        // Start all added tasks
        scheduler.Start();

        // Block and wait for signal to stop scheduler tasks
        while (!g_stop_sched) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        // Received signal to stop scheduler tasks
        scheduler.Stop();

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
