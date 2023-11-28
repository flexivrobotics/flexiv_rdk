/**
 * @test test_loop_latency.cpp
 * A test to benchmark RDK's loop latency, including communication, computation,
 * etc. The workstation PC's serial port is used as reference, and the robot
 * server's digital out port is used as test target.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Scheduler.hpp>
#include <flexiv/Utility.hpp>

#include <iostream>
#include <thread>
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
/** file descriptor of the serial port */
int g_fd = 0;

/** Atomic signal to stop scheduler tasks */
std::atomic<bool> g_schedStop = {false};
}

// callback function for realtime periodic task
void periodicTask(flexiv::Robot& robot, flexiv::Log& log)
{
    // Loop counter
    static unsigned int loopCounter = 0;

    try {
        // Monitor fault on robot server
        if (robot.isFault()) {
            throw flexiv::ServerException(
                "periodicTask: Fault occurred on robot server, exiting ...");
        }

        // send signal at 1Hz
        switch (loopCounter % 1000) {
            case 0: {
                log.info(
                    "Sending benchmark signal to both workstation PC's serial "
                    "port and robot server's digital out port[0]");
                break;
            }
            case 1: {
                // signal robot server's digital out port
                robot.writeDigitalOutput(std::vector<unsigned int> {0}, std::vector<bool> {true});

                // signal workstation PC's serial port
                auto n = write(g_fd, "0", 1);
                if (n < 0) {
                    log.error("Failed to write to serial port");
                }

                break;
            }
            case 900: {
                // reset digital out after a few seconds
                robot.writeDigitalOutput(std::vector<unsigned int> {0}, std::vector<bool> {false});
                break;
            }
            default:
                break;
        }
        loopCounter++;

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        g_schedStop = true;
    }
}

void printHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot IP] [local IP] [serial port name]" << std::endl;
    std::cout << "    robot IP: address of the robot server" << std::endl;
    std::cout << "    local IP: address of this PC" << std::endl;
    std::cout << "    serial port name: /dev/ttyS0 for COM1, /dev/ttyS1 for "
                 "COM2, /dev/ttyUSB0 for USB-serial converter" << std::endl;
    std::cout << "Optional arguments: None" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

int main(int argc, char* argv[])
{
    // log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //=============================================================================
    if (argc < 4 || flexiv::utility::programArgsExistAny(argc, argv, {"-h", "--help"})) {
        printHelp();
        return 1;
    }

    // IP of the robot server
    std::string robotIP = argv[1];

    // IP of the workstation PC running this program
    std::string localIP = argv[2];

    // serial port name
    std::string serialPort = argv[3];

    try {
        // RDK Initialization
        //=============================================================================
        // Instantiate robot interface
        flexiv::Robot robot(robotIP, localIP);

        // Clear fault on robot server if any
        if (robot.isFault()) {
            log.warn("Fault occurred on robot server, trying to clear ...");
            // Try to clear the fault
            robot.clearFault();
            std::this_thread::sleep_for(std::chrono::seconds(2));
            // Check again
            if (robot.isFault()) {
                log.error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            log.info("Fault on robot server is cleared");
        }

        // enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...");
        robot.enable();

        // Wait for the robot to become operational
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        log.info("Robot is now operational");

        // Benchmark Signal
        //=============================================================================
        // get workstation PC's serial port ready,
        g_fd = open(serialPort.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL | O_CLOEXEC);

        if (g_fd == -1) {
            log.error("Unable to open serial port " + serialPort);
        }

        // print messages
        log.warn("Benchmark signal will be sent every 1 second");

        // Periodic Tasks
        //=============================================================================
        flexiv::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.addTask(std::bind(periodicTask, std::ref(robot), std::ref(log)), "HP periodic", 1,
            scheduler.maxPriority());
        // Start all added tasks
        scheduler.start();

        // Block and wait for signal to stop scheduler tasks
        while (!g_schedStop) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        // Received signal to stop scheduler tasks
        scheduler.stop();

        // Wait a bit for any last-second robot log message to arrive and get printed
        std::this_thread::sleep_for(std::chrono::seconds(2));

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
