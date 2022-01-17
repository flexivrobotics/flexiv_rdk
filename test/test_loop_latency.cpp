/**
 * @test test_loop_latency.cpp
 * A test to benchmark RDK's loop latency, including communication, computation,
 * etc. The workstation PC's serial port is used as reference, and the robot
 * server's digital out port is used as test target.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <Robot.hpp>
#include <Log.hpp>

#include <iostream>
#include <thread>
#include <string.h>

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
// loop counter
unsigned int g_loopCounter = 0;

// file descriptor of the serial port
int g_fd = 0;
}

// callback function for realtime periodic task
void periodicTask(std::shared_ptr<flexiv::Robot> robot, flexiv::Log& log)
{
    // send signal at 1Hz
    switch (g_loopCounter % 1000) {
        case 0: {
            log.info(
                "Sending benchmark signal to both workstation PC's serial port "
                "and robot server's digital out port[0]");
            break;
        }
        case 1: {
            // signal robot server's digital out port
            robot->writeDigitalOutput(0, true);

            // signal workstation PC's serial port
            auto n = write(g_fd, "0", 1);
            if (n < 0) {
                log.error("Failed to write to serial port");
            }

            break;
        }
        case 900: {
            // reset digital out after a few seconds
            robot->writeDigitalOutput(0, false);
            break;
        }
        default:
            break;
    }

    g_loopCounter++;
}

int main(int argc, char* argv[])
{
    // log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //=============================================================================
    // check if program has 3 arguments
    if (argc != 4) {
        log.error(
            "Invalid program arguments. Usage: <robot_ip> <local_ip> "
            "<serial_port_name>");
        log.info(
            "Some examples of <serial_port_name>: /dev/ttyS0 corresponds to "
            "COM1, /dev/ttyS1 corresponds to COM2, /dev/ttyUSB0 corresponds to "
            "a USB-serial converter");
        return 0;
    }
    // IP of the robot server
    std::string robotIP = argv[1];

    // IP of the workstation PC running this program
    std::string localIP = argv[2];

    // serial port name
    std::string serialPort = argv[3];

    // RDK Initialization
    //=============================================================================
    // instantiate robot interface
    auto robot = std::make_shared<flexiv::Robot>();

    // initialize robot interface and connect to the robot server
    robot->init(robotIP, localIP);

    // wait for the connection to be established
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (!robot->isConnected());

    // enable the robot, make sure the E-stop is released before enabling
    if (robot->enable()) {
        log.info("Enabling robot ...");
    }

    // wait for the robot to become operational
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (!robot->isOperational());
    log.info("Robot is now operational");

    // Benchmark Signal
    //=============================================================================
    // get workstation PC's serial port ready,
    g_fd = open(
        serialPort.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL | O_CLOEXEC);

    if (g_fd == -1) {
        log.error("Unable to open serial port " + serialPort);
    }

    // print messages
    log.warn("Benchmark signal will be sent every 1 second");

    // High-priority Realtime Periodic Task @ 1kHz
    //=============================================================================
    // this is a blocking method, so all other user-defined background
    // threads should be spawned before this
    robot->start(std::bind(periodicTask, robot, log));

    return 0;
}
