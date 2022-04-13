/**
 * @test test_timeliness_monitor.cpp
 * A test to evaluate RDK's internal timeliness monitor on real-time modes. Bad
 * communication or insufficient real-time performance of the workstation PC
 * will cause the monitor's timeliness check to fail. A warning will be issued
 * first, then if the check has failed too many times, the RDK connection with
 * the server will be closed. During this test, the robot will hold its position
 * using joint torque streaming mode.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Scheduler.hpp>

#include <iostream>
#include <string>
#include <cmath>
#include <thread>

namespace {
// robot DOF
const int k_robotDofs = 7;

// joint impedance control gains
const std::vector<double> k_impedanceKp
    = {3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0};
const std::vector<double> k_impedanceKd
    = {80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0};

}

// callback function for realtime periodic task
void periodicTask(flexiv::Robot* robot, flexiv::RobotStates* robotStates,
    flexiv::Scheduler* scheduler, flexiv::Log* log)
{
    // Loop counter
    static unsigned int loopCounter = 0;

    // Whether initial joint position is set
    static bool isInitPositionSet = false;

    // Initial position of robot joints
    static std::vector<double> initPosition;

    try {
        // Monitor fault on robot server
        if (robot->isFault()) {
            throw flexiv::ServerException(
                "periodicTask: Fault occurred on robot server, exiting ...");
        }

        // Read robot states
        robot->getRobotStates(robotStates);

        // Set initial joint position
        if (!isInitPositionSet) {
            // check vector size before saving
            if (robotStates->m_q.size() == k_robotDofs) {
                initPosition = robotStates->m_q;
                isInitPositionSet = true;
            }
        }
        // Run control after init position is set
        else {
            // initialize target position to hold position
            auto targetPosition = initPosition;

            std::vector<double> torqueDesired(k_robotDofs);
            // impedance control on all joints
            for (size_t i = 0; i < k_robotDofs; ++i) {
                torqueDesired[i]
                    = k_impedanceKp[i]
                          * (targetPosition[i] - robotStates->m_q[i])
                      - k_impedanceKd[i] * robotStates->m_dtheta[i];
            }

            // send target joint torque to RDK server
            robot->streamJointTorque(torqueDesired, true);
        }

        if (loopCounter == 5000) {
            log->warn(">>>>> Adding simulated loop delay <<<<<");
        }
        // simulate prolonged loop time after 5 seconds
        else if (loopCounter > 5000) {
            std::this_thread::sleep_for(std::chrono::microseconds(995));
        }

        loopCounter++;

    } catch (const flexiv::Exception& e) {
        log->error(e.what());
        scheduler->stop();
    }
}

int main(int argc, char* argv[])
{
    // log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //=============================================================================
    // check if program has 3 arguments
    if (argc != 3) {
        log.error("Invalid program arguments. Usage: <robot_ip> <local_ip>");
        return 0;
    }
    // IP of the robot server
    std::string robotIP = argv[1];

    // IP of the workstation PC running this program
    std::string localIP = argv[2];

    try {
        // RDK Initialization
        //=============================================================================
        // Instantiate robot interface
        flexiv::Robot robot(robotIP, localIP);

        // create data struct for storing robot states
        flexiv::RobotStates robotStates;

        // Clear fault on robot server if any
        if (robot.isFault()) {
            log.warn("Fault occurred on robot server, trying to clear ...");
            // Try to clear the fault
            robot.clearFault();
            std::this_thread::sleep_for(std::chrono::seconds(2));
            // Check again
            if (robot.isFault()) {
                log.error("Fault cannot be cleared, exiting ...");
                return 0;
            }
            log.info("Fault on robot server is cleared");
        }

        // enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...");
        robot.enable();

        // wait for the robot to become operational
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        log.info("Robot is now operational");

        // set mode after robot is operational
        robot.setMode(flexiv::MODE_JOINT_TORQUE);

        // wait for the mode to be switched
        while (robot.getMode() != flexiv::MODE_JOINT_TORQUE) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        log.warn(
            ">>>>> Simulated loop delay will be added after 5 "
            "seconds <<<<<");

        // Periodic Tasks
        //=============================================================================
        flexiv::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.addTask(
            std::bind(periodicTask, &robot, &robotStates, &scheduler, &log),
            "HP periodic", 1, 45);
        // Start all added tasks, this is by default a blocking method
        scheduler.start();

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 0;
    }

    return 0;
}
