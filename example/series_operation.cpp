/**
 * @example series_operation.cpp
 * Consecutively run a series of operations of different types using
 * various RDK APIs.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Scheduler.hpp>

#include <string>
#include <cmath>
#include <thread>
#include <chrono>

namespace {
/** Robot DOF */
const int k_robotDofs = 7;
/** Size of Cartesian pose vector [position 3x1 + rotation (quaternion) 4x1 ] */
const unsigned int k_cartPoseSize = 7;
/** Loop period [s] */
const double k_loopPeriod = 0.001;

/** Joint impedance control gains */
const std::vector<double> k_impedanceKp
    = {3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0};
const std::vector<double> k_impedanceKd
    = {80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0};

/** Joint sine-sweep trajectory amplitude and frequency */
const std::vector<double> k_sineAmp
    = {0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035};
const std::vector<double> k_sineFreq = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

/** TCP sine-sweep amplitude [m] */
const double k_swingAmp = 0.1;
/** TCP sine-sweep frequency [Hz] */
const double k_swingFreq = 0.3;

/** Loop counter for 1 second */
const unsigned int k_secToCount = 1000;

/** Type of operations */
enum Operation
{
    OP_IDLE,
    OP_STOP_ROBOT,
    OP_GO_HOME,
    OP_MULTIDOF_SINE,
    OP_JOINT_POSITION_SINE,
    OP_JOINT_TORQUE_SINE,
    OP_CARTESIAN_SINE,
    OP_FINISH,
    OP_NUM = OP_FINISH - OP_IDLE + 1, ///< Helper
};
std::vector<Operation> operationList;

/** Name list for printing */
const std::string OperationNames[OP_NUM] = {"Idle", "Stop robot", "Go home",
    "Multi-dof sine", "Sine-sweep with joint position control",
    "Sine-sweep with joint torque control",
    "Sine-sweep with Cartesian impedance control", "Finished"};

}

/** Callback function for realtime periodic task */
void periodicTask(flexiv::Robot* robot, flexiv::RobotStates* robotStates,
    flexiv::Scheduler* scheduler, flexiv::Log* log)
{
    // Sine counter
    static unsigned int sineCounter = 0;

    // Initial Cartesian-space pose (position + rotation) of robot TCP
    static std::vector<double> initTcpPose;

    // Initial joint positions
    static std::vector<double> initJointPos;

    // Index of the current executing operation from operationList
    static unsigned int opIndex = 0;

    // Time elapsed for the current operation
    static unsigned int opTimer = 0;

    // Flag for sending plan command only once
    static bool isPlanSent = false;

    try {
        // Monitor fault on robot server
        if (robot->isFault()) {
            throw flexiv::ServerException(
                "periodicTask: Fault occurred on robot server, exiting ...");
        }

        // Read robot states
        robot->getRobotStates(robotStates);

        // State machine on the operation list
        switch (operationList[opIndex]) {
            case OP_IDLE: {
                // Do nothing, transit to next operation in list
                opIndex++;
                // Reset operation timer
                opTimer = 0;
                // Print
                log->info("Running operation: "
                          + OperationNames[operationList[opIndex]]);
                break;
            }
            case OP_STOP_ROBOT: {
                // Command robot to stop, call this once. It's recommended to
                // call stop() when switching robot mode
                if (opTimer == 0) {
                    robot->stop();
                }

                // Wait for a while before checking if robot has come to a
                // complete stop
                if (++opTimer >= 1 * k_secToCount) {
                    if (robot->isStopped()) {
                        // Done, transit to next operation in list
                        opIndex++;
                        // Reset operation timer
                        opTimer = 0;
                        // Print
                        log->info("Running operation: "
                                  + OperationNames[operationList[opIndex]]);
                    }
                }
                break;
            }
            case OP_GO_HOME: {
                // Must switch to the correct mode first
                if (robot->getMode() != flexiv::MODE_PLAN_EXECUTION) {
                    robot->setMode(flexiv::MODE_PLAN_EXECUTION);
                } else {
                    // Send plan command only once
                    if (!isPlanSent) {
                        robot->executePlanByName("PLAN-Home");
                        isPlanSent = true;
                    }

                    // Wait for a while before checking if robot has come to a
                    // complete stop
                    if (++opTimer >= 1 * k_secToCount) {
                        if (robot->isStopped()) {
                            // Done, transit to next operation in list
                            opIndex++;
                            // Reset operation timer
                            opTimer = 0;
                            // Reset flag
                            isPlanSent = false;
                            // Print
                            log->info("Running operation: "
                                      + OperationNames[operationList[opIndex]]);
                        }
                    }
                }
                break;
            }
            case OP_MULTIDOF_SINE: {
                // Must switch to the correct mode first
                if (robot->getMode() != flexiv::MODE_PLAN_EXECUTION) {
                    robot->setMode(flexiv::MODE_PLAN_EXECUTION);
                } else {
                    // Send plan command only once
                    if (!isPlanSent) {
                        robot->executePlanByName("MultiDOFSineTest_A5");
                        isPlanSent = true;
                    }

                    // Wait for operation period to timeout
                    if (++opTimer >= 20 * k_secToCount) {
                        // Done, transit to next operation in list
                        opIndex++;
                        // Reset operation timer
                        opTimer = 0;
                        // Reset flag
                        isPlanSent = false;
                        // Print
                        log->info("Running operation: "
                                  + OperationNames[operationList[opIndex]]);
                    }
                }
                break;
            }
            case OP_JOINT_POSITION_SINE: {
                // Must switch to the correct mode first
                if (robot->getMode() != flexiv::MODE_JOINT_POSITION) {
                    robot->setMode(flexiv::MODE_JOINT_POSITION);

                    // Set initial joint position
                    if (robotStates->m_q.size() == k_robotDofs) {
                        initJointPos = robotStates->m_q;
                    } else {
                        log->error("Invalid size of received joint position");
                        // Stop robot and terminate program
                        robot->stop();
                        exit(1);
                    }

                    // Reset sine counter
                    sineCounter = 0;
                } else {
                    std::vector<double> targetPosition(k_robotDofs, 0);
                    std::vector<double> targetVelocity(k_robotDofs, 0);
                    std::vector<double> targetAcceleration(k_robotDofs, 0);

                    for (size_t i = 0; i < k_robotDofs; ++i) {
                        targetPosition[i]
                            = initJointPos[i]
                              + k_sineAmp[i]
                                    * sin(2 * M_PI * k_sineFreq[i] * sineCounter
                                          * k_loopPeriod);
                    }

                    // Increment sine counter
                    sineCounter++;

                    // Send command
                    robot->streamJointPosition(
                        targetPosition, targetVelocity, targetAcceleration);

                    // Wait for operation period to timeout
                    if (++opTimer >= 10 * k_secToCount) {
                        // Done, transit to next operation in list
                        opIndex++;
                        // Reset operation timer
                        opTimer = 0;
                        // Print
                        log->info("Running operation: "
                                  + OperationNames[operationList[opIndex]]);
                    }
                }
                break;
            }
            case OP_JOINT_TORQUE_SINE: {
                // Must switch to the correct mode first
                if (robot->getMode() != flexiv::MODE_JOINT_TORQUE) {
                    robot->setMode(flexiv::MODE_JOINT_TORQUE);

                    // Set initial joint position
                    if (robotStates->m_q.size() == k_robotDofs) {
                        initJointPos = robotStates->m_q;
                    } else {
                        log->error("Invalid size of received joint position");
                        // Stop robot and terminate program
                        robot->stop();
                        exit(1);
                    }
                    // Reset sine counter
                    sineCounter = 0;
                } else {
                    std::vector<double> targetPosition(k_robotDofs, 0);
                    for (size_t i = 0; i < k_robotDofs; ++i) {
                        targetPosition[i]
                            = initJointPos[i]
                              + k_sineAmp[i]
                                    * sin(2 * M_PI * k_sineFreq[i] * sineCounter
                                          * k_loopPeriod);
                    }

                    // Increment sine counter
                    sineCounter++;

                    // Impedance control on all joints
                    std::vector<double> torqueDesired(k_robotDofs, 0);
                    for (size_t i = 0; i < k_robotDofs; ++i) {
                        torqueDesired[i]
                            = k_impedanceKp[i]
                                  * (targetPosition[i] - robotStates->m_q[i])
                              - k_impedanceKd[i] * robotStates->m_dtheta[i];
                    }

                    // Send command
                    robot->streamJointTorque(torqueDesired, true);

                    // Wait for operation period to timeout
                    if (++opTimer >= 10 * k_secToCount) {
                        // Done, transit to next operation in list
                        opIndex++;
                        // Reset operation timer
                        opTimer = 0;
                        // Print
                        log->info("Running operation: "
                                  + OperationNames[operationList[opIndex]]);
                    }
                }
                break;
            }
            case OP_CARTESIAN_SINE: {
                // Must switch to the correct mode first
                if (robot->getMode() != flexiv::MODE_CARTESIAN_IMPEDANCE) {
                    robot->setMode(flexiv::MODE_CARTESIAN_IMPEDANCE);

                    // Set initial TCP pose
                    if (robotStates->m_tcpPose.size() == k_cartPoseSize) {
                        initTcpPose = robotStates->m_tcpPose;
                    } else {
                        log->error("Invalid size of received TCP pose");
                        // Stop robot and terminate program
                        robot->stop();
                        exit(1);
                    }
                    // Reset sine counter
                    sineCounter = 0;
                } else {
                    auto targetTcpPose = initTcpPose;
                    // Use target TCP velocity and acceleration = 0
                    std::vector<double> targetTcpVel(6, 0);
                    std::vector<double> targetTcpAcc(6, 0);
                    // Sine-sweep X direction
                    targetTcpPose[1]
                        = initTcpPose[1]
                          + k_swingAmp
                                * sin(2 * M_PI * k_swingFreq * sineCounter
                                      * k_loopPeriod);

                    // Increment sine counter
                    sineCounter++;

                    // Send command
                    robot->streamTcpPose(
                        targetTcpPose, targetTcpVel, targetTcpAcc);

                    // Wait for operation period to timeout
                    if (++opTimer >= 20 * k_secToCount) {
                        // Done, transit to next operation in list
                        opIndex++;
                        // Reset operation timer
                        opTimer = 0;
                        // Print
                        log->info("Running operation: "
                                  + OperationNames[operationList[opIndex]]);
                    }
                }
                break;
            }
            case OP_FINISH: {
                // Repeat the whole sequence
                opIndex = 0;
                break;
            }
            default:
                break;
        }

    } catch (const flexiv::Exception& e) {
        log->error(e.what());
        scheduler->stop();
    }
}

int main(int argc, char* argv[])
{
    // Log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //=============================================================================
    // Check if program has 3 arguments
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

        // Create data struct for storing robot states
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

        // Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...");
        robot.enable();

        // Wait for the robot to become operational
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        log.info("Robot is now operational");

        // List of Operations
        //=============================================================================
        // It is recommended to call stop() when switching robot mode
        operationList = {
            OP_IDLE,
            OP_GO_HOME,
            OP_STOP_ROBOT,
            OP_CARTESIAN_SINE,
            OP_STOP_ROBOT,
            OP_GO_HOME,
            OP_STOP_ROBOT,
            OP_JOINT_POSITION_SINE,
            OP_STOP_ROBOT,
            OP_GO_HOME,
            OP_STOP_ROBOT,
            OP_JOINT_TORQUE_SINE,
            OP_STOP_ROBOT,
            OP_MULTIDOF_SINE,
            OP_STOP_ROBOT,
            OP_GO_HOME,
            OP_FINISH,
        };

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
