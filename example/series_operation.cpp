/**
 * @example series_operation.cpp
 * Consecutively run a series of operations of different types using
 * various RDK APIs.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <Robot.hpp>
#include <Log.hpp>

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
const double k_loopPeiord = 0.001;

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

/** Initial joint positions */
std::vector<double> g_initJointPos;
/** Initial TCP pose */
std::vector<double> g_initTcpPose;

/** Sine counter */
unsigned int g_sineCounter = 0;

/** Index of the current executing operation from op_list */
unsigned int g_opIndex = 0;

/** Time elapsed for the current operation */
unsigned int g_opTimer = 0;
/** Loop counter for 1 second */
const unsigned int k_secToCount = 1000;

/** Flag for sending plan command only once */
bool g_isPlanSent = false;

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

/** Name list for printing */
const std::string OperationNames[OP_NUM] = {"Idle", "Stop robot", "Go home",
    "Multi-dof sine", "Sine-sweep with joint position control",
    "Sine-sweep with joint torque control",
    "Sine-sweep with Cartesian impedance control", "Finished"};

}

/** Callback function for realtime periodic task */
void periodicTask(std::shared_ptr<flexiv::RobotStates> robotStates,
    std::shared_ptr<flexiv::Robot> robot, const std::vector<Operation>& op_list,
    flexiv::Log& log)
{
    // Read robot states
    robot->getRobotStates(robotStates.get());

    // State machine on the operation list
    switch (op_list[g_opIndex]) {
        case OP_IDLE: {
            // Do nothing, transit to next operation in list
            g_opIndex++;
            // Reset operation timer
            g_opTimer = 0;
            // Print
            log.info(
                "Running operation: " + OperationNames[op_list[g_opIndex]]);
            break;
        }
        case OP_STOP_ROBOT: {
            // Command robot to stop, call this once. It's recommended to call
            // stop() when switching robot mode
            if (g_opTimer == 0) {
                robot->stop();
            }

            // Wait for a while before checking if robot has come to a
            // complete stop
            if (++g_opTimer >= 1 * k_secToCount) {
                if (robot->isStopped()) {
                    // Done, transit to next operation in list
                    g_opIndex++;
                    // Reset operation timer
                    g_opTimer = 0;
                    // Print
                    log.info("Running operation: "
                             + OperationNames[op_list[g_opIndex]]);
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
                if (!g_isPlanSent) {
                    robot->executePlanByName("PLAN-Home");
                    g_isPlanSent = true;
                }

                // Wait for a while before checking if robot has come to a
                // complete stop
                if (++g_opTimer >= 1 * k_secToCount) {
                    if (robot->isStopped()) {
                        // Done, transit to next operation in list
                        g_opIndex++;
                        // Reset operation timer
                        g_opTimer = 0;
                        // Reset flag
                        g_isPlanSent = false;
                        // Print
                        log.info("Running operation: "
                                 + OperationNames[op_list[g_opIndex]]);
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
                if (!g_isPlanSent) {
                    robot->executePlanByName("MultiDOFSineTest_A5");
                    g_isPlanSent = true;
                }

                // Wait for operation period to timeout
                if (++g_opTimer >= 20 * k_secToCount) {
                    // Done, transit to next operation in list
                    g_opIndex++;
                    // Reset operation timer
                    g_opTimer = 0;
                    // Reset flag
                    g_isPlanSent = false;
                    // Print
                    log.info("Running operation: "
                             + OperationNames[op_list[g_opIndex]]);
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
                    g_initJointPos = robotStates->m_q;
                } else {
                    log.error("Invalid size of received joint position");
                    // Stop robot and terminate program
                    robot->stop();
                    exit(1);
                }

                // Reset sine counter
                g_sineCounter = 0;
            } else {
                std::vector<double> targetPosition(k_robotDofs, 0);
                std::vector<double> targetVelocity(k_robotDofs, 0);
                std::vector<double> targetAcceleration(k_robotDofs, 0);

                for (size_t i = 0; i < k_robotDofs; ++i) {
                    targetPosition[i]
                        = g_initJointPos[i]
                          + k_sineAmp[i]
                                * sin(2 * M_PI * k_sineFreq[i] * g_sineCounter
                                      * k_loopPeiord);
                }

                // Increment sine counter
                g_sineCounter++;

                // Send command
                robot->streamJointPosition(
                    targetPosition, targetVelocity, targetAcceleration);

                // Wait for operation period to timeout
                if (++g_opTimer >= 10 * k_secToCount) {
                    // Done, transit to next operation in list
                    g_opIndex++;
                    // Reset operation timer
                    g_opTimer = 0;
                    // Print
                    log.info("Running operation: "
                             + OperationNames[op_list[g_opIndex]]);
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
                    g_initJointPos = robotStates->m_q;
                } else {
                    log.error("Invalid size of received joint position");
                    // Stop robot and terminate program
                    robot->stop();
                    exit(1);
                }
                // Reset sine counter
                g_sineCounter = 0;
            } else {
                std::vector<double> targetPosition(k_robotDofs, 0);
                for (size_t i = 0; i < k_robotDofs; ++i) {
                    targetPosition[i]
                        = g_initJointPos[i]
                          + k_sineAmp[i]
                                * sin(2 * M_PI * k_sineFreq[i] * g_sineCounter
                                      * k_loopPeiord);
                }

                // Increment sine counter
                g_sineCounter++;

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
                if (++g_opTimer >= 10 * k_secToCount) {
                    // Done, transit to next operation in list
                    g_opIndex++;
                    // Reset operation timer
                    g_opTimer = 0;
                    // Print
                    log.info("Running operation: "
                             + OperationNames[op_list[g_opIndex]]);
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
                    g_initTcpPose = robotStates->m_tcpPose;
                } else {
                    log.error("Invalid size of received TCP pose");
                    // Stop robot and terminate program
                    robot->stop();
                    exit(1);
                }
                // Reset sine counter
                g_sineCounter = 0;
            } else {
                auto targetTcpPose = g_initTcpPose;
                // Use target TCP velocity and acceleration = 0
                std::vector<double> targetTcpVel(6, 0);
                std::vector<double> targetTcpAcc(6, 0);
                // Sine-sweep X direction
                targetTcpPose[1] = g_initTcpPose[1]
                                   + k_swingAmp
                                         * sin(2 * M_PI * k_swingFreq
                                               * g_sineCounter * k_loopPeiord);

                // Increment sine counter
                g_sineCounter++;

                // Send command
                robot->streamTcpPose(targetTcpPose, targetTcpVel, targetTcpAcc);

                // Wait for operation period to timeout
                if (++g_opTimer >= 20 * k_secToCount) {
                    // Done, transit to next operation in list
                    g_opIndex++;
                    // Reset operation timer
                    g_opTimer = 0;
                    // Print
                    log.info("Running operation: "
                             + OperationNames[op_list[g_opIndex]]);
                }
            }
            break;
        }
        case OP_FINISH: {
            // Repeat the whole sequence
            g_opIndex = 0;
            break;
        }
        default:
            break;
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

    // RDK Initialization
    //=============================================================================
    // Instantiate robot interface
    auto robot = std::make_shared<flexiv::Robot>();

    // Create data struct for storing robot states
    auto robotStates = std::make_shared<flexiv::RobotStates>();

    // Initialize robot interface and connect to the robot server
    robot->init(robotIP, localIP);

    // Wait for the connection to be established
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (!robot->isConnected());

    // Enable the robot, make sure the E-stop is released before enabling
    if (robot->enable()) {
        log.info("Enabling robot ...");
    }

    // Wait for the robot to become operational
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (!robot->isOperational());
    log.info("Robot is now operational");

    // List of Operations
    //=============================================================================
    // It is recommended to call stop() when switching robot mode
    std::vector<Operation> op_list = {
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

    // High-priority Realtime Periodic Task @ 1kHz
    //=============================================================================
    // This is a blocking method, so all other user-defined background threads
    // should be spawned before this
    robot->start(std::bind(periodicTask, robotStates, robot, op_list, log));

    return 0;
}
