/**
 * @example Move robot TCP with keyboard
 * @copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <config.h>
#include <Robot.hpp>

#include <iostream>
#include <thread>
#include <cmath>
#include <mutex>

#include <linux/input.h>
#include <fcntl.h> // open
#include <unistd.h> // read, close

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <RBDyn/IK.h>
#include <RBDyn/parsers/urdf.h>
#include <SpaceVecAlg/Conversions.h>

namespace {
// robot DOF
const unsigned int k_robotDofs = 7;

// size of Cartesian pose vector [position 3x1 + rotation (quaternion) 4x1 ]
const unsigned int k_cartPoseSize = 7;

// flag whether robot is at home position
bool g_isInitalized = false;

// flag whether robot is finish going to home position
bool g_isFinished = false;

// current/target Cartesian-space pose (position + rotation) of robot TCP
std::vector<double> g_currentTcpPose(k_robotDofs);

// home joint position for ik initial value
std::vector<double> g_homeJointPos(k_robotDofs);

// last joint position for ik initial value
std::vector<double> g_lastJointPos(k_robotDofs);

// flag whether user want to exit teleop
bool g_isExit = false;

// user input string
char g_userInput;

// user input mutex
std::mutex g_userInputMutex;

// RBDyn configuration
rbd::parsers::ParserResult g_rbdynURDF;

// index of joint which is not fixed
std::vector<size_t> g_rbdIndices;

// end effecter index
int g_eeIndex;
}

void printJointInDeg(const std::vector<double>& vec)
{
    for (auto i : vec) {
        std::cout << i * 180 / M_PI << "  ";
    }
    std::cout << std::endl;
}

// get the index of body in RBDyn
int getRbdIndex(const std::string& linkName)
{
    for (size_t i = 0; i < g_rbdynURDF.mb.nrBodies(); ++i)
        if (g_rbdynURDF.mb.body(i).name() == linkName)
            return i;
    return 0;
}

// initialize RBDyn indices
void initRBDyn(const std::string& urdfFilePath, const std::string& targetName)
{
    g_rbdynURDF = rbd::parsers::from_urdf_file(urdfFilePath);

    g_rbdIndices.clear();

    for (size_t i = 0; i < g_rbdynURDF.mb.nrJoints(); ++i) {
        if (g_rbdynURDF.mb.joint(i).type() != rbd::Joint::Fixed)
            g_rbdIndices.push_back(i);
    }

    g_eeIndex = getRbdIndex(targetName);
}

// set targeted flange pose from keyboard reading
void setTargetPose(const char& input)
{
    double speed = 0.0032; //[m]; 1mm per press
    // read key at 32Hz, control at 1Hz --> press once, send 30 times
    double delta = speed / 32;
    if (input == 'w') {
        g_currentTcpPose[0] += delta;
    } else if (input == 's') {
        g_currentTcpPose[0] -= delta;
    } else if (input == 'a') {
        g_currentTcpPose[1] += delta;
    } else if (input == 'd') {
        g_currentTcpPose[1] -= delta;
    } else if (input == 'q') {
        g_currentTcpPose[2] += delta;
    } else if (input == 'e') {
        g_currentTcpPose[2] -= delta;
    } else {
        // do nothing;
    }
}

// vector[XYZwxyz] to sva pose
sva::PTransformd vectorToSVATransform(const std::vector<double>& q)
{
    Eigen::Vector3d trans; // x,y,z
    trans << q[0], q[1], q[2];
    Eigen::Quaterniond rot(q[3], q[4], q[5], q[6]); // w,x,y,z
    sva::PTransformd targetPose(rot, trans);

    return targetPose;
}

// wrap angle in [-pi,pi]
double wrapAngle(const double& angle)
{
    double wrappedAngle;
    if ((angle <= M_PI) && (angle >= -M_PI)) {
        wrappedAngle = angle;
    } else if (angle < 0.0) {
        wrappedAngle = std::fmod(angle - M_PI, 2.0 * M_PI) + M_PI;
    } else {
        wrappedAngle = std::fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
    }
    return wrappedAngle;
}

// solve IK with RBDyn
std::vector<double> solveIK(std::shared_ptr<rbd::InverseKinematics> ikSolver,
    const std::vector<double>& targetPose,
    const std::vector<double>& lastJointPosition)
{
    sva::PTransformd eeTargetPose = vectorToSVATransform(targetPose);

    // read limits from urdf
    Eigen::VectorXd q_low = Eigen::VectorXd::Ones(g_rbdIndices.size());
    Eigen::VectorXd q_high = q_low;

    for (size_t i = 0; i < g_rbdIndices.size(); ++i) {
        size_t index = g_rbdIndices[i];
        q_low(i)
            = g_rbdynURDF.limits.lower[g_rbdynURDF.mb.joint(index).name()][0]
              + 3 / 180 * M_PI;
        q_high(i)
            = g_rbdynURDF.limits.upper[g_rbdynURDF.mb.joint(index).name()][0]
              - 3 / 180 * M_PI;
    }

    // set IK initial joint value
    for (size_t i = 0; i < g_rbdIndices.size(); ++i) {
        size_t rbd_index = g_rbdIndices[i];
        g_rbdynURDF.mbc.q[rbd_index][0] = lastJointPosition[i];
    }

    // solve IK
    bool ikResult = ikSolver->inverseKinematics(
        g_rbdynURDF.mb, g_rbdynURDF.mbc, eeTargetPose);

    // determine whether IK solution is valid
    std::vector<double> jointResult(k_robotDofs, 0);
    if (ikResult) {
        for (size_t i = 0; i < k_robotDofs; ++i) {
            size_t index = g_rbdIndices[i];
            jointResult[i] = wrapAngle(g_rbdynURDF.mbc.q[index][0]);

            // add limits
            if (jointResult[i] < q_low(i))
                jointResult[i] = q_low(i);
            if (jointResult[i] > q_high(i))
                jointResult[i] = q_high(i);
        }
    } else {
        std::cout << "IK NOT SOLVED !! Robot NOT Moving Anymore" << std::endl;
        for (size_t i = 0; i < k_robotDofs; ++i) {
            jointResult[i] = wrapAngle(lastJointPosition[i]);
        }
    }

    return jointResult;
}

void highPriorityPeriodicTask(std::shared_ptr<flexiv::Robot> robot,
    std::shared_ptr<flexiv::RobotStates> robotStates,
    std::shared_ptr<rbd::InverseKinematics> ikSolver)
{
    robot->getRobotStates(robotStates.get());

    // get robot home info
    if (!g_isInitalized) {
        flexiv::SystemStatus systemStatus;
        robot->getSystemStatus(&systemStatus);

        if (robotStates->m_flangePose.size() == k_cartPoseSize) {
            g_currentTcpPose = robotStates->m_flangePose;
            g_lastJointPos = robotStates->m_linkPosition;
        }
        std::cout << "Initial joint position of robot (in degree) :"
                  << std::endl;
        printJointInDeg(g_lastJointPos);

        // teleop info print in terminal
        std::cout << std::endl;
        std::cout << "Robot is ready! teleoperate the robot with keyboard: (in "
                     "World Frame)"
                  << std::endl;
        std::cout << "[w] - move forward  - [+X]" << std::endl;
        std::cout << "[s] - move backward - [-X]" << std::endl;
        std::cout << "[a] - move left     - [+Y]" << std::endl;
        std::cout << "[d] - move right    - [-Y]" << std::endl;
        std::cout << "[q] - move up       - [+Z]" << std::endl;
        std::cout << "[e] - move down     - [-Z]" << std::endl;
        std::cout << "[Esc] - exit" << std::endl;

        // set mode to joint streaming
        robot->setMode(flexiv::MODE_JOINT_POSITION);
        g_isInitalized = true;
    }
    // set target, solve ik and stream joint to robot
    else {
        // send target
        if (!g_isExit) {
            char inputBuffer = ' ';
            {
                std::lock_guard<std::mutex> lock(g_userInputMutex);
                inputBuffer = g_userInput;
            }

            // set targetPose
            setTargetPose(inputBuffer);
            // solve ik
            auto targetPosition
                = solveIK(ikSolver, g_currentTcpPose, g_lastJointPos);

            // send joint stream command
            std::vector<double> targetVelocity(k_robotDofs, 0);
            std::vector<double> targetAcceleration(k_robotDofs, 0);
            std::fill(targetVelocity.begin(), targetVelocity.end(), 0.5);
            std::fill(targetAcceleration.begin(), targetAcceleration.end(), 0);
            robot->streamJointPosition(
                targetPosition, targetVelocity, targetAcceleration);

            // update last joint position for IK initial value
            g_lastJointPos = robotStates->m_linkPosition;
        }
        // handle exit
        else {
            robot->setMode(flexiv::MODE_PLAN_EXECUTION);
            // wait until mode changed
            if (robot->getMode() != flexiv::MODE_PLAN_EXECUTION) {
                return;
            }

            // if robot hasn't been home
            if (!g_isFinished) {
                robot->executePlanByName("PLAN-Home");

                std::cout << "wait robot to home" << std::endl;
                flexiv::SystemStatus systemStatus;
                // wait until execution finished
                do {
                    robot->getSystemStatus(&systemStatus);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                } while (systemStatus.m_programRunning == true);

                g_isFinished = true;
                std::cout << "exit with ctrl-C" << std::endl;
            } else
                return;
        }
    }
}

// read keyboard command through input_event
char readKeyboard(int keysFd, input_event& t)
{
    char inputBuffer;
    if (read(keysFd, &t, sizeof(t))) {
        if (t.code == KEY_W) {
            if (t.value == 1 || t.value == 2) {
                inputBuffer = 'w';
            } else if (t.value == 0) {
                inputBuffer = ' ';
            }
        } else if (t.code == KEY_S) {
            if (t.value == 1 || t.value == 2) {
                inputBuffer = 's';
            } else if (t.value == 0) {
                inputBuffer = ' ';
            }
        } else if (t.code == KEY_A) {
            if (t.value == 1 || t.value == 2) {
                inputBuffer = 'a';
            } else if (t.value == 0) {
                inputBuffer = ' ';
            }
        } else if (t.code == KEY_D) {
            if (t.value == 1 || t.value == 2) {
                inputBuffer = 'd';
            } else if (t.value == 0) {
                inputBuffer = ' ';
            }
        } else if (t.code == KEY_Q) {
            if (t.value == 1 || t.value == 2) {
                inputBuffer = 'q';
            } else if (t.value == 0) {
                inputBuffer = ' ';
            }
        } else if (t.code == KEY_E) {
            if (t.value == 1 || t.value == 2) {
                inputBuffer = 'e';
            } else if (t.value == 0) {
                inputBuffer = ' ';
            }
        } else if (t.code == KEY_ESC) {
            g_isExit = true;
        }
    }
    return inputBuffer;
}

// keyboard reading task
void readKeyboardTask()
{
    // keyboard device configuration
    std::string keyboardDevice = "/dev/input/event3";
    int keysFd;
    struct input_event t;
    keysFd = open(keyboardDevice.c_str(), O_RDONLY);
    if (keysFd <= 0) {
        std::cout << "can't open keyboard device!" << std::endl;
        exit(-1);
    }
    std::cout
        << "open keyboard device successfully, enter keyboard reading loop"
        << std::endl;

    // user input polling
    char inputBuffer = ' ';
    while (true) {
        if (g_isExit)
            break;
        inputBuffer = readKeyboard(keysFd, t);
        {
            std::lock_guard<std::mutex> lock(g_userInputMutex);
            g_userInput = inputBuffer;
        }
    }

    close(keysFd);
    std::cout << "end keyboard" << std::endl;
}

int main(int argc, char* argv[])
{
    // Parse Parameters
    //=============================================================================
    if (argc != 2) {
        std::cerr << "Invalid program arguments. Usage: <urdf_file_path>"
                  << std::endl;
        return 0;
    }

    // path for urdf file to parse
    std::string urdfFilePath = argv[1];

    // RBDyn Initialization
    //=============================================================================
    // parse urdf and set which link to move
    initRBDyn(urdfFilePath, "flange");

    // set rbdyn IK classes
    auto ikSolver
        = std::make_shared<rbd::InverseKinematics>(g_rbdynURDF.mb, g_eeIndex);

    // RDK Initialization
    //=============================================================================
    // RDK robot interface
    auto robot = std::make_shared<flexiv::Robot>();

    // robot states data from RDK server
    auto robotStates = std::make_shared<flexiv::RobotStates>();

    // initialize connection
    robot->init(ROBOT_IP, LOCAL_IP);

    // wait for the connection to be established
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (!robot->isConnected());

    // enable the robot, make sure the E-stop is released before enabling
    if (robot->enable()) {
        std::cout << "Enabling robot ..." << std::endl;
    }

    // wait for the robot to become operational
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (!robot->isOperational());
    std::cout << "Robot is now operational" << std::endl;

    // set mode after robot is operational
    robot->setMode(flexiv::MODE_PLAN_EXECUTION);

    // wait for mode to be set
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (robot->getMode() != flexiv::MODE_PLAN_EXECUTION);

    // Bring Robot To Home
    //=============================================================================
    robot->executePlanByName("PLAN-Home");

    flexiv::SystemStatus systemStatus;
    // wait execution begin
    do {
        robot->getSystemStatus(&systemStatus);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (systemStatus.m_programRunning == false);

    // wait execution finished
    do {
        robot->getSystemStatus(&systemStatus);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (systemStatus.m_programRunning == true);

    // put mode back to IDLE
    robot->setMode(flexiv::MODE_IDLE);
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (robot->getMode() != flexiv::MODE_IDLE);

    // Low-priority Background Tasks
    //=============================================================================
    // use std::thread for some non-realtime tasks, not joining (non-blocking)
    std::thread lowPriorityThread(readKeyboardTask);

    // High-priority Realtime Periodic Task @ 1kHz
    //=============================================================================
    // this is a blocking method, so all other user-defined background threads
    // should be spawned before this
    robot->start(
        std::bind(highPriorityPeriodicTask, robot, robotStates, ikSolver));

    return 0;
}