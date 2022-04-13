/**
 * @example visualization.cpp
 * Run the RDK's integrated visualizer with the robot TCP doing sine-sweep
 * movement.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Visualization.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Scheduler.hpp>

#include <mutex>
#include <cmath>
#include <thread>

namespace {
/** Size of Cartesian pose vector [position 3x1 + rotation (quaternion) 4x1 ] */
const unsigned int k_cartPoseSize = 7;

/** RT loop period [sec] */
const double k_loopPeriod = 0.001;

/** TCP sine-sweep amplitude [m] */
const double k_swingAmp = 0.1;

/** TCP sine-sweep frequency [Hz] */
const double k_swingFreq = 0.3;

/** Data shared between threads */
struct SharedData
{
    std::vector<double> jointPositions;
} g_data;

/** Mutex on shared data */
std::mutex g_mutex;

}

/** User-defined high-priority periodic task @ 1kHz */
void highPriorityTask(flexiv::Robot* robot, flexiv::RobotStates* robotStates,
    flexiv::Scheduler* scheduler, flexiv::Log* log)
{
    // Sine counter
    static unsigned int sineCounter = 0;

    // Initial Cartesian-space pose (position + rotation) of robot TCP
    static std::vector<double> initTcpPose;

    // Flag whether initial Cartesian position is set
    static bool isInitPoseSet = false;

    try {
        // Monitor fault on robot server
        if (robot->isFault()) {
            throw flexiv::ServerException(
                "highPriorityTask: Fault occurred on robot server, exiting "
                "...");
        }

        // Read robot states
        robot->getRobotStates(robotStates);

        // Use target TCP velocity and acceleration = 0
        std::vector<double> tcpVel(6, 0);
        std::vector<double> tcpAcc(6, 0);

        // Set initial TCP pose
        if (!isInitPoseSet) {
            // Check vector size before saving
            if (robotStates->m_tcpPose.size() == k_cartPoseSize) {
                initTcpPose = robotStates->m_tcpPose;
                isInitPoseSet = true;
            }
        }
        // Run control only after initial pose is set
        else {
            auto targetTcpPose = initTcpPose;
            targetTcpPose[1] = initTcpPose[1]
                               + k_swingAmp
                                     * sin(2 * M_PI * k_swingFreq * sineCounter
                                           * k_loopPeriod);
            robot->streamTcpPose(targetTcpPose, tcpVel, tcpAcc);
            sineCounter++;
        }

        // Safely write shared data
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            g_data.jointPositions = robotStates->m_q;
        }

    } catch (const flexiv::Exception& e) {
        log->error(e.what());
        scheduler->stop();
    }
}

/** Low-priority periodic visualizer task */
void visualizerTask(flexiv::Visualization* visualizer)
{
    static unsigned int sineCounter = 0;
    sineCounter++;

    // Safely read shared data
    std::vector<double> jointPositions;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        jointPositions = g_data.jointPositions;
    }

    // Update visualized robot with new joint positions
    visualizer->updateRobot(jointPositions);

    // Update visualized object with new position and orientation, use a sine
    // function for demo
    constexpr double k_sineFreq = 0.1;
    constexpr double k_t = 0.02;
    double sineOutput = sin(2 * M_PI * k_sineFreq * sineCounter * k_t);
    Eigen::Vector3d newPosition(1, sineOutput, 0);

    // Use Euler angles to generate rotation matrix
    Eigen::Vector3d newEuler(
        M_PI * sineOutput, M_PI * sineOutput, M_PI * sineOutput);
    Eigen::AngleAxisd yaw(newEuler[2], Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitch(newEuler[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd roll(newEuler[0], Eigen::Vector3d::UnitX());
    Eigen::Matrix3d newOrientation
        = Eigen::Quaterniond(yaw * pitch * roll).toRotationMatrix();

    // Set new position and orientation for all existing objects
    visualizer->updateScene("box1", -newPosition, newOrientation);
    visualizer->updateScene("mesh1", newPosition, newOrientation);
}

int main(int argc, char* argv[])
{
    // Log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //=============================================================================
    // Check if program has 3 arguments
    if (argc != 5) {
        log.error(
            "Invalid program arguments. Usage: <robot_ip> <local_ip> "
            "<robot_urdf> <scene_urdf>");
        return 0;
    }
    // IP of the robot server
    std::string robotIP = argv[1];

    // IP of the workstation PC running this program
    std::string localIP = argv[2];

    // Path to URDFs used by visualizer
    std::string robotURDF = argv[3];
    std::string sceneURDF = argv[4];

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

        // Set mode after robot is operational
        robot.setMode(flexiv::MODE_CARTESIAN_IMPEDANCE);

        // Wait for the mode to be switched
        while (robot.getMode() != flexiv::MODE_CARTESIAN_IMPEDANCE) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Visualizer Initialization
        //=============================================================================
        log.info(
            "Initializing visualizer, make sure meshcat-server is started");
        flexiv::Visualization visualizer(robotURDF, sceneURDF);

        // Periodic Tasks
        //=============================================================================
        flexiv::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.addTask(
            std::bind(highPriorityTask, &robot, &robotStates, &scheduler, &log),
            "HP periodic", 1, 45);
        // Add periodic task with 20ms interval and low priority
        scheduler.addTask(
            std::bind(visualizerTask, &visualizer), "LP periodic", 20, 0);
        // Start all added tasks, this is by default a blocking method
        scheduler.start();

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 0;
    }

    return 0;
}
