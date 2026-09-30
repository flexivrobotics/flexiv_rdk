/**
 * @example intermediate6_realtime_cartesian_motion_force_control.cpp
 * This tutorial runs real-time Cartesian-space unified motion-force control. The Z axis of the
 * chosen reference frame will be activated for explicit force control, while the rest axes in the
 * same reference frame will stay motion controlled.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>

#include <iostream>
#include <cmath>
#include <thread>
#include <atomic>

using namespace flexiv;

namespace {
/** RT loop period [sec] */
constexpr double kLoopPeriod = 0.001;

/** TCP sine-sweep amplitude [m] */
constexpr double kSwingAmp = 0.1;

/** TCP sine-sweep frequency [Hz] */
constexpr double kSwingFreq = 0.3;

/** Pressing force to apply during the unified motion-force control [N] */
constexpr double kPressingForce = 5.0;

/** Cartesian linear velocity used to search for contact [m/s] */
constexpr double kSearchVelocity = 0.02;

/** Maximum distance to travel when searching for contact [m] */
constexpr double kSearchDistance = 1.0;

/** Maximum contact wrench during contact search for soft contact */
const std::array<double, rdk::kCartDoF> kMaxWrenchForContactSearch
    = {10.0, 10.0, 10.0, 3.0, 3.0, 3.0};

/** Atomic signal to stop scheduler tasks */
std::atomic<bool> g_stop_sched = {false};
}

/** @brief Print program usage help */
void PrintHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot_sn]" << std::endl;
    std::cout << "    robot_sn: Serial number of the robot to connect. Remove any space, e.g. Rizon4s-123456" << std::endl;
    std::cout << "Optional arguments: [--TCP] [--polish]" << std::endl;
    std::cout << "    --TCP: use TCP frame as reference frame for force control, otherwise use world frame" << std::endl;
    std::cout << "    --polish: run a simple polish motion along XY plane in world frame, otherwise hold robot motion in non-force-control axes"
              << std::endl
              << std::endl;
    // clang-format on
}

/** Callback function for realtime periodic task */
void PeriodicTask(rdk::Robot& robot, const std::array<double, rdk::kPoseSize>& init_pose,
    rdk::CoordType force_ctrl_frame, bool enable_polish)
{
    // Local periodic loop counter
    static uint64_t loop_counter = 0;

    try {
        // Monitor fault on the connected robot
        if (robot.fault()) {
            throw std::runtime_error(
                "PeriodicTask: Fault occurred on the connected robot, exiting ...");
        }

        // Initialize target pose to initial pose
        auto target_pose = init_pose;

        // Set Fz according to reference frame to achieve a "pressing down" behavior
        double Fz = 0.0;
        if (force_ctrl_frame == rdk::CoordType::WORLD) {
            Fz = kPressingForce;
        } else if (force_ctrl_frame == rdk::CoordType::TCP) {
            Fz = -kPressingForce;
        }
        std::array<double, rdk::kCartDoF> target_wrench = {0.0, 0.0, Fz, 0.0, 0.0, 0.0};

        // Apply constant force along Z axis of chosen reference frame, and do a simple polish
        // motion along XY plane in robot world frame
        if (enable_polish) {
            // Create motion command to sine-sweep along Y direction
            target_pose[1] = init_pose[1]
                             + kSwingAmp * sin(2 * M_PI * kSwingFreq * loop_counter * kLoopPeriod);
            // Command target pose and target wrench
            robot.StreamCartesianMotionForce(target_pose, target_wrench);
        }
        // Apply constant force along Z axis of chosen reference frame, and hold motions in all
        // other axes
        else {
            // Command initial pose and target wrench
            robot.StreamCartesianMotionForce(init_pose, target_wrench);
        }

        // Increment loop counter
        loop_counter++;

    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        g_stop_sched = true;
    }
}

int main(int argc, char* argv[])
{
    // Program Setup
    // =============================================================================================
    // Parse parameters
    if (argc < 2 || rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }
    // Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456
    std::string robot_sn = argv[1];

    // Print description
    std::cout << ">>> Tutorial description <<<\nThis tutorial runs real-time Cartesian-space "
                 "unified motion-force control. The Z axis of the chosen reference frame will be "
                 "activated for explicit force control, while the rest axes in the same reference "
                 "frame will stay motion controlled.\n"
              << std::endl;

    // The reference frame used for force control, see Robot::SetForceControlFrame()
    auto force_ctrl_frame = rdk::CoordType::WORLD;
    if (rdk::utility::ProgramArgsExist(argc, argv, "--TCP")) {
        std::cout << "Reference frame used for force control: robot TCP frame" << std::endl;
        force_ctrl_frame = rdk::CoordType::TCP;
    } else {
        std::cout << "Reference frame used for force control: robot world frame" << std::endl;
    }

    // Whether to enable polish motion
    bool enable_polish = false;
    if (rdk::utility::ProgramArgsExist(argc, argv, "--polish")) {
        std::cout << "Robot will run a polish motion along XY plane in robot world frame"
                  << std::endl;
        enable_polish = true;
    } else {
        std::cout << "Robot will hold its motion in all non-force-controlled axes" << std::endl;
    }

    try {
        // RDK Initialization
        // =========================================================================================
        // Instantiate robot interface
        rdk::Robot robot(robot_sn);

        // Clear fault on the connected robot if any
        if (robot.fault()) {
            std::cerr << "[warn] Fault occurred on the connected robot, trying to clear ..."
                      << std::endl;
            // Try to clear the fault
            if (!robot.ClearFault()) {
                std::cerr << "[error] Fault cannot be cleared, exiting ..." << std::endl;
                return 1;
            }
            std::cout << "Fault on the connected robot is cleared" << std::endl;
        }

        // Enable the robot, make sure the E-stop is released before enabling
        std::cout << "Enabling robot ..." << std::endl;
        robot.Enable();

        // Wait for the robot to become operational
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "Robot is now operational" << std::endl;

        // Move robot to home pose
        std::cout << "Moving to home pose" << std::endl;
        robot.SwitchMode(rdk::Mode::NRT_PLAN_EXECUTION);
        robot.ExecutePlan("PLAN-Home");
        // Wait for the plan to finish
        while (robot.busy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // Zero Force-torque Sensor
        // =========================================================================================
        robot.SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        // IMPORTANT: must zero force/torque sensor offset for accurate force/torque measurement
        robot.ExecutePrimitive("ZeroFTSensor", std::map<std::string, rdk::FlexivDataTypes> {});

        // WARNING: during the process, the robot must not contact anything, otherwise the result
        // will be inaccurate and affect following operations
        std::cerr
            << "[warn] Zeroing force/torque sensors, make sure nothing is in contact with the robot"
            << std::endl;

        // Wait for primitive to finish
        while (!std::get<int>(robot.primitive_states()["terminated"])) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "Sensor zeroing complete" << std::endl;

        // Search for Contact
        // =========================================================================================
        // NOTE: there are several ways to do contact search, such as using primitives, or real-time
        // and non-real-time direct motion controls, etc. Here we use non-real-time direct Cartesian
        // control for example.
        std::cout << "Searching for contact ..." << std::endl;

        // Set initial pose to current TCP pose
        auto init_pose = robot.states().tcp_pose;
        std::cout << "Initial TCP pose set to [position 3x1, rotation (quaternion) 4x1]: "
                  << rdk::utility::Arr2Str(init_pose) << std::endl;

        // Use non-real-time mode to make the robot go to a set point with its own motion generator
        robot.SwitchMode(rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);

        // Search for contact with max contact wrench set to a small value for making soft contact
        robot.SetMaxContactWrench(kMaxWrenchForContactSearch);

        // Set target point along -Z direction and expect contact to happen during the travel
        auto target_pose = init_pose;
        target_pose[2] -= kSearchDistance;

        // Send target point to robot to start searching for contact and limit the velocity. Keep
        // target wrench 0 at this stage since we are not doing force control yet
        robot.SendCartesianMotionForce(target_pose, {}, {}, kSearchVelocity);

        // Use a while loop to poll robot states and check if a contact is made
        bool is_contacted = false;
        while (!is_contacted) {
            // Compute norm of sensed external force applied on robot TCP
            Eigen::Vector3d ext_force = {robot.states().ext_wrench_in_world[0],
                robot.states().ext_wrench_in_world[1], robot.states().ext_wrench_in_world[2]};

            // Contact is considered to be made if sensed TCP force exceeds the threshold
            if (ext_force.norm() > kPressingForce) {
                is_contacted = true;
                std::cout << "Contact detected at robot TCP" << std::endl;
            }
            // Check at 1ms interval
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Configure Force Control
        // =========================================================================================
        // Switch to real-time mode for continuous motion force control
        robot.SwitchMode(rdk::Mode::RT_CARTESIAN_MOTION_FORCE);

        // Set force control reference frame based on program argument. See function doc for more
        // details
        robot.SetForceControlFrame(force_ctrl_frame);

        // Set which Cartesian axis(s) to activate for force control. See function doc for more
        // details. Here we only active Z axis
        robot.SetForceControlAxis(
            std::array<bool, rdk::kCartDoF> {false, false, true, false, false, false});

        // Uncomment the following line to enable passive force control, otherwise active force
        // control is used by default. See function doc for more details
        /* robot.setPassiveForceControl(true); */

        // NOTE: motion control always uses robot world frame, while force control can use
        // either world or TCP frame as reference frame

        // Start Unified Motion Force Control
        // =========================================================================================
        // Disable max contact wrench regulation. Need to do this AFTER the force control in Z axis
        // is activated (i.e. motion control disabled in Z axis) and the motion force control mode
        // is entered, this way the contact force along Z axis is explicitly regulated and will not
        // spike after the max contact wrench regulation for motion control is disabled
        std::array<double, rdk::kCartDoF> inf;
        inf.fill(std::numeric_limits<double>::infinity());
        robot.SetMaxContactWrench(inf);

        // Update initial pose to current TCP pose
        init_pose = robot.states().tcp_pose;

        // Create real-time scheduler to run periodic tasks
        rdk::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.AddTask(std::bind(PeriodicTask, std::ref(robot), std::ref(init_pose),
                              std::ref(force_ctrl_frame), enable_polish),
            "HP periodic", 1, scheduler.max_priority());
        // Start all added tasks
        scheduler.Start();

        // Block and wait for signal to stop scheduler tasks
        while (!g_stop_sched) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        // Received signal to stop scheduler tasks
        scheduler.Stop();

    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
