/**
 * @test test_dynamics_with_tool.cpp
 * A test to evaluate the dynamics engine with tool mounted.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Model.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Utility.hpp>

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

namespace {
/** M, G ground truth from MATLAB */
struct GroundTruth
{
    Eigen::Matrix<double, 7, 7> M;
    Eigen::Matrix<double, 7, 1> G;
};
}

/** Step the dynamics engine once */
void stepDynamics(flexiv::Robot& robot, flexiv::Model& model, const GroundTruth& ref)
{
    // Mark timer start point
    auto tic = std::chrono::high_resolution_clock::now();

    // Get current robot states
    auto robotStates = robot.getRobotStates();

    // Update robot model in dynamics engine
    model.update(robotStates.q, robotStates.dtheta);

    // Get M and G after setTool from dynamic engine
    auto M = model.getMassMatrix();
    auto G = model.getGravityForce();

    // Get and print computation time
    auto toc = std::chrono::high_resolution_clock::now();
    auto computationTime = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count();
    std::cout << "===================================================================" << std::endl;
    std::cout << "Computation time = " << computationTime << " us" << std::endl;

    // evaluate M, G after setTool and compute their norm
    auto deltaM = M - ref.M;
    auto deltaG = G - ref.G;

    std::cout << std::fixed << std::setprecision(5);
    std::cout << "Difference of M between ground truth (MATLAB) and integrated dynamics engine "
                 "after setTool() = "
              << std::endl
              << deltaM << std::endl;
    std::cout << "Norm of delta M: " << deltaM.norm() << '\n' << std::endl;

    std::cout << "Difference of G between ground truth (MATLAB) and integrated dynamics engine "
                 "after setTool() = "
              << std::endl
              << deltaG.transpose() << std::endl;
    std::cout << "Norm of delta G: " << deltaG.norm() << '\n' << std::endl;
}

void printHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot SN]" << std::endl;
    std::cout << "    robot SN: Serial number of the robot to connect to. "
                 "Remove any space, for example: Rizon4s-123456" << std::endl;
    std::cout << "Optional arguments: None" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

int main(int argc, char* argv[])
{
    // Log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //==============================================================================================
    if (argc < 2 || flexiv::utility::programArgsExistAny(argc, argv, {"-h", "--help"})) {
        printHelp();
        return 1;
    }

    // Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456
    std::string robotSN = argv[1];

    try {
        // RDK Initialization
        //==========================================================================================
        // Instantiate robot interface
        flexiv::Robot robot(robotSN);

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

        // set mode after robot is operational
        robot.setMode(flexiv::Mode::NRT_PLAN_EXECUTION);

        // Bring Robot To Home
        //==========================================================================================
        robot.executePlan("PLAN-Home");

        // wait for the execution to finish
        do {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } while (robot.isBusy());

        // put mode back to IDLE
        robot.setMode(flexiv::Mode::IDLE);

        // Robot Model (Dynamics Engine) Initialization
        //==========================================================================================
        flexiv::Model model(robot);

        // Set Tool
        //==========================================================================================
        // artificial tool parameters for verification
        double mass = 0.9;
        // com is relative to flange frame
        Eigen::Vector3d com = {0.0, 0.0, 0.057};
        // inertia relative to com
        Eigen::Matrix3d inertia;
        inertia << 2.768e-03, 0, 0, 0, 3.149e-03, 0, 0, 0, 5.64e-04;

        log.info("Artificial tool parameters:");
        std::cout << "mass = " << mass << std::endl;
        std::cout << "CoM = " << com << std::endl;
        std::cout << "inertia = " << inertia << std::endl;

        // Hard-coded Dynamics Ground Truth from MATLAB
        //==========================================================================================
        GroundTruth ref;
        // clang-format off
        ref.M << 
        2.916316686749461, -0.052869517013466,  1.903540434220357, -0.124348845003517, -0.041914639740668,  0.027649255000000, -0.001464000000000,
        -0.052869517013466,  3.222619358431081,  0.053667041477633, -1.414236317529289, -0.184390078851855,  0.259867572215541, -0.000000000000000,
        1.903540434220357,  0.053667041477633,  1.416023230140298, -0.002724223477633,  0.000093550780077,  0.001155982477633, -0.001121489064726,
        -0.124348845003517, -1.414236317529289, -0.002724223477633,  1.287676548627496,  0.177147398851855, -0.244344118313748,  0.000000000000000,
        -0.041914639740668, -0.184390078851855,  0.000093550780077,  0.177147398851855,  0.054219756143365, -0.038829881851855,  0.000941041060581,
        0.027649255000000,  0.259867572215541,  0.001155982477633, -0.244344118313748, -0.038829881851855,  0.082171270000000,                  0,
        -0.001464000000000, -0.000000000000000, -0.001121489064726,  0.000000000000000,  0.000941041060581,                  0,  0.001464000000000;

        // Matlab value is the joint torque to resist gravity
        ref.G << 
        -0.000000000000001,  52.664497076609663,  0.830964961569619, -22.968509865473024, -2.721399343355234, 3.272076450000000,                 0;
        // clang-format on

        // Step Dynamics Engine
        //==========================================================================================
        int i = 0;
        while (++i <= 5) {
            stepDynamics(robot, model, ref);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

    } catch (const std::exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
