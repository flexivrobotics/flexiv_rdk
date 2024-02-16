/**
 * @test test_dynamics_engine.cpp
 * A test to evaluate the dynamics engine (J, M, G), with and without end-effector tool.
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Model.hpp>
#include <flexiv/Tool.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Utility.hpp>

#include <Eigen/Eigen>

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

namespace {
/** J, M, G ground truth from MATLAB */
struct GroundTruth
{
    /** Jacobian matrix */
    Eigen::Matrix<double, 6, 7> J;
    /** Mass matrix */
    Eigen::Matrix<double, 7, 7> M;
    /** Gravity torques */
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

    // Get J, M, G from dynamic engine
    Eigen::MatrixXd J = model.getJacobian("flange");
    Eigen::MatrixXd M = model.getMassMatrix();
    Eigen::VectorXd G = model.getGravityForce();

    // Get and print computation time
    auto toc = std::chrono::high_resolution_clock::now();
    auto computationTime = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count();
    std::cout << "===================================================================" << std::endl;
    std::cout << "Computation time = " << computationTime << " us" << std::endl;

    // Evaluate J, M, G with true value
    auto deltaJ = J - ref.J;
    auto deltaM = M - ref.M;
    auto deltaG = G - ref.G;

    std::cout << std::fixed << std::setprecision(5);
    std::cout << "Difference of J between ground truth (MATLAB) and integrated dynamics engine = "
              << std::endl
              << deltaJ << std::endl;
    std::cout << "Norm of delta J: " << deltaJ.norm() << '\n' << std::endl;

    std::cout << "Difference of M between ground truth (MATLAB) and integrated dynamics engine = "
              << std::endl
              << deltaM << std::endl;
    std::cout << "Norm of delta M: " << deltaM.norm() << '\n' << std::endl;

    std::cout << "Difference of G between ground truth (MATLAB) and integrated dynamics engine = "
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

        // Create data struct for storing robot states
        flexiv::RobotStates robotStates;

        // Clear fault on the connected robot if any
        if (robot.isFault()) {
            log.warn("Fault occurred on the connected robot, trying to clear ...");
            // Try to clear the fault
            if (!robot.clearFault()) {
                log.error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            log.info("Fault on the connected robot is cleared");
        }

        // Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...");
        robot.enable();

        // Wait for the robot to become operational
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        log.info("Robot is now operational");

        // Set mode after robot is operational
        robot.setMode(flexiv::Mode::NRT_PLAN_EXECUTION);

        // Bring Robot To Home
        //==========================================================================================
        robot.executePlan("PLAN-Home");

        // Wait for the execution to finish
        do {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } while (robot.isBusy());

        // Put mode back to IDLE
        robot.setMode(flexiv::Mode::IDLE);

        // Test Dynamics Engine without Tool
        //==========================================================================================
        log.info(">>>>> Test 1: no end-effector tool <<<<<");

        // Instantiate dynamics engine
        flexiv::Model model(robot);

        // Ground truth from MATLAB without robot tool
        GroundTruth ref;
        // clang-format off
        ref.J <<
        0.110000000000000,  0.078420538028673,  0.034471999940354, -0.368152340866938, -0.064278760968654,  0.136000000000000, -0.000000000000000,
        0.687004857483100, -0.000000000000000,  0.576684003660457,  0.000000000000000,  0.033475407198662, -0.000000000000000, -0.000000000000000,
        0.000000000000000,  0.687004857483100, -0.028925442435894, -0.417782862794537, -0.076604444311898,  0.110000000000000, -0.000000000000000,
        0.000000000000000, -0.000000000000000,  0.642787609686539,  0.000000000000000,  0.766044443118978, -0.000000000000000,  0.000000000000000,
                        0, -1.000000000000000, -0.000000000000000,  1.000000000000000,  0.000000000000000, -1.000000000000000,  0.000000000000000,
        1.000000000000000, -0.000000000000000,  0.766044443118978,  0.000000000000000, -0.642787609686539, -0.000000000000000, -1.000000000000000;
        ref.M << 
        2.480084579964625, -0.066276150278304,  1.520475428405090, -0.082258763257690, -0.082884472612488,  0.008542255000000, -0.000900000000000,
        -0.066276150278304,  2.778187401738267,  0.067350373889147, -1.100953424157635, -0.129191018084721,  0.165182543869134, -0.000000000000000,
        1.520475428405090,  0.067350373889147,  1.074177611590570, -0.000410055889147, -0.043572229972025, -0.001968185110853, -0.000689439998807,
        -0.082258763257690, -1.100953424157635, -0.000410055889147,  0.964760218577003,  0.123748338084721, -0.125985653288502,  0.000000000000000,
        -0.082884472612488, -0.129191018084721, -0.043572229972025,  0.123748338084721,  0.038006882479315, -0.020080821084721,  0.000578508848718,
        0.008542255000000,  0.165182543869134, -0.001968185110853, -0.125985653288502, -0.020080821084721,  0.034608170000000,                  0,
        -0.000900000000000, -0.000000000000000, -0.000689439998807,  0.000000000000000,  0.000578508848718,                  0,  0.000900000000000;
        ref.G << 
        0.000000000000000, 46.598931189891374, 1.086347692836129, -19.279904969860052, -2.045058704525489,  2.300886450000000,                  0;
        // clang-format on

        // Step the dynamics engine
        int i = 0;
        while (++i <= 3) {
            stepDynamics(robot, model, ref);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Test Dynamics Engine with Tool
        //==========================================================================================
        log.info(">>>>> Test 2: with end-effector tool <<<<<");

        // Ground truth from MATLAB with robot tool
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

        // Instantiate tool interface
        flexiv::Tool tool(robot);

        // Set name and parameters for the test tool
        std::string toolName = "DynamicsTestTool";
        flexiv::ToolParams toolParams;
        toolParams.mass = 0.9;
        toolParams.CoM = {0.0, 0.0, 0.057};
        toolParams.inertia = {2.768e-03, 3.149e-03, 5.64e-04, 0.0, 0.0, 0.0};
        toolParams.tcpLocation = {0.0, -0.207, 0.09, 0.7071068, 0.7071068, 0.0, 0.0};

        // Remove any existing tool with the same name
        if (tool.isExist(toolName)) {
            tool.remove(toolName);
        }

        // Add the test tool
        log.info("Adding test tool [" + toolName + "] to the robot");
        tool.add(toolName, toolParams);

        // Switch to the newly added test tool, i.e. set it as the active tool
        log.info("Switching to test tool [" + toolName + "]");
        tool.switchTo(toolName);

        // Get and print the current active tool, should be the test tool
        log.info("Current active tool: " + tool.getCurrentToolName());

        // Reload robot + tool model using the latest data synced from the connected robot
        model.reload();

        // Step the dynamics engine again
        i = 0;
        while (++i <= 3) {
            stepDynamics(robot, model, ref);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Clean up by removing the test tool
        log.info("Removing tool [" + toolName + "]");
        tool.remove(toolName);

        log.info("Program finished");
    } catch (const std::exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
