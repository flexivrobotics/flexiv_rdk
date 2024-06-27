/**
 * @test test_dynamics_engine.cpp
 * A test to evaluate the dynamics engine (J, M, G), with and without end-effector tool.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/model.hpp>
#include <flexiv/rdk/tool.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>
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
void StepDynamics(flexiv::rdk::Robot& robot, flexiv::rdk::Model& model, const GroundTruth& ref)
{
    // Mark timer start point
    auto tic = std::chrono::high_resolution_clock::now();

    // Update robot model in dynamics engine
    model.Update(robot.states().q, robot.states().dtheta);

    // Get J, M, G from dynamic engine
    Eigen::MatrixXd J = model.J("flange");
    Eigen::MatrixXd M = model.M();
    Eigen::VectorXd G = model.g();

    // Get and print computation time
    auto toc = std::chrono::high_resolution_clock::now();
    auto computation_time
        = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count();
    std::cout << "===================================================================" << std::endl;
    std::cout << "Computation time = " << computation_time << " us" << std::endl;

    // Evaluate J, M, G with true value
    auto delta_J = J - ref.J;
    auto delta_M = M - ref.M;
    auto delta_G = G - ref.G;

    std::cout << std::fixed << std::setprecision(5);
    std::cout << "Difference of J between ground truth (MATLAB) and integrated dynamics engine = "
              << std::endl
              << delta_J << std::endl;
    std::cout << "Norm of delta J: " << delta_J.norm() << '\n' << std::endl;

    std::cout << "Difference of M between ground truth (MATLAB) and integrated dynamics engine = "
              << std::endl
              << delta_M << std::endl;
    std::cout << "Norm of delta M: " << delta_M.norm() << '\n' << std::endl;

    std::cout << "Difference of G between ground truth (MATLAB) and integrated dynamics engine = "
              << std::endl
              << delta_G.transpose() << std::endl;
    std::cout << "Norm of delta G: " << delta_G.norm() << '\n' << std::endl;
}

void PrintHelp()
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
    // Parse Parameters
    //==============================================================================================
    if (argc < 2 || flexiv::rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }

    // Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456
    std::string robot_sn = argv[1];

    try {
        // RDK Initialization
        //==========================================================================================
        // Instantiate robot interface
        flexiv::rdk::Robot robot(robot_sn);

        // Clear fault on the connected robot if any
        if (robot.fault()) {
            spdlog::warn("Fault occurred on the connected robot, trying to clear ...");
            // Try to clear the fault
            if (!robot.ClearFault()) {
                spdlog::error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            spdlog::info("Fault on the connected robot is cleared");
        }

        // Enable the robot, make sure the E-stop is released before enabling
        spdlog::info("Enabling robot ...");
        robot.Enable();

        // Wait for the robot to become operational
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        spdlog::info("Robot is now operational");

        // Set mode after robot is operational
        robot.SwitchMode(flexiv::rdk::Mode::NRT_PLAN_EXECUTION);

        // Bring Robot To Home
        //==========================================================================================
        robot.ExecutePlan("PLAN-Home");

        // Wait for the execution to finish
        do {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } while (robot.busy());

        // Put mode back to IDLE
        robot.SwitchMode(flexiv::rdk::Mode::IDLE);

        // Test Dynamics Engine without Tool
        //==========================================================================================
        spdlog::info(">>>>> Test 1: no end-effector tool <<<<<");

        // Instantiate dynamics engine
        flexiv::rdk::Model model(robot);

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
            StepDynamics(robot, model, ref);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Test Dynamics Engine with Tool
        //==========================================================================================
        spdlog::info(">>>>> Test 2: with end-effector tool <<<<<");

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
        flexiv::rdk::Tool tool(robot);

        // Set name and parameters for the test tool
        std::string tool_name = "DynamicsTestTool";
        flexiv::rdk::ToolParams tool_params;
        tool_params.mass = 0.9;
        tool_params.CoM = {0.0, 0.0, 0.057};
        tool_params.inertia = {2.768e-03, 3.149e-03, 5.64e-04, 0.0, 0.0, 0.0};
        tool_params.tcp_location = {0.0, -0.207, 0.09, 0.7071068, 0.7071068, 0.0, 0.0};

        // Remove any existing tool with the same name
        if (tool.exist(tool_name)) {
            spdlog::warn("Tool with the same name [{}] already exists, removing it now", tool_name);
            // Switch to other tool or no tool (Flange) before removing the current tool
            tool.Switch("Flange");
            tool.Remove(tool_name);
        }

        // Add the test tool
        spdlog::info("Adding test tool [{}] to the robot", tool_name);
        tool.Add(tool_name, tool_params);

        // Switch to the newly added test tool, i.e. set it as the active tool
        spdlog::info("Switching to test tool [{}]", tool_name);
        tool.Switch(tool_name);

        // Get and print the current active tool, should be the test tool
        spdlog::info("Current active tool: {}", tool.name());

        // Reload robot + tool model using the latest data synced from the connected robot
        model.Reload();

        // Step the dynamics engine again
        i = 0;
        while (++i <= 3) {
            StepDynamics(robot, model, ref);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Switch to other tool or no tool (Flange) before removing the current tool
        tool.Switch("Flange");

        // Clean up by removing the test tool
        spdlog::info("Removing tool [{}]", tool_name);
        tool.Remove(tool_name);

        spdlog::info("Program finished");
    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
