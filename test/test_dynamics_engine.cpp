/**
 * @test test_dynamics_engine.cpp
 * A test to evaluate the dynamics engine (J, M, G)
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Model.hpp>
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
    Eigen::Matrix<double, 6, 7> J;
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

        // Robot Model (Dynamics Engine) Initialization
        //==========================================================================================
        flexiv::Model model(robot);

        // Hard-coded Dynamics Ground Truth from MATLAB
        //==========================================================================================
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

        // Matlab value is the joint torque to resist gravity
        ref.G << 
        0.000000000000000, 46.598931189891374, 1.086347692836129, -19.279904969860052, -2.045058704525489,  2.300886450000000,                  0;
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
