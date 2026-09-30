/**
 * @example basics10_logging_behavior.cpp
 * This tutorial shows how to change the logging behaviors of RDK client, how the example programs
 * log messages, and how to capture that output to a file. RDK examples log through the C++ standard
 * streams: informational messages go to std::cout and warnings/errors go to std::cerr.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/utility.hpp>

#include <fstream>
#include <iostream>
#include <string>

using namespace flexiv;

/** @brief Print program usage help */
void PrintHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot_sn]" << std::endl;
    std::cout << "    robot_sn: Serial number of the robot to connect. Remove any space, e.g. Rizon4s-123456" << std::endl;
    std::cout << "Optional arguments: None" << std::endl;
    std::cout << std::endl;
    // clang-format on
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
    std::cout << ">>> Tutorial description <<<\nThis tutorial shows how to change the logging "
                 "behaviors of RDK client.\n"
              << std::endl;

    // Suppress log messages from RDK client
    // =============================================================================================
    // RDK client keeps its logging implementation to itself: the logging library it uses is
    // statically embedded into the RDK library and its symbols are hidden, so it cannot conflict
    // with whatever logging library the user application chooses. What can be configured from the
    // outside is how much RDK client prints: pass verbose = false to the rdk::Robot constructor to
    // suppress its info and warning messages.
    try {
        rdk::Robot robot(robot_sn, {}, false);
    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
    }

    // Logging in the examples
    // =============================================================================================
    // The RDK examples do not depend on any third-party logging library. They log through the C++
    // standard streams: informational messages are written to std::cout and warnings/errors to
    // std::cerr.
    std::cout << "This is an informational message on std::cout" << std::endl;
    std::cerr << "[warn] This is a warning message on std::cerr" << std::endl;
    std::cerr << "[error] This is an error message on std::cerr" << std::endl;

    // Output all log messages to a file
    // =============================================================================================
    // To capture an example's output to a file, redirect the standard streams on the command line,
    // e.g. run the program as:
    //     ./basics10_logging_behavior [robot_sn] > example.log 2>&1
    // The snippet below shows how to do the same from within the program by redirecting the stream
    // buffers to a file for the remainder of execution.
    std::ofstream log_file("example.log", std::ios::trunc);
    if (log_file.is_open()) {
        // Mirror std::cout and std::cerr into the log file for the rest of the program
        std::cout.rdbuf(log_file.rdbuf());
        std::cerr.rdbuf(log_file.rdbuf());
    }

    // Instantiate RDK client again, this time with the default verbose = true, so all of its
    // messages are printed as well and end up in the log file
    try {
        rdk::Robot robot(robot_sn);
    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
    }

    std::cerr << "[warn] This message should also appear in the log file" << std::endl;
    std::cout << "Program finished" << std::endl;

    return 0;
}
