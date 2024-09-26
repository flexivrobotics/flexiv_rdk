/**
 * @example basics10_logging_behavior.cpp
 * This tutorial shows how to change the logging behaviors of RDK client.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <iostream>
#include <string>

using namespace flexiv;

namespace {
constexpr char kDefaultLogPattern[] = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v";
}

/** @brief Print program usage help */
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
    spdlog::info(
        ">>> Tutorial description <<<\nThis tutorial shows how to change the logging behaviors of "
        "RDK client.");

    // Suppress log messages from RDK client
    // =========================================================================================
    // Change log level to suppress certain type of log messages from RDK client. The log level
    // setting will apply globally to all following spdlog messages
    spdlog::set_level(spdlog::level::warn); ///< Suppress info messages
    // messages spdlog::set_level(spdlog::level::off); ///< Suppress all messages

    // Instantiate RDK client, all info messages are suppressed
    try {
        rdk::Robot robot(robot_sn);
    } catch (const std::exception& e) {
        spdlog::error(e.what());
    }

    // Output all log messages to a file
    // =========================================================================================
    // Create a logger that outputs the messages to both the console and a log file
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("example.log", true);
    std::vector<spdlog::sink_ptr> sinks {console_sink, file_sink};
    // Since writing to file takes some time, so we use async logger to avoid blocking the program
    spdlog::init_thread_pool(10240, 1);
    auto tp = spdlog::thread_pool();
    auto logger
        = std::make_shared<spdlog::async_logger>("global_logger", sinks.begin(), sinks.end(), tp);
    // Use default pattern for log messages
    logger->set_pattern(kDefaultLogPattern);

    // Use the logger configured above as the default global logger
    spdlog::set_default_logger(logger);

    // Allow all messages
    spdlog::set_level(spdlog::level::info);

    // Instantiate RDK client again, all messages are printed to console and output to a log file
    try {
        rdk::Robot robot(robot_sn);
    } catch (const std::exception& e) {
        spdlog::error(e.what());
    }

    spdlog::warn("This message should also appear in the log file");
    spdlog::info("Program finished");

    return 0;
}
