/**
 * @test test_log.cpp
 * Test log functions of the flexiv::Log class
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/log.h>
#include <flexiv/utility.h>

#include <iostream>
#include <thread>

void PrintHelp()
{
    // clang-format off
    std::cout << "Required arguments: None" << std::endl;
    std::cout << "Optional arguments: None" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

int main(int argc, char* argv[])
{
    if (flexiv::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }

    // log object for printing message with timestamp and coloring
    flexiv::Log log;

    // print info message
    log.Info("This is an INFO message with timestamp and GREEN coloring");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // print warning message
    log.Warn("This is a WARNING message with timestamp and YELLOW coloring");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // print error message
    log.Error("This is an ERROR message with timestamp and RED coloring");

    return 0;
}
