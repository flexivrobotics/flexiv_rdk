/**
 * @test test_log.cpp
 * Test log functions of the flexiv::Log class
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Log.hpp>
#include <thread>

int main(int argc, char* argv[])
{
    // log object for printing message with timestamp and coloring
    flexiv::Log log;

    // check if program has 3 arguments
    if (argc != 1) {
        log.error("No program argument is needed");
        return 0;
    }

    // print info message
    log.info("This is an INFO message with timestamp and GREEN coloring");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // print warning message
    log.warn("This is a WARNING message with timestamp and YELLOW coloring");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // print error message
    log.error("This is an ERROR message with timestamp and RED coloring");

    return 0;
}
