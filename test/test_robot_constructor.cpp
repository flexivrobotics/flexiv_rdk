/**
 * @test test_robot_constructor.cpp
 * @brief Test offline construction of flexivrdk.Robot.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 */

#include <flexiv/rdk/robot.hpp>

#include <gtest/gtest.h>

#include <exception>
#include <string>
#include <string_view>

namespace {

template <typename ExceptionT>
void ExpectRobotConstructorFailure(
    const std::string& robot_sn, std::string_view expected_message_fragment)
{
    try {
        flexiv::rdk::Robot robot(robot_sn, false);
        FAIL() << "Expected constructor to fail for serial [" << robot_sn << "], but it succeeded";
    } catch (const ExceptionT& e) {
        const std::string_view message(e.what());
        EXPECT_NE(message.find(expected_message_fragment), std::string_view::npos)
            << "Constructor for serial [" << robot_sn
            << "] threw the expected exception type but with unexpected message: " << e.what();
    } catch (const std::exception& e) {
        FAIL() << "Constructor for serial [" << robot_sn
               << "] threw an unexpected exception type: " << e.what();
    }
}

} // namespace

TEST(RobotConstructor, InvalidSerialNumberFormat)
{
    ExpectRobotConstructorFailure<std::invalid_argument>("Enlight412345", "serial number is invalid");
}

TEST(RobotConstructor, UnsupportedProductModel)
{
    for (const std::string robot_sn : {"Enlight-X-123456", "Enlight X-123456"}) {
        SCOPED_TRACE(robot_sn);
        ExpectRobotConstructorFailure<std::logic_error>(robot_sn, "is not supported");
    }
}

TEST(RobotConstructor, SupportedModelWithoutServer)
{
    for (const std::string robot_sn : {"Enlight-L-123456", "Enlight L-123456"}) {
        SCOPED_TRACE(robot_sn);
        ExpectRobotConstructorFailure<std::runtime_error>(robot_sn, "Failed to connect");
    }
}