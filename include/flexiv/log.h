/**
 * @file log.h
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_LOG_H_
#define FLEXIVRDK_LOG_H_

#include <string>

namespace flexiv {

/**
 * @class Log
 * @brief Helper functions to print messages with timestamp and coloring.
 * @warning For maximum portability, std::cout is used to handle the printing, which is a relatively
 * slow process compared to dedicated performance loggers. Therefore, it's not recommended to use
 * flexiv::Log in performance-critical loops.
 */
class Log
{
public:
    Log() = default;
    virtual ~Log() = default;

    /**
     * @brief [Non-blocking] Print info message with timestamp and coloring.
     * @param[in] message Info message.
     * @note Color = green.
     */
    void Info(const std::string& message) const;

    /**
     * @brief [Non-blocking] Print warning message with timestamp and coloring.
     * @param[in] message Warning message.
     * @note Color = yellow.
     */
    void Warn(const std::string& message) const;

    /**
     * @brief [Non-blocking] Print error message with timestamp and coloring.
     * @param[in] message Error message.
     * @note Color = red.
     */
    void Error(const std::string& message) const;
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_LOG_H_ */
