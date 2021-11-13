/**
 * @file Log.hpp
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_LOG_HPP_
#define FLEXIVRDK_LOG_HPP_

#include <string>

namespace flexiv {

/**
 * @class Log
 * @brief Helper functions to print messages with timestamp and coloring.
 * Logging raw data to csv file coming soon.
 */
class Log
{
public:
    Log();
    virtual ~Log();

    /**
     * @brief Pprint info message with timestamp and coloring
     * @param[in] message Info message to log
     * @note Color = green
     */
    void info(const std::string& message) const;

    /**
     * @brief Pprint warning message with timestamp and coloring
     * @param[in] message Warning message to log
     * @note Color = yellow
     */
    void warn(const std::string& message) const;

    /**
     * @brief Pprint error message with timestamp and coloring
     * @param[in] message Error message to log
     * @note Color = red
     */
    void error(const std::string& message) const;

private:
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_LOG_HPP_ */
