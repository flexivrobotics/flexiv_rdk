/**
 * @file Scheduler.hpp
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_SCHEDULER_HPP_
#define FLEXIVRDK_SCHEDULER_HPP_

#include <string>
#include <functional>
#include <memory>

namespace flexiv {

/**
 * @class Scheduler
 * @brief Integrated scheduler to add and periodically run user tasks.
 */
class Scheduler
{
public:
    /**
     * @brief Create a flexiv::Scheduler instance.
     * @throw InitException if the instance failed to initialize.
     */
    Scheduler();
    virtual ~Scheduler();

    /**
     * @brief Add new periodic task to the scheduler, a new thread with
     * user-defined priority will be created to run this task.
     * @param[in] callback Callback function of user task
     * @param[in] taskName A unique name for this task
     * @param[in] interval Execution interval of this periodic task [ms], cannot
     * be smaller than 1 ms
     * @param[in] priority Thread priority for this task, can be set to 0 ~ 45,
     * with 0 being the lowest, and 45 being the highest
     * @note Calling this method after start() is not allowed
     * @throw ExecutionException if error occurred during execution.
     * @throw InputException if specified interval or priority is invalid.
     */
    void addTask(std::function<void(void)>&& callback,
        const std::string& taskName, unsigned int interval = 1,
        unsigned int priority = 45);

    /**
     * @brief Start to execute all added tasks periodically.
     * @param[in] isBlocking Whether to block the thread from which this
     * method is called until the scheduler is stopped. A common usage is to
     * call this method from main() with this parameter set to true to keep
     * main() from returning
     * @throw ExecutionException if error occurred during execution.
     */
    void start(bool isBlocking = true);

    /**
     * @brief Stop all added tasks. start() will stop blocking and return.
     * @note Call start() again to restart the scheduler.
     * @throw ExecutionException if error occurred during execution.
     */
    void stop();

    /**
     * @brief Get number of tasks added to the scheduler
     * @return Number of added tasks
     */
    unsigned int getTaskCount() const;

private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_SCHEDULER_HPP_ */
