/**
 * @file Visualization.hpp
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_VISUALIZATION_HPP_
#define FLEXIVRDK_VISUALIZATION_HPP_

#include <Eigen/Eigen>

#include <memory>
#include <vector>
#include <string>

namespace flexiv {

/**
 * @class Visualization
 * @brief Integrated lightweight visualization based on Meshcat.
 */
class Visualization
{
public:
    /**
     * @brief Initialize the visualizer with robot only.
     * @param[in] robotURDF Path to robot URDF.
     * @throw InitException if the instance failed to initialize.
     */
    Visualization(const std::string& robotURDF);

    /**
     * @brief Initialize the visualizer with robot and scene.
     * @param[in] robotURDF Path to robot URDF.
     * @param[in] sceneURDF Path to scene URDF.
     * @throw InitException if the instance failed to initialize.
     */
    Visualization(const std::string& robotURDF, const std::string& sceneURDF);
    virtual ~Visualization();

    /**
     * @brief Update robot visualization with new joint positions.
     * @param[in] positions \f$ \mathbb{R}^{Dof \times 1} \f$ joint
     * positions \f$ q~[rad] \f$.
     * @throw ExecutionException if error occurred during execution.
     */
    void updateRobot(const std::vector<double>& positions);

    /**
     * @brief Update scene visualization by specifying new position and
     * orientation of an existing object in the scene.
     * @param[in] objectName Name of the object in the scene to update, as
     * specified in the scene URDF.
     * @param[in] position New position of the specified object in world frame.
     * @param[in] orientation New orientation of the specified object in world
     * frame.
     * @throw ExecutionException if error occurred during execution.
     */
    void updateScene(const std::string& objectName, Eigen::Vector3d position,
        Eigen::Matrix3d orientation);

    /**
     * @brief Update scene visualization by specifying new color of an existing
     * object in the scene.
     * @param[in] objectName Name of the object in the scene to update, as
     * specified in the scene URDF.
     * @param[in] r Red component, 0 to 1
     * @param[in] g Green component, 0 to 1
     * @param[in] b Blue component, 0 to 1
     * @param[in] alpha Transparency, 0 to 1
     * @throw ExecutionException if error occurred during execution.
     */
    void updateScene(const std::string& objectName, double r, double g,
        double b, double alpha);

private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_VISUALIZATION_HPP_ */
