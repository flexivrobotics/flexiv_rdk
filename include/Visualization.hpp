/**
 * @file Visualization.hpp
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_VISUALIZATION_HPP_
#define FLEXIVRDK_VISUALIZATION_HPP_

#include <memory>
#include <vector>
#include <string>

namespace flexiv {

class VisualizationHandler;

/**
 * @class Visualization
 * @brief Integrated lightweight visualization based on Meshcat.
 */
class Visualization
{
public:
    Visualization();
    virtual ~Visualization();

    /**
     * @brief Initialize the visualizer with robot only
     * @param[in] robotURDF Path to robot URDF
     * @return True: success, false: failed
     */
    bool init(const std::string& robotURDF);

    /**
     * @brief Initialize the visualizer with robot and scene
     * @param[in] robotURDF Path to robot URDF
     * @param[in] sceneURDF Path to scene URDF
     * @return True: success, false: failed
     */
    bool init(const std::string& robotURDF, const std::string& sceneURDF);

    /**
     * @brief Update visualization with joint positions
     * @param[in] positions \f$ \mathbb{R}^{Dof \times 1} \f$ joint
     * positions \f$ q~[rad] \f$
     * @return True: success, false: failed
     */
    bool update(const std::vector<double>& positions);

private:
    std::unique_ptr<VisualizationHandler> m_handler;
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_VISUALIZATION_HPP_ */
