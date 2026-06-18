/**
 * @file model.hpp
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_RDK_MODEL_HPP_
#define FLEXIV_RDK_MODEL_HPP_

#include "robot.hpp"
#include <Eigen/Eigen>
#include <memory>

namespace flexiv::rdk {

/**
 * @struct IKParams
 * @brief Per-joint-group IK input parameters for SolveConstrainedIK().
 */
struct IKParams
{
    /** Cartesian pose to be solved. */
    std::array<double, kPoseSize> cartesian_pose = {};

    /** Seed joint positions for IK solver. Unit: [rad]. */
    std::vector<double> seed_q = {};

    /** Whether to only constrain Cartesian position and allow orientation to vary. */
    bool free_orientation = false;
};

/**
 * @struct IKResult
 * @brief Result from SolveConstrainedIK().
 */
struct IKResult
{
    /** Whether the IK solution is successful. */
    bool success = false;

    /** Solved joint positions by active joint group. Unit: \f$ [rad] \f$. */
    std::map<JointGroup, std::vector<double>> solved_q = {};
};

/**
 * @class Model
 * @brief Interface to obtain certain model data of the robot, including kinematics and dynamics.
 */
class Model
{
public:
    /**
     * @brief [Non-blocking] Instantiate the robot model interface.
     * @param[in] robot Reference to the instance of flexiv::rdk::Robot.
     * @param[in] gravity_vector Earth's gravity vector in world frame. Default to \f$ [0.0, 0.0,
     * -9.81]^T \f$. Unit: \f$ [m/s^2] \f$.
     * @throw std::runtime_error if the initialization sequence failed.
     * @throw std::logic_error if the connected robot does not have an RDK professional license; or
     * the parsed robot model is not supported.
     */
    Model(const Robot& robot,
        const Eigen::Vector3d& gravity_vector = Eigen::Vector3d(0.0, 0.0, -9.81));
    virtual ~Model();

    /**
     * @brief [Non-blocking] Names of all links in the robot model.
     * @return Name vector in the same order as the robot's kinematic chain.
     */
    std::vector<std::string> link_names() const;

    /**
     * @brief [Non-blocking] Names of all actuated joints in the robot model.
     * @return Name vector in the same order as the robot's kinematic chain.
     */
    std::vector<std::string> joint_names() const;

    /**
     * @brief [Blocking] Reload (refresh) parameters of the robot model stored locally in this class
     * using the latest data synced from the connected robot. Tool model is also synced.
     * @throw std::runtime_error if failed to sync model data.
     * @throw std::logic_error if the synced robot model contains invalid data.
     * @note This function blocks until the model parameters are synced and reloaded.
     * @note This function does not affect the kinematics functions.
     * @warning Parameters of the locally-stored robot model must be manually refreshed using this
     * function whenever a physical change is made to the connected robot (e.g. a tool is added or
     * changed). Otherwise the locally computed functions will return incorrect results.
     */
    void Reload();

    /**
     * @brief [Non-blocking] Update the configuration (posture) of the locally-stored robot model so
     * that the locally computed functions return results based on the updated configuration.
     * @param[in] full_q Current joint positions of the whole robot and matching the order of
     * joint_names(): \f$ q \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad] \f$.
     * @param[in] full_dq Current joint velocities of the whole robot and matching the order of
     * joint_names(): \f$ \dot{q} \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad/s] \f$.
     * @throw std::invalid_argument if size of any input vector does not match robot full DoF.
     */
    void Update(const std::vector<double>& full_q, const std::vector<double>& full_dq);

    //========================================== DYNAMICS ==========================================
    /**
     * @brief [Non-blocking] Compute the time derivative of Jacobian matrix at the specified frame
     * w.r.t. world frame.
     * @param[in] link_name Name of the link whose frame is the specified one.
     * @return Time derivative of the Jacobian matrix: \f$ ^{O}\dot{J_i} \in \mathbb{R}^{m \times n}
     * \f$.
     * @throw std::invalid_argument if [link_name] does not exist.
     * @note Call Update() before this function.
     * @see link_names().
     */
    Eigen::MatrixXd dJ(const std::string& link_name);

    /**
     * @brief [Non-blocking] Compute the mass matrix in generalized coordinates, i.e. joint space.
     * @return Symmetric positive definite mass matrix: \f$ M(q) \in \mathbb{S}^{n \times n}_{++}
     * \f$. Unit: \f$ [kgm^2] \f$.
     * @note Call Update() before this function.
     */
    Eigen::MatrixXd M();

    /**
     * @brief [Non-blocking] Compute the Coriolis/centripetal matrix in generalized coordinates,
     * i.e. joint space.
     * @return Coriolis/centripetal matrix: \f$ C(q,\dot{q}) \in \mathbb{R}^{n \times n} \f$.
     * @note Call Update() before this function.
     */
    Eigen::MatrixXd C();

    /**
     * @brief [Non-blocking] Compute the gravity force vector in generalized coordinates, i.e.
     * joint space.
     * @return Gravity force vector: \f$ g(q) \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [Nm] \f$.
     * @note Call Update() before this function.
     */
    Eigen::VectorXd g();

    /**
     * @brief [Non-blocking] Compute the Coriolis force vector in generalized coordinates, i.e.
     * joint space.
     * @return Coriolis force vector: \f$ c(q,\dot{q}) \in \mathbb{R}^{n \times 1} \f$. Unit: \f$
     * [Nm] \f$.
     * @note Call Update() before this function.
     */
    Eigen::VectorXd c();

    //========================================= KINEMATICS =========================================
    /**
     * @brief [Non-blocking] Compute the Jacobian matrix at the specified frame w.r.t. world frame.
     * @param[in] link_name Name of the link whose frame is the specified one.
     * @return Jacobian matrix: \f$ ^{O}J_i \in \mathbb{R}^{m \times n} \f$.
     * @throw std::invalid_argument if [link_name] does not exist.
     * @note Call Update() before this function.
     * @see link_names().
     */
    Eigen::MatrixXd J(const std::string& link_name);

    /**
     * @brief [Non-blocking] Compute the transformation matrix of the specified frame w.r.t. world
     * frame.
     * @param[in] link_name Name of the link whose frame is the specified one.
     * @return Transformation matrix: \f$ ^{O}T_i \in \mathbb{R}^{4 \times 4} \f$.
     * @throw std::invalid_argument if [link_name] does not exist.
     * @note Call Update() before this function.
     * @see link_names().
     */
    Eigen::Isometry3d T(const std::string& link_name);

    /**
     * @brief [Blocking] Sync the actual kinematic parameters of the connected robot into the
     * template URDF file.
     * @param[in] template_urdf_path Path to the template URDF file that can be generated in
     * [flexiv_description](https://github.com/flexivrobotics/flexiv_description). This template
     * URDF file will be updated when the sync is finished.
     * @throw std::invalid_argument if failed to load the template URDF file.
     * @throw std::runtime_error if failed to sync the URDF file.
     * @note This function blocks until the sync is finished.
     * @par Why is this function needed?
     * The template URDF contains kinematic parameters of the latest version of robot hardware,
     * which might be different from older versions. This function is therefore provided to sync the
     * actual kinematic parameters of the connected robot into the template URDF.
     */
    void SyncURDF(const std::string& template_urdf_path);

    /**
     * @brief [Blocking] Sync the actual kinematic parameters of the connected robot into the
     * template YAML file.
     * @param[in] template_yaml_path Path to the template YAML file located at
     * [flexiv_description/config/.../default_kinematics.yaml]. This template YAML file will be
     * updated when the sync is finished.
     * @return Total number of joints that have been successfully synced.
     * @throw std::invalid_argument if failed to load the template YAML file.
     * @throw std::runtime_error if failed to sync the YAML file.
     * @note This function blocks until the sync is finished.
     */
    size_t SyncKinematicsYAML(const std::string& template_yaml_path);

    /**
     * @brief [Blocking] Solve constrained IK using one or more active joint groups.
     * @param[in] ik_params_by_group IK input parameters mapped by active joint group. Joint groups
     * not included in this map are treated as inactive. Only single-arm joint groups like ARM_1 and
     * ARM_2 are accepted.
     * @return Solver result. [solved_q] contains one entry per requested active joint group.
     * @throw std::invalid_argument if input map is empty, if any joint group is not an existing
     * single-arm joint group, or if any [seed_q] size does not match DoF of its joint group.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    IKResult SolveConstrainedIK(const std::map<JointGroup, IKParams>& ik_params_by_group);

    /**
     * @brief [Blocking] Score of each joint group's current configuration (posture), calculated
     * from the manipulability measurements.
     * @return Configuration score mapped by joint group as {translation_score,
     * orientation_score}. The quality of configuration based on the score can be interpreted as:
     * poor = [0, 20), medium = [20, 40), good = [40, 100].
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     * @warning A poor configuration score means the robot is near or at singularity, which can lead
     * to degraded Cartesian performance. Use configuration with high scores for better
     * manipulability and task results.
     */
    std::map<JointGroup, std::pair<double, double>> configuration_score() const;

    //======================================= MULTI-CONTACT ========================================
    /**
     * @brief [Non-blocking] Estimated multi-contact forces applied on each link of applicable joint
     * groups, calculated using the force-torque sensors installed in every joint of the robot.
     * @return \f$ f_c \in \mathbb{R}^{n \times 1} \f$ mapped by joint group. Each vector element is
     * a \f$ \mathbb{R}^{3 \times 1} \f$ force vector w.r.t. the corresponding link frame. Only
     * contains joint groups that are capable of multi-contact estimation.
     * @warning This data is only available on certain robot models. An empty vector will be
     * returned if the connected robot does not support multi-contact estimation.
     */
    std::map<JointGroup, std::vector<Eigen::Vector3d>> multi_contact_forces() const;

    /**
     * @brief [Non-blocking] Estimated multi-contact positions on each link of applicable joint
     * groups, calculated using the force-torque sensors installed in every joint of the robot.
     * @return \f$ p_c \in \mathbb{R}^{n \times 1} \f$ mapped by joint group. Each vector element is
     * a \f$ \mathbb{R}^{3 \times 1} \f$ position vector w.r.t. the corresponding link frame. Only
     * contains joint groups that are capable of multi-contact estimation.
     * @warning This data is only available on certain robot models. An empty vector will be
     * returned if the connected robot does not support multi-contact estimation.
     */
    std::map<JointGroup, std::vector<Eigen::Vector3d>> multi_contact_positions() const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} /* namespace flexiv::rdk */

#endif /* FLEXIV_RDK_MODEL_HPP_ */
