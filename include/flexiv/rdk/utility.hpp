/**
 * @file utility.hpp
 * @note Thread safety: all functions declared in this file are thread-safe and can be called
 * concurrently from multiple threads.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_RDK_UTILITY_HPP_
#define FLEXIV_RDK_UTILITY_HPP_

#include "data.hpp"
#include <Eigen/Eigen>
#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <variant>

namespace flexiv::rdk {

namespace utility {

/** @brief Whether the specified group represents a single arm. */
inline bool IsSingleArmGroup(JointGroup group)
{
    return group == JointGroup::ARM_1 || group == JointGroup::ARM_2;
}

/**
 * @brief Check whether one primitive state is true.
 * @param[in] primitive_states Primitive states.
 * @param[in] state_name Primitive state name to check, e.g. "reachedTarget".
 * @return True if the state exists and is true, or is an integer state that is non-zero.
 */
inline bool PrimitiveStateTrue(
    const PrimitiveStates& primitive_states, const std::string& state_name)
{
    const auto state_it = primitive_states.names_and_values.find(state_name);
    if (state_it == primitive_states.names_and_values.end()) {
        return false;
    }

    // "BOOL" states arrive as bool. Integer states still count when non-zero
    if (const auto* flag = std::get_if<bool>(&state_it->second)) {
        return *flag;
    }
    if (const auto* number = std::get_if<int>(&state_it->second)) {
        return *number != 0;
    }
    return false;
}

/**
 * @brief Check whether one primitive state is true for all groups included in [primitive_states].
 * @param[in] primitive_states Primitive states keyed by joint group.
 * @param[in] state_name Primitive state name to check, e.g. "reachedTarget".
 * @return True if all included groups contain the state and it is true, see PrimitiveStateTrue().
 */
inline bool PrimitiveStateTrueForGroups(
    const std::map<JointGroup, PrimitiveStates>& primitive_states, const std::string& state_name)
{
    return std::all_of(primitive_states.begin(), primitive_states.end(),
        [&](const auto& entry) { return PrimitiveStateTrue(entry.second, state_name); });
}

/**
 * @brief Check whether one primitive state is true for each group included in [primitive_states],
 * using a per-group state name.
 * @param[in] primitive_states Primitive states keyed by joint group.
 * @param[in] state_names Primitive state names keyed by joint group.
 * @return True if every included group has a matching state name entry and it is true, see
 * PrimitiveStateTrue().
 */
inline bool PrimitiveStateTrueForGroups(
    const std::map<JointGroup, PrimitiveStates>& primitive_states,
    const std::map<JointGroup, std::string>& state_names)
{
    return std::all_of(primitive_states.begin(), primitive_states.end(), [&](const auto& entry) {
        if (!state_names.contains(entry.first)) {
            return false;
        }
        return PrimitiveStateTrue(entry.second, state_names.at(entry.first));
    });
}

/**
 * @brief Convert quaternion to Euler angles with ZYX axis rotations.
 * @param[in] quat Quaternion input in [w,x,y,z] order. It does not need to be normalized.
 * @return Euler angles in [x,y,z] order [rad].
 * @throw std::invalid_argument if [quat] contains NaN or infinite value, or is (close to) zero and
 * thus does not represent an orientation.
 * @note Every orientation has two equivalent sets of ZYX Euler angles, of which the one with the
 * smaller sum of absolute angles is returned, e.g. [0, 0, -30] deg rather than [180, 180, 150]
 * deg. This is the same selection the robot makes when it converts an orientation to Euler
 * angles, so the return value, when converted to degrees, is the same Euler angles used by Move
 * primitives. At gimbal lock, i.e. a pitch of +/-pi/2, the Z angle is 0.
 */
inline std::array<double, 3> Quat2EulerZYX(const std::array<double, 4>& quat)
{
    // Form quaternion, which must be a unit one for the conversion below to be a rotation.
    // Normalizing does not change the orientation a non-zero quaternion represents
    Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);
    constexpr double kMinQuatNorm = 1e-6;
    if (!q.coeffs().allFinite() || !(q.norm() >= kMinQuatNorm)) {
        throw std::invalid_argument(
            "[flexiv::rdk::utility::Quat2EulerZYX] Input quaternion is not finite or is zero");
    }
    q.normalize();
    const Eigen::Matrix3d rot = q.toRotationMatrix();

    // Fold an angle into [-pi, pi]
    constexpr double kPi = 3.14159265358979323846;
    const auto wrap_to_pi = [kPi](double angle) {
        if (angle > kPi) {
            return angle - 2.0 * kPi;
        }
        if (angle < -kPi) {
            return angle + 2.0 * kPi;
        }
        return angle;
    };

    // Of the Euler angle sets that represent the same orientation, report the one the robot
    // reports (fvr::FvrFrame, with a zero reference). At gimbal lock (pitch at +/-90 deg) only a
    // combination of the Z and X angles is determined, so the Z angle is pinned to 0
    constexpr double kGimbalLockTol = 1e-7;
    if (rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0) < kGimbalLockTol * kGimbalLockTol) {
        const bool pitch_up = rot(2, 0) < 0.0;
        const double angle_x
            = pitch_up ? std::atan2(rot(0, 1), rot(1, 1)) : std::atan2(-rot(0, 1), rot(1, 1));
        return {wrap_to_pi(angle_x), pitch_up ? kPi / 2.0 : -kPi / 2.0, 0.0};
    }

    // Otherwise there are two sets. Eigen returns the one with the Z angle in [0, pi], so a yaw of
    // -30 deg would come back as [180, 180, 150] deg; take the set with the smaller sum of absolute
    // angles instead, as the robot does. Both are in [z,y,x] order
    const Eigen::Vector3d euler_1 = rot.eulerAngles(2, 1, 0);
    const Eigen::Vector3d euler_2(
        wrap_to_pi(euler_1[0] + kPi), wrap_to_pi(kPi - euler_1[1]), wrap_to_pi(euler_1[2] + kPi));
    const Eigen::Vector3d& euler_ZYX
        = (euler_1.cwiseAbs().sum() < euler_2.cwiseAbs().sum()) ? euler_1 : euler_2;

    // Convert to general [x,y,z] order
    return (std::array<double, 3> {euler_ZYX[2], euler_ZYX[1], euler_ZYX[0]});
}

/**
 * @brief Convert radians to degrees for a single value.
 */
inline double Rad2Deg(double rad)
{
    constexpr double kPi = 3.14159265358979323846;
    return (rad / kPi * 180.0);
}

/**
 * @brief Convert radians to degrees for an array.
 */
template <size_t N>
inline std::array<double, N> Rad2Deg(const std::array<double, N>& rad_arr)
{
    std::array<double, N> deg_arr;
    for (size_t i = 0; i < N; i++) {
        deg_arr[i] = Rad2Deg(rad_arr[i]);
    }
    return deg_arr;
}

/**
 * @brief Convert radians to degrees for a vector.
 */
inline std::vector<double> Rad2Deg(const std::vector<double>& rad_vec)
{
    std::vector<double> deg_vec;
    for (const auto& v : rad_vec) {
        deg_vec.push_back(Rad2Deg(v));
    }
    return deg_vec;
}

/**
 * @brief Convert an std::vector to a string.
 * @param[in] vec std::vector of any type and size.
 * @param[in] decimal Decimal places to keep for each floating-point number in the vector.
 * @param[in] separator Character to separate between numbers.
 * @return The converted string.
 */
template <typename T>
inline std::string Vec2Str(
    const std::vector<T>& vec, size_t decimal = 3, const std::string& separator = " ")
{
    std::string padding = "";
    std::ostringstream oss;
    oss.precision(decimal);
    oss << std::fixed;

    for (const auto& v : vec) {
        oss << padding << v;
        padding = separator;
    }
    return oss.str();
}

/**
 * @brief Convert an std::array to a string.
 * @param[in] arr std::array of any type and size.
 * @param[in] decimal Decimal places to keep for each floating-point number in the array.
 * @param[in] separator Character to separate between numbers.
 * @return The converted string.
 */
template <typename T, size_t N>
inline std::string Arr2Str(
    const std::array<T, N>& arr, size_t decimal = 3, const std::string& separator = " ")
{
    std::vector<T> vec(N);
    std::copy(arr.begin(), arr.end(), vec.begin());
    return Vec2Str(vec, decimal, separator);
}

/**
 * @brief Convert a vector of Flexiv data structs to a string, with the elements separated by " : ".
 * @param[in] vec std::vector of rdk::JPos, rdk::Coord, rdk::DJPos, rdk::DCoord, rdk::OCoord or
 * rdk::Pose.
 * @return The converted string.
 */
template <typename T>
inline std::string StructVec2Str(const std::vector<T>& vec)
{
    std::string ret;
    // Separate two structs by " : "
    for (const auto& v : vec) {
        ret += v.str() + " : ";
    }
    // Remove the trailing " : "
    if (!ret.empty()) {
        ret.erase(ret.size() - 3);
    }
    return ret;
}

namespace detail {
/** True if T is any of the types U. */
template <typename T, typename... U>
inline constexpr bool kIsAnyOf = (std::is_same_v<T, U> || ...);
} // namespace detail

/**
 * @brief Convert the commonly used std::variant to a string.
 * @param[in] variant std::variant used by multiple rdk::Robot functions.
 * @param[in] decimal Decimal places to keep for each floating-point number in the variant.
 * @param[in] separator Character to separate between numbers in the vector.
 * @return The converted string.
 * @note The Flexiv data structs (rdk::JPos, rdk::Coord, rdk::DJPos, rdk::DCoord, rdk::OCoord and
 * rdk::Pose) are converted using their own str() function, which always keeps 2 decimal places and
 * separates the numbers by space, thus [decimal] and [separator] do not apply to them. A
 * std::array is formatted like a std::vector, with [separator] between its numbers, while the
 * elements of a std::vector of structs or of std::array are separated by " : ". A bool is written
 * as true or false.
 */
inline std::string FlexivTypes2Str(
    const rdk::FlexivDataTypes& variant, size_t decimal = 3, const std::string& separator = " ")
{
    return std::visit(
        [&](const auto& value) -> std::string {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, bool>) {
                return value ? "true" : "false";
            } else if constexpr (std::is_same_v<T, std::vector<bool>>) {
                std::string ret;
                std::string padding;
                for (const bool v : value) {
                    ret += padding + (v ? "true" : "false");
                    padding = separator;
                }
                return ret;
            } else if constexpr (std::is_same_v<T, int> || std::is_same_v<T, double>) {
                return Vec2Str(std::vector<T> {value}, decimal);
            } else if constexpr (std::is_same_v<T, std::string>) {
                return value;
            } else if constexpr (detail::kIsAnyOf<T, rdk::JPos, rdk::Coord, rdk::DJPos, rdk::DCoord,
                                     rdk::OCoord, rdk::Pose>) {
                return value.str();
            } else if constexpr (detail::kIsAnyOf<T, std::vector<int>, std::vector<double>,
                                     std::vector<std::string>>) {
                return Vec2Str(value, decimal, separator);
            } else if constexpr (requires { std::tuple_size<T>::value; }) {
                // A fixed-size std::array
                return Arr2Str(value, decimal, separator);
            } else if constexpr (requires { std::tuple_size<typename T::value_type>::value; }) {
                // A std::vector of fixed-size std::array
                std::string ret;
                for (const auto& arr : value) {
                    ret += Arr2Str(arr, decimal, separator) + " : ";
                }
                if (!ret.empty()) {
                    ret.erase(ret.size() - 3);
                }
                return ret;
            } else if constexpr (requires { value.front().str(); }) {
                // A std::vector of Flexiv data structs
                return StructVec2Str(value);
            } else {
                static_assert(sizeof(T) == 0, "unhandled alternative of rdk::FlexivDataTypes");
            }
        },
        variant);
}

/**
 * @brief Check if any provided strings exist in the program arguments.
 * @param[in] argc Argument count passed to main() of the program.
 * @param[in] argv Argument vector passed to main() of the program, where argv[0] is the program
 * name.
 * @param[in] ref_strings Reference strings to check against.
 * @return True if the program arguments contain one or more reference strings.
 */
inline bool ProgramArgsExistAny(int argc, char** argv, const std::vector<std::string>& ref_strings)
{
    for (int i = 0; i < argc; i++) {
        for (const auto& v : ref_strings) {
            if (v == std::string(argv[i])) {
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief Check if one specific string exists in the program arguments.
 * @param[in] argc Argument count passed to main() of the program.
 * @param[in] argv Argument vector passed to main() of the program, with argv[0] being the program
 * name.
 * @param[in] ref_strings Reference string to check against.
 * @return True if the program arguments contain this specific reference string.
 */
inline bool ProgramArgsExist(int argc, char** argv, const std::string& ref_strings)
{
    return ProgramArgsExistAny(argc, argv, {ref_strings});
}

/**
 * @brief Scan the network for robots currently reachable by RDK and return their serial numbers.
 * @param[in] timeout_ms Duration [ms] to listen for robots advertising on the network before
 * returning. A longer timeout gives more robots a chance to be discovered.
 * @return Sorted vector of serial numbers (dash-separated format) of the discovered robots. Empty
 * if none were found within the timeout.
 * @note This function opens (or reuses) a Zenoh session and passively listens for the realtime
 * states that every running robot publishes, then collects the unique serial numbers seen. Only
 * robots whose control is currently running and reachable on the same Zenoh network are discovered.
 * @warning This is a blocking function that returns after [timeout_ms].
 */
RDK_API std::vector<std::string> ScanRobotsInNetwork(unsigned int timeout_ms = 1000);

} /* namespace utility */
} /* namespace flexiv::rdk */

#endif /* FLEXIV_RDK_UTILITY_HPP_ */
