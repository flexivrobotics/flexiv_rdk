/**
 * @file utility.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_RDK_UTILITY_HPP_
#define FLEXIV_RDK_UTILITY_HPP_

#include <Eigen/Eigen>
#include <array>

namespace flexiv {
namespace rdk {
namespace utility {

/**
 * @brief Convert quaternion to Euler angles with ZYX axis rotations.
 * @param[in] quat Quaternion input in [w,x,y,z] order.
 * @return Euler angles in [x,y,z] order [rad].
 * @note The return value, when converted to degrees, is the same Euler angles used by Move
 * primitives.
 */
inline std::array<double, 3> Quat2EulerZYX(const std::array<double, 4>& quat)
{
    // Form quaternion
    Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);

    // The returned array is in [z,y,x] order
    auto euler_ZYX = q.toRotationMatrix().eulerAngles(2, 1, 0);

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
 * @brief Convert a std::array to a string.
 * @param[in] arr std::array of any type and size.
 * @param[in] decimal Decimal places to keep for each number in the array.
 * @param[in] trailing_space Whether to include a space after the last element.
 * @param[in] separator Character to separate between numbers.
 * @return A string with format "arr[0] arr[1] ... arr[n] ", i.e. each element followed by a space,
 * including the last one if trailing_space = true.
 */
template <typename T, size_t N>
inline std::string Arr2Str(const std::array<T, N>& arr, size_t decimal = 3,
    bool trailing_space = true, const std::string& separator = " ")
{
    std::string padding = "";
    std::stringstream ss;
    ss.precision(decimal);
    ss << std::fixed;

    for (const auto& v : arr) {
        ss << padding << v;
        padding = separator;
    }

    if (trailing_space) {
        ss << " ";
    }
    return ss.str();
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
 * @brief Parse the value of a specified primitive state from the pt_states string list.
 * @param[in] pt_states Primitive states string list returned from Robot::primitive_states().
 * @param[in] parse_target Name of the primitive state to parse for.
 * @return Value of the specified primitive state in string format. Empty string is returned if
 * parse_target does not exist.
 */
inline std::string ParsePtStates(
    const std::vector<std::string>& pt_states, const std::string& parse_target)
{
    for (const auto& state : pt_states) {
        // Skip if empty
        if (state.empty()) {
            continue;
        }
        std::stringstream ss(state);
        std::string buffer;
        std::vector<std::string> parsed_state;
        while (ss >> buffer) {
            parsed_state.push_back(buffer);
        }
        if (parsed_state.front() == parse_target) {
            return parsed_state.back();
        }
    }

    return "";
}

} /* namespace utility */
} /* namespace rdk */
} /* namespace flexiv */

#endif /* FLEXIV_RDK_UTILITY_HPP_ */
