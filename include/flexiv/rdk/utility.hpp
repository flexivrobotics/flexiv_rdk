/**
 * @file utility.hpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_RDK_UTILITY_HPP_
#define FLEXIV_RDK_UTILITY_HPP_

#include "data.hpp"
#include <Eigen/Eigen>
#include <sstream>

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
 * @brief Convert the commonly used std::variant to a string.
 * @param[in] variant std::variant used by multiple rdk::Robot functions.
 * @param[in] decimal Decimal places to keep for each floating-point number in the variant.
 * @param[in] separator Character to separate between numbers in the vector.
 * @return The converted string.
 */
inline std::string FlexivTypes2Str(
    const rdk::FlexivDataTypes& variant, size_t decimal = 3, const std::string& separator = " ")
{
    if (auto* val = std::get_if<int>(&variant)) {
        return Vec2Str(std::vector<int> {*val}, decimal);
    } else if (auto* val = std::get_if<double>(&variant)) {
        return Vec2Str(std::vector<double> {*val}, decimal);
    } else if (auto* val = std::get_if<std::string>(&variant)) {
        return *val;
    } else if (auto* val = std::get_if<rdk::Coord>(&variant)) {
        return (*val).str();
    } else if (auto* vec = std::get_if<std::vector<int>>(&variant)) {
        return Vec2Str(*vec, decimal, separator);
    } else if (auto* vec = std::get_if<std::vector<double>>(&variant)) {
        return Vec2Str(*vec, decimal, separator);
    } else if (auto* vec = std::get_if<std::vector<std::string>>(&variant)) {
        return Vec2Str(*vec, decimal, separator);
    } else if (auto* vec = std::get_if<std::vector<rdk::Coord>>(&variant)) {
        std::string ret;
        // Separate two Coord by " : "
        for (const auto& v : (*vec)) {
            ret += v.str() + " : ";
        }
        // Remove the trailing " : "
        if (!ret.empty()) {
            ret.erase(ret.size() - 3);
        }
        return ret;
    }
    return "";
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

} /* namespace utility */
} /* namespace rdk */
} /* namespace flexiv */

#endif /* FLEXIV_RDK_UTILITY_HPP_ */
