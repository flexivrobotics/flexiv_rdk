/**
 * @file file_io.hpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_RDK_FILE_IO_HPP_
#define FLEXIV_RDK_FILE_IO_HPP_

#include "robot.hpp"

namespace flexiv {
namespace rdk {

/**
 * @class FileIO
 * @brief Interface to exchange files with the robot. Only certain types of file can be transferred.
 */
class FileIO
{
public:
    /**
     * @brief [Non-blocking] Instantiate the file exchange interface.
     * @param[in] robot Reference to the instance of flexiv::rdk::Robot.
     * @throw std::runtime_error if the initialization sequence failed.
     */
    FileIO(const Robot& robot);
    virtual ~FileIO();

    /**
     * @brief [Blocking] A list of all trajectory files currently stored in the connected robot.
     * @return Trajectory filenames as a string list.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    std::vector<std::string> traj_files_list() const;

    /**
     * @brief [Blocking] A list of all user projects currently stored in the connected robot.
     * @return Project names as a string list.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    std::vector<std::string> projects_list() const;

    /**
     * @brief [Blocking] Upload a local trajectory file (.traj) to the robot.
     * @param[in] file_dir Relative or absolute path of the directory that contains the file to
     * upload, e.g. /home/user/Documents/. Do not include the file name here.
     * @param[in] file_name Full name of the trajectory file to upload, including the extension.
     * E.g. PolishSpiral.traj. Do not include the directory path here.
     * @throw std::invalid_argument if failed to find or load the specified file.
     * @throw std::length_error if the file is too large to transfer. Maximum file size is 1 MB.
     * @throw std::runtime_error if failed to upload the file.
     * @note This function blocks until the file is successfully uploaded.
     */
    void UploadTrajFile(const std::string& file_dir, const std::string& file_name);

    /**
     * @brief [Blocking] Download a trajectory file (.traj) from the robot.
     * @param[in] file_name Full name of the trajectory file to download, including the extension.
     * E.g. PolishSpiral.traj.
     * @return String content of the trajectory file.
     * @throw std::invalid_argument if the specified file does not exist.
     * @throw std::runtime_error if failed to download the file.
     * @note This function blocks until the file is successfully downloaded.
     */
    std::string DownloadTrajFile(const std::string& file_name);

    /**
     * @brief [Blocking] Download a trajectory file (.traj) from the robot and save to the specified
     * directory.
     * @param[in] file_name Full name of the trajectory file to download, including the extension.
     * E.g. "PolishSpiral.traj".
     * @param[in] save_dir Relative or absolute path of the directory to save the downloaded file.
     * E.g. /home/user/Documents/. Do not include the file name here. The specified directory will
     * be created if not yet exist.
     * @throw std::invalid_argument if the specified file does not exist.
     * @throw std::runtime_error if failed to download or save the file.
     * @throw std::filesystem::filesystem_error if failed to create the specified directory.
     * @note This function blocks until the file is successfully downloaded.
     */
    void DownloadTrajFile(const std::string& file_name, const std::string& save_dir);

    /**
     * @brief [Blocking] Upload a local project to the robot.
     * @param[in] project_dir Relative or absolute path of the project directory, which should
     * contain a .proj file and a .plan file.
     * @throw std::invalid_argument if failed to find the specified directory or the project files
     * are incomplete.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::length_error if any of the project files is too large to transfer. Maximum file
     * size is 1 MB.
     * @throw std::runtime_error if failed to upload the project.
     * @note Applicable control modes: IDLE.
     * @note This function blocks until the project is successfully uploaded.
     * @warning If the uploaded project was created by Flexiv Elements with a software package
     * version different from the connected robot, the project might not be executable before it's
     * converted to the compatible version.
     */
    void UploadProject(const std::string& project_dir);

    /**
     * @brief [Blocking] Download a project from the robot and save to the specified directory.
     * @param[in] project_name Full name of the project to download. E.g. "polish_table".
     * @param[in] save_dir Relative or absolute path of the directory to save the downloaded
     * project. E.g. /home/user/Documents/.
     * @throw std::invalid_argument if the specified project does not exist.
     * @throw std::runtime_error if failed to download or save the project.
     * @throw std::filesystem::filesystem_error if failed to create the specified directory.
     * @note This function blocks until the project is successfully downloaded.
     */
    void DownloadProject(const std::string& project_name, const std::string& save_dir);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} /* namespace rdk */
} /* namespace flexiv */

#endif /* FLEXIV_RDK_FILE_IO_HPP_ */
