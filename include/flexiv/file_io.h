/**
 * @file file_io.h
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_FILE_IO_H_
#define FLEXIVRDK_FILE_IO_H_

#include "robot.h"

namespace flexiv {

/**
 * @class FileIO
 * @brief Interface for doing file transfer with the robot. The robot must be put into IDLE mode
 * when transferring files.
 */
class FileIO
{
public:
    /**
     * @brief [Non-blocking] Create an instance and initialize file transfer interface.
     * @param[in] robot Reference to the instance of flexiv::Robot.
     * @throw std::runtime_error if the initialization sequence failed.
     */
    FileIO(const Robot& robot);
    virtual ~FileIO();

    /**
     * @brief [Blocking] Upload a trajectory file (.traj) to the robot.
     * @param[in] file_dir Relative or absolute path of the directory that contains the file to
     * upload, e.g. /home/user/Documents/. Do not include the file name here.
     * @param[in] file_name Full name of the trajectory file to upload, including the suffix, e.g.
     * PolishSpiral.traj. Do not include the directory path here.
     * @throw std::invalid_argument if failed to find or load the specified file.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to transfer the file.
     * @note Applicable control modes: IDLE.
     * @note This function blocks until the file is successfully uploaded.
     */
    void UploadTrajFile(const std::string& file_dir, const std::string& file_name);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_FILE_IO_H_ */
