#ifndef IO_CAMERA_LOADER_HPP
#define IO_CAMERA_LOADER_HPP

#include "core/types.hpp"

#include <string>

bool loadCameraParams(const std::string& path, CameraParams& cam,
                      cv::Size frameSize = cv::Size());

#endif
