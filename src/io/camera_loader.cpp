#include "io/camera_loader.hpp"

#include <cmath>
#include <iostream>

bool loadCameraParams(const std::string& path, CameraParams& cam, cv::Size frameSize) {
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "Impossible d'ouvrir " << path << std::endl;
    return false;
  }

  fs["camera_matrix"] >> cam.K;
  fs["distortion_coefficients"] >> cam.dist;
  if (cam.K.empty()) return false;
  if (cam.dist.empty()) cam.dist = cv::Mat::zeros(1, 5, CV_64F);
  cam.K.convertTo(cam.K, CV_64F);
  cam.dist.convertTo(cam.dist, CV_64F);

  if (frameSize.area() > 0) {
    double calibW = 2.0 * cam.K.at<double>(0, 2);
    double calibH = 2.0 * cam.K.at<double>(1, 2);
    double sx = frameSize.width / calibW;
    double sy = frameSize.height / calibH;
    if (std::abs(sx - 1.0) > 1e-3 || std::abs(sy - 1.0) > 1e-3) {
      cam.K.at<double>(0, 0) *= sx;
      cam.K.at<double>(0, 2) *= sx;
      cam.K.at<double>(1, 1) *= sy;
      cam.K.at<double>(1, 2) *= sy;
    }
  }
  return true;
}
