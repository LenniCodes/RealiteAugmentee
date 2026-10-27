#ifndef CORE_TYPES_HPP
#define CORE_TYPES_HPP

#include <opencv2/core.hpp>
#include <vector>

struct CameraParams {
  cv::Mat K;
  cv::Mat dist;
};

struct TrackerParams {
  int procWidth = 960;
  double roiMargin = 0.25;
  double minAreaFrac = 0.02;
  int openKernel = 5;
  bool subPix = true;
  double maxReprojErr = 6.0;
  int maxCoastFrames = 8;
  double filterMinCutoff = 1.2;
  double filterBeta = 0.01;
};

using Quad = std::vector<cv::Point2f>;

enum class TrackState { Lost, Coasting, Tracking };

#endif
