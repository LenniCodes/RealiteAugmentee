#ifndef VISION_SHEET_DETECTOR_HPP
#define VISION_SHEET_DETECTOR_HPP

#include "core/types.hpp"

#include <opencv2/core.hpp>

class SheetDetector {
 public:
  SheetDetector(const CameraParams& cam, const TrackerParams& p = TrackerParams());

  bool detect(const cv::Mat& gray, const Quad& reference, bool allowFullFrame,
              Quad& out);
  const cv::Rect& searchRoi() const { return searchRoi_; }
  const cv::Mat& debugMask() const { return debugMask_; }

 private:
  CameraParams cam_;
  TrackerParams params_;
  cv::Rect searchRoi_;
  cv::Mat debugMask_;
};

#endif
