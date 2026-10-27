#ifndef TRACKING_SHEET_TRACKER_HPP
#define TRACKING_SHEET_TRACKER_HPP

#include "core/types.hpp"
#include "tracking/one_euro_filter.hpp"
#include "vision/sheet_detector.hpp"

#include <array>
#include <vector>

class SheetTracker {
 public:
  SheetTracker(const CameraParams& cam, const TrackerParams& p = TrackerParams());

  bool process(const cv::Mat& frame, double dt);
  void reset();

  bool hasPose() const { return state_ != TrackState::Lost && !rvec_.empty() && !tvec_.empty(); }
  TrackState state() const { return state_; }
  const cv::Mat& rvec() const { return rvec_; }
  const cv::Mat& tvec() const { return tvec_; }
  const Quad& smoothCorners() const { return smooth_; }
  double reprojErr() const { return reprojErr_; }
  const cv::Rect& searchRoi() const { return detector_.searchRoi(); }
  const cv::Mat& debugMask() const { return detector_.debugMask(); }

 private:
  CameraParams cam_;
  TrackerParams params_;
  SheetDetector detector_;
  std::array<OneEuroFilter, 8> filters_;
  Quad raw_;
  Quad smooth_;
  Quad pred_;
  cv::Mat prevGray_;
  cv::Mat gray_;
  cv::Mat rvec_;
  cv::Mat tvec_;
  TrackState state_ = TrackState::Lost;
  int coast_ = 0;
  double reprojErr_ = 1e9;
};

#endif
