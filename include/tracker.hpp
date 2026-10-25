#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <array>
#include <vector>

#include <opencv2/core.hpp>

#include "pose.hpp"

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

class OneEuroFilter {
 public:
  OneEuroFilter(double minCutoff = 1.2, double beta = 0.01, double dCutoff = 1.0)
      : minCutoff_(minCutoff), beta_(beta), dCutoff_(dCutoff) {}

  void reset() {
    init_ = false;
    xPrev_ = 0.0;
    dxPrev_ = 0.0;
  }

  double filter(double x, double dt);

 private:
  double alpha(double cutoff, double dt);

  double minCutoff_ = 1.2;
  double beta_ = 0.01;
  double dCutoff_ = 1.0;
  bool init_ = false;
  double xPrev_ = 0.0;
  double dxPrev_ = 0.0;
};

enum class TrackState { Lost, Coasting, Tracking };

class SheetTracker {
 public:
  SheetTracker(const CameraParams& cam, const TrackerParams& p = TrackerParams());

  bool process(const cv::Mat& frame, double dt);
  void reset();

  bool hasPose() const { return state_ != TrackState::Lost && !rvec_.empty() && !tvec_.empty(); }
  TrackState state() const { return state_; }

  const cv::Mat& rvec() const { return rvec_; }
  const cv::Mat& tvec() const { return tvec_; }
  double reprojErr() const { return reprojErr_; }

  const std::vector<cv::Point2f>& rawCorners() const { return raw_; }
  const std::vector<cv::Point2f>& smoothCorners() const { return smooth_; }
  const cv::Rect& searchRoi() const { return search_; }
  const cv::Mat& debugMask() const { return mask_; }

 private:
  bool detect(const cv::Mat& gray, std::vector<cv::Point2f>& out);
  std::vector<cv::Point2f> matchOrder(const std::vector<cv::Point2f>& o) const;

  CameraParams cam_;
  TrackerParams p_;

  TrackState state_ = TrackState::Lost;
  int coast_ = 0;

  std::vector<cv::Point2f> raw_;
  std::vector<cv::Point2f> smooth_;

  cv::Mat rvec_;
  cv::Mat tvec_;
  cv::Mat prevGray_;
  std::vector<cv::Point2f> pred_;
  double reprojErr_ = 1e9;

  cv::Rect search_;
  cv::Mat gray_;
  cv::Mat mask_;

  std::array<OneEuroFilter, 8> filters_;
};

#endif
