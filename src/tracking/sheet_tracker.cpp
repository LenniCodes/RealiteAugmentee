#include "tracking/sheet_tracker.hpp"

#include "core/quad_utils.hpp"
#include "geometry/pose_estimator.hpp"
#include "vision/edge_refiner.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace {

bool validRect(const cv::Rect& r, int cols, int rows) {
  return r.x >= 0 && r.y >= 0 && r.width > 0 && r.height > 0 &&
         r.x + r.width <= cols && r.y + r.height <= rows;
}

bool flowQuad(const cv::Mat& prev, const cv::Mat& cur, const Quad& quad, Quad& out) {
  if (quad.size() != 4) return false;
  const cv::Rect full(0, 0, cur.cols, cur.rows);
  const cv::Rect b = cv::boundingRect(quad);
  const cv::Rect crop = cv::Rect(b.x - b.width / 2, b.y - b.height / 2, b.width * 2, b.height * 2) & full;
  if (!validRect(crop, cur.cols, cur.rows) || crop.width < 64 || crop.height < 64) return false;

  const cv::Point2f off(float(crop.x), float(crop.y));
  std::vector<cv::Point2f> p0(4), fwd, bwd;
  for (int i = 0; i < 4; ++i) p0[i] = quad[i] - off;
  const cv::Mat A = prev(crop), B = cur(crop);
  std::vector<uchar> st1, st2;
  std::vector<float> e1, e2;
  const cv::Size win(21, 21);
  cv::calcOpticalFlowPyrLK(A, B, p0, fwd, st1, e1, win, 2);
  if (fwd.size() != 4 || st1.size() != 4) return false;
  cv::calcOpticalFlowPyrLK(B, A, fwd, bwd, st2, e2, win, 2);
  if (bwd.size() != 4 || st2.size() != 4) return false;
  for (int i = 0; i < 4; ++i) if (!st1[i] || !st2[i] || cv::norm(bwd[i] - p0[i]) > 2.0) return false;

  const double a0 = polyArea(p0), a1 = polyArea(fwd);
  double move = 0;
  for (int i = 0; i < 4; ++i) move += cv::norm(fwd[i] - p0[i]) / 4;
  if (!cv::isContourConvex(fwd) || a1 < 0.5 * a0 || a1 > 2.0 * a0 || move > 0.6 * std::sqrt(a0))
    return false;

  out.resize(4);
  for (int i = 0; i < 4; ++i) out[i] = fwd[i] + off;
  return true;
}

}  // namespace

SheetTracker::SheetTracker(const CameraParams& cam, const TrackerParams& p)
    : cam_(cam), params_(p), detector_(cam, p) {
  for (auto& f : filters_) f = OneEuroFilter(p.filterMinCutoff, p.filterBeta);
}

void SheetTracker::reset() {
  state_ = TrackState::Lost;
  coast_ = 0;
  raw_.clear();
  smooth_.clear();
  pred_.clear();
  prevGray_.release();
  for (auto& f : filters_) f.reset();
  rvec_.release();
  tvec_.release();
  reprojErr_ = 1e9;
}

bool SheetTracker::process(const cv::Mat& frame, double dt) {
  if (frame.channels() == 3) cv::cvtColor(frame, gray_, cv::COLOR_BGR2GRAY);
  else gray_ = frame;

  pred_.clear();
  if (state_ != TrackState::Lost && raw_.size() == 4) pred_ = raw_;

  Quad det;
  bool found = detector_.detect(gray_, pred_, state_ != TrackState::Lost, det);

  if (!found && state_ != TrackState::Lost && coast_ < params_.maxCoastFrames && !prevGray_.empty() &&
      prevGray_.size() == gray_.size()) {
    Quad fq;
    if (flowQuad(prevGray_, gray_, raw_, fq)) {
      Quad q = fq;
      if (refineQuad(gray_, q, 8)) {
        det = q;
        pred_ = fq;
        found = true;
      }
    }
  }

  cv::Mat rv, tv;
  double err = 1e9;
  if (found) {
    Quad cand;
    if (state_ == TrackState::Lost) {
      found = bestA4Order(det, cam_, cand, rv, tv, err) && err < params_.maxReprojErr * 4.0;
    } else {
      cand = alignToReference(sortClockwise(det), raw_);
      found = estimatePose(cand, cam_, rv, tv, &err) && err < params_.maxReprojErr * 8.0;
      if (found && pred_.size() == 4) {
        double jump = 0.0;
        for (int i = 0; i < 4; ++i) jump += cv::norm(cand[i] - pred_[i]) / 4.0;
        found = jump < 0.6 * std::sqrt(polyArea(cand));
      }
    }
    if (found) det = cand;
  }

  if (found) {
    if (state_ == TrackState::Lost) for (auto& f : filters_) f.reset();
    raw_ = det;
    gray_.copyTo(prevGray_);
    smooth_.resize(4);
    for (int i = 0; i < 4; ++i) {
      smooth_[i].x = float(filters_[2 * i].filter(raw_[i].x, dt));
      smooth_[i].y = float(filters_[2 * i + 1].filter(raw_[i].y, dt));
    }
    cv::Mat rv2, tv2;
    double e2 = 0.0;
    if (estimatePose(smooth_, cam_, rv2, tv2, &e2)) {
      rvec_ = rv2;
      tvec_ = tv2;
    } else {
      rvec_ = rv;
      tvec_ = tv;
    }
    reprojErr_ = err;
    if (state_ == TrackState::Lost) {
      state_ = TrackState::Tracking;
      coast_ = 0;
    } else {
      state_ = TrackState::Tracking;
      coast_ = 0;
    }
  } else if (state_ != TrackState::Lost && ++coast_ <= params_.maxCoastFrames) {
    state_ = TrackState::Coasting;
  } else {
    reset();
  }
  return hasPose();
}
