#include "geometry/pose_estimator.hpp"

#include "core/quad_utils.hpp"

#include <array>

#if CV_VERSION_MAJOR >= 5
#include <opencv2/3d.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif

std::vector<cv::Point3f> a4ObjectPoints(float w, float h) {
  return {
      {-w / 2,  h / 2, 0},
      { w / 2,  h / 2, 0},
      { w / 2, -h / 2, 0},
      {-w / 2, -h / 2, 0},
  };
}

bool estimatePose(const Quad& imgPts, const CameraParams& cam, cv::Mat& rvec,
                  cv::Mat& tvec, double* reprojErr) {
  if (imgPts.size() != 4) return false;
  std::vector<cv::Point3f> objPts = a4ObjectPoints();
  bool ok = cv::solvePnP(objPts, imgPts, cam.K, cam.dist, rvec, tvec, false,
                         cv::SOLVEPNP_IPPE);
  if (!ok) return false;

  if (reprojErr) {
    std::vector<cv::Point2f> proj;
    cv::projectPoints(objPts, rvec, tvec, cam.K, cam.dist, proj);
    double e = 0.0;
    for (size_t i = 0; i < proj.size(); ++i) e += cv::norm(proj[i] - imgPts[i]);
    *reprojErr = e / proj.size();
  }
  return tvec.at<double>(2) > 0;
}

bool bestA4Order(const Quad& q, const CameraParams& cam, Quad& order,
                 cv::Mat& rvec, cv::Mat& tvec, double& err) {
  const Quad cw = sortClockwise(q);
  std::array<double, 4> e{};
  std::array<cv::Mat, 4> r, t;
  double best = 1e18;
  for (int s = 0; s < 4; ++s) {
    Quad c(4);
    for (int i = 0; i < 4; ++i) c[i] = cw[(i + s) % 4];
    double ee = 1e18;
    e[s] = estimatePose(c, cam, r[s], t[s], &ee) ? ee : 1e18;
    best = std::min(best, e[s]);
  }
  if (best >= 1e17) return false;

  int pick = -1;
  for (int s = 0; s < 4; ++s) {
    if (e[s] > best + 0.5) continue;
    if (pick < 0 || cw[s].x + cw[s].y < cw[pick].x + cw[pick].y) pick = s;
  }

  order.resize(4);
  for (int i = 0; i < 4; ++i) order[i] = cw[(i + pick) % 4];
  rvec = r[pick];
  tvec = t[pick];
  err = e[pick];
  return true;
}
