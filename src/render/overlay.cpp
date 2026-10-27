#include "render/overlay.hpp"

#include <opencv2/imgproc.hpp>

#if CV_VERSION_MAJOR >= 5
#include <opencv2/3d.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif

void drawCube(cv::Mat& img, const CameraParams& cam, const cv::Mat& rvec,
              const cv::Mat& tvec, float size) {
  float h = size / 2.f;
  std::vector<cv::Point3f> cube = {
      {-h, -h, 0}, {h, -h, 0}, {h, h, 0}, {-h, h, 0},
      {-h, -h, size}, {h, -h, size}, {h, h, size}, {-h, h, size},
  };
  std::vector<cv::Point2f> p;
  cv::projectPoints(cube, rvec, tvec, cam.K, cam.dist, p);

  const cv::Scalar base(0, 200, 0), top(255, 100, 0), pillar(0, 200, 255);
  for (int i = 0; i < 4; ++i) {
    cv::line(img, p[i], p[(i + 1) % 4], base, 2, cv::LINE_AA);
    cv::line(img, p[i + 4], p[(i + 1) % 4 + 4], top, 2, cv::LINE_AA);
    cv::line(img, p[i], p[i + 4], pillar, 2, cv::LINE_AA);
  }
  cv::drawFrameAxes(img, cam.K, cam.dist, rvec, tvec, size * 0.8f, 3);
}

void drawSheetOutline(cv::Mat& img, const Quad& corners) {
  if (corners.size() != 4) return;
  for (int i = 0; i < 4; ++i) {
    cv::line(img, corners[i], corners[(i + 1) % 4], {0, 255, 0}, 2, cv::LINE_AA);
    cv::putText(img, std::to_string(i), corners[i] + cv::Point2f(8, -8),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 0, 255}, 2);
  }
}

void drawHud(cv::Mat& img, TrackState state, double algoMs, double fps,
             double reprojErr) {
  const char* st = state == TrackState::Tracking ? "TRACKING"
                 : state == TrackState::Coasting ? "COASTING" : "LOST";
  cv::putText(img, cv::format("%s  algo %.1f ms  affichage %.0f fps  reproj %.2f px",
                             st, algoMs, fps, reprojErr),
              {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 255}, 2);
}
