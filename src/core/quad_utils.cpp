#include "core/quad_utils.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

double polyArea(const Quad& q) {
  if (q.size() < 3) return 0.0;
  return std::abs(cv::contourArea(q));
}

double quadDist(const Quad& a, const Quad& b) {
  if (a.size() != 4 || b.size() != 4) return 1e18;
  double best = 1e18;
  for (int dir = 0; dir < 2; ++dir) {
    for (int s = 0; s < 4; ++s) {
      double d = 0.0;
      for (int i = 0; i < 4; ++i) {
        int j = dir ? ((s - i) % 4 + 4) % 4 : (s + i) % 4;
        d += cv::norm(a[i] - b[j]);
      }
      best = std::min(best, d / 4.0);
    }
  }
  return best;
}

Quad sortClockwise(Quad pts) {
  if (pts.size() != 4) return pts;
  cv::Point2f c(0, 0);
  for (const auto& p : pts) c += p;
  c *= 0.25f;
  std::sort(pts.begin(), pts.end(), [&](const cv::Point2f& a, const cv::Point2f& b) {
    return std::atan2(a.y - c.y, a.x - c.x) < std::atan2(b.y - c.y, b.x - c.x);
  });
  return pts;
}

Quad alignToReference(const Quad& candidate, const Quad& reference) {
  if (candidate.size() != 4 || reference.size() != 4) return candidate;
  int bestShift = 0;
  double bestDist = 1e18;
  for (int s = 0; s < 4; ++s) {
    double d = 0.0;
    for (int i = 0; i < 4; ++i) d += cv::norm(candidate[(i + s) % 4] - reference[i]);
    if (d < bestDist) {
      bestDist = d;
      bestShift = s;
    }
  }
  Quad aligned(4);
  for (int i = 0; i < 4; ++i) aligned[i] = candidate[(i + bestShift) % 4];
  return aligned;
}
