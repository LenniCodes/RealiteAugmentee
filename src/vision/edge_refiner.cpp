#include "vision/edge_refiner.hpp"

#include "core/quad_utils.hpp"

#include <algorithm>
#include <array>

#include <opencv2/imgproc.hpp>

namespace {

static inline float sampleBilinear(const cv::Mat& g, float x, float y) {
  const int x0 = int(std::floor(x));
  const int y0 = int(std::floor(y));
  const float fx = x - x0;
  const float fy = y - y0;
  const uchar* p0 = g.ptr<uchar>(y0);
  const uchar* p1 = g.ptr<uchar>(y0 + 1);
  return (1 - fy) * ((1 - fx) * p0[x0] + fx * p0[x0 + 1]) +
         fy * ((1 - fx) * p1[x0] + fx * p1[x0 + 1]);
}

static bool refineQuadEdges(const cv::Mat& gray, Quad& q, int R) {
  if (q.size() != 4) return false;
  const cv::Point2f ctr = (q[0] + q[1] + q[2] + q[3]) * 0.25f;
  const float size = float(std::sqrt(polyArea(q)));
  const auto inside = [&](const cv::Point2f& p) {
    return p.x >= 0 && p.y >= 0 && p.x < gray.cols - 1 && p.y < gray.rows - 1;
  };

  std::array<cv::Vec4f, 4> lines;
  std::vector<float> prof(2 * R + 1);
  for (int i = 0; i < 4; ++i) {
    const cv::Point2f a = q[i];
    const cv::Point2f b = q[(i + 1) % 4];
    const cv::Point2f d = b - a;
    const float len = float(cv::norm(d));
    if (len < 20.f) return false;
    const cv::Point2f dir = d * (1.f / len);
    cv::Point2f nrm(-dir.y, dir.x);
    if (nrm.dot((a + b) * 0.5f - ctr) < 0) nrm = -nrm;

    const int n = std::clamp(int(len / 6.f), 10, 40);
    std::vector<cv::Point2f> pts;
    std::vector<int> signs;
    for (int s = 0; s < n; ++s) {
      const float t = 0.12f + 0.76f * (s + 0.5f) / n;
      const cv::Point2f p = a + d * t;
      if (!inside(p - nrm * float(R + 1)) || !inside(p + nrm * float(R + 1))) continue;

      for (int k = -R; k <= R; ++k) {
        const cv::Point2f pk = p + nrm * float(k);
        prof[k + R] = sampleBilinear(gray, pk.x, pk.y);
      }
      float bestG = 0;
      int bestI = -1;
      for (int j = 1; j < 2 * R; ++j) {
        const float g = prof[j + 1] - prof[j - 1];
        if (std::abs(g) > std::abs(bestG)) {
          bestG = g;
          bestI = j;
        }
      }
      if (bestI < 0 || std::abs(bestG) < 12.f) continue;

      float off = float(bestI - R);
      if (bestI >= 2 && bestI <= 2 * R - 2) {
        const float gm = std::abs(prof[bestI] - prof[bestI - 2]);
        const float g0 = std::abs(bestG);
        const float gp = std::abs(prof[bestI + 2] - prof[bestI]);
        const float den = gm - 2 * g0 + gp;
        if (std::abs(den) > 1e-3f) off += std::clamp(0.5f * (gm - gp) / den, -0.5f, 0.5f);
      }
      pts.push_back(p + nrm * off);
      signs.push_back(bestG > 0 ? 1 : -1);
    }

    const int pos = int(std::count(signs.begin(), signs.end(), 1));
    const int keep = (pos * 2 >= int(signs.size())) ? 1 : -1;
    std::vector<cv::Point2f> good;
    for (size_t j = 0; j < pts.size(); ++j) if (signs[j] == keep) good.push_back(pts[j]);
    if (good.size() < std::max<size_t>(6, size_t(0.4 * n))) return false;

    cv::Vec4f l;
    cv::fitLine(good, l, cv::DIST_HUBER, 0, 0.01, 0.01);
    std::vector<cv::Point2f> inl;
    for (const auto& g : good) {
      const float dist = std::abs((g.x - l[2]) * l[1] - (g.y - l[3]) * l[0]);
      if (dist < 1.5f) inl.push_back(g);
    }
    if (inl.size() >= 6) cv::fitLine(inl, l, cv::DIST_L2, 0, 0.01, 0.01);
    lines[i] = l;
  }

  Quad nq(4);
  const float maxShift = 0.15f * size + float(R) + 2.f;
  for (int i = 0; i < 4; ++i) {
    const cv::Vec4f& l1 = lines[(i + 3) % 4];
    const cv::Vec4f& l2 = lines[i];
    const cv::Point2f d1(l1[0], l1[1]), p1(l1[2], l1[3]);
    const cv::Point2f d2(l2[0], l2[1]), p2(l2[2], l2[3]);
    const float den = d1.x * d2.y - d1.y * d2.x;
    if (std::abs(den) < 0.2f) return false;
    const cv::Point2f w = p2 - p1;
    const float t = (w.x * d2.y - w.y * d2.x) / den;
    nq[i] = p1 + d1 * t;
    if (cv::norm(nq[i] - q[i]) > maxShift) return false;
  }
  q = nq;
  return true;
}

}  // namespace

bool refineQuad(const cv::Mat& gray, Quad& q, int searchRadius) {
  if (!refineQuadEdges(gray, q, searchRadius)) return false;
  Quad q2 = q;
  if (refineQuadEdges(gray, q2, std::max(2, searchRadius / 2))) q = q2;
  return true;
}
