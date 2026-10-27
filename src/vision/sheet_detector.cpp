#include "vision/sheet_detector.hpp"

#include "core/quad_utils.hpp"
#include "geometry/pose_estimator.hpp"
#include "vision/edge_refiner.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <vector>

namespace {

bool validRect(const cv::Rect& r, int cols, int rows) {
  return r.x >= 0 && r.y >= 0 && r.width > 0 && r.height > 0 &&
         r.x + r.width <= cols && r.y + r.height <= rows;
}

struct Candidate {
  Quad q;
  double rough = 0.0;
};

bool plausibleQuad(const std::vector<cv::Point>& q) {
  if (q.size() != 4 || !cv::isContourConvex(q)) return false;
  for (int i = 0; i < 4; ++i) {
    cv::Point2f a = q[(i + 3) % 4] - q[i];
    cv::Point2f b = q[(i + 1) % 4] - q[i];
    double c = a.dot(b) / (cv::norm(a) * cv::norm(b) + 1e-9);
    if (std::abs(c) > 0.85) return false;
  }
  return true;
}

cv::Mat buildMask(const cv::Mat& gray, const cv::Rect& zone, const Quad& reference,
                  const TrackerParams& params, double scale, bool haveRef,
                  double& otsuThreshold) {
  cv::Mat small = gray;
  cv::Mat mask;
  if (haveRef) {
    Quad localRef;
    for (const auto& p : reference) {
      localRef.push_back(cv::Point2f(p.x - zone.x, p.y - zone.y));
    }
    cv::Point2f c(0, 0);
    for (const auto& p : localRef) c += p;
    c *= 0.25f;
    std::vector<cv::Point> pin, pout;
    for (const auto& p : localRef) {
      pin.push_back(cv::Point(cvRound(p.x * scale), cvRound(p.y * scale)));
      pout.push_back(cv::Point(cvRound(p.x * scale), cvRound(p.y * scale)));
    }
    for (const auto& p : localRef) {
      pin.push_back(cv::Point(cvRound((c.x + (p.x - c.x) * 0.6f) * scale),
                              cvRound((c.y + (p.y - c.y) * 0.6f) * scale)));
      pout.push_back(cv::Point(cvRound((c.x + (p.x - c.x) * 1.12f) * scale),
                              cvRound((c.y + (p.y - c.y) * 1.12f) * scale)));
    }
    cv::Mat in = cv::Mat::zeros(small.size(), CV_8U);
    cv::Mat ring(small.size(), CV_8U, cv::Scalar(255));
    cv::fillConvexPoly(in, pin, cv::Scalar(255));
    cv::fillConvexPoly(ring, pout, cv::Scalar(0));
    if (cv::countNonZero(in) <= 50 || cv::countNonZero(ring) <= 50) return mask;
    const double mi = cv::mean(small, in)[0], mo = cv::mean(small, ring)[0];
    if (std::abs(mi - mo) <= 12) return mask;
    cv::threshold(small, mask, (mi + mo) / 2, 255,
                 mi > mo ? cv::THRESH_BINARY : cv::THRESH_BINARY_INV);
    return mask;
  }

  otsuThreshold = cv::threshold(small, mask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  return mask;
}

std::vector<Candidate> collectCandidates(const cv::Mat& gray, const cv::Rect& zone,
                                        const cv::Mat& mask, const Quad& reference,
                                        const TrackerParams& params, double scale,
                                        bool haveRef, double frameArea) {
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(std::max(3, (params.openKernel | 1)),
                                                                            std::max(3, (params.openKernel | 1))));
  cv::Mat opened;
  cv::morphologyEx(mask, opened, cv::MORPH_OPEN, kernel);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(opened, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  const double minArea = params.minAreaFrac * opened.total();

  std::vector<Candidate> cands;
  for (const auto& c : contours) {
    const double area = cv::contourArea(c);
    if (area < minArea) continue;
    cv::Rect br = cv::boundingRect(c);
    if (br.x <= 1 || br.y <= 1 || br.br().x >= opened.cols - 1 || br.br().y >= opened.rows - 1) continue;

    std::vector<cv::Point> hull, quad;
    cv::convexHull(c, hull);
    const double peri = cv::arcLength(hull, true);
    for (double eps = 0.02; eps <= 0.061 && quad.size() != 4; eps += 0.01)
      cv::approxPolyDP(hull, quad, eps * peri, true);
    if (!plausibleQuad(quad)) continue;
    const double quadArea = cv::contourArea(quad);
    if (quadArea < 1e-6 || area / quadArea < 0.8) continue;

    Candidate cd;
    for (const auto& p : quad) {
      cd.q.push_back(cv::Point2f(float((p.x + 0.5) / scale - 0.5 + zone.x),
                                float((p.y + 0.5) / scale - 0.5 + zone.y)));
    }
    cd.rough = polyArea(cd.q) / frameArea;
    if (haveRef) cd.rough /= 1.0 + 5.0 * quadDist(cd.q, reference) / std::sqrt(polyArea(reference) + 1.0);
    cands.push_back(cd);
  }
  return cands;
}

bool pickBest(const std::vector<Candidate>& cands, const cv::Mat& gray,
              const CameraParams& cam, const TrackerParams& params, bool haveRef,
              double gate, Quad& detected) {
  if (cands.empty()) return false;
  std::vector<Candidate> sorted = cands;
  std::sort(sorted.begin(), sorted.end(), [](const Candidate& a, const Candidate& b) {
    return a.rough > b.rough;
  });
  if (sorted.size() > 3) sorted.resize(3);

  double bestScore = -1.0;
  detected.clear();
  for (const auto& cd : sorted) {
    Quad q = cd.q;
    if (params.subPix) refineQuad(gray, q, std::clamp(int(std::ceil(2.5 / 1.0)) + 2, 4, 16));
    Quad ord;
    cv::Mat rv, tv;
    double err = 1e9;
    if (!bestA4Order(q, cam, ord, rv, tv, err) || err > gate) continue;
    const double score = cd.rough / (1.0 + err / std::max(0.5, params.maxReprojErr));
    if (score > bestScore) {
      bestScore = score;
      detected = q;
    }
  }
  return bestScore >= 0.0;
}

}  // namespace

SheetDetector::SheetDetector(const CameraParams& cam, const TrackerParams& p)
    : cam_(cam), params_(p) {}

bool SheetDetector::detect(const cv::Mat& gray, const Quad& reference, bool allowFullFrame,
                          Quad& out) {
  const cv::Rect full(0, 0, gray.cols, gray.rows);
  const bool haveRef = !reference.empty();
  cv::Rect roi = full;
  if (haveRef) {
    cv::Rect b = cv::boundingRect(reference);
    int mx = int(b.width * params_.roiMargin);
    int my = int(b.height * params_.roiMargin);
    roi = cv::Rect(b.x - mx, b.y - my, b.width + 2 * mx, b.height + 2 * my) & full;
    if (roi.area() < 400) roi = full;
  }

  auto run = [&](const cv::Rect& zone) -> bool {
    if (!validRect(zone, gray.cols, gray.rows)) return false;
    const double scale = std::min(1.0, double(params_.procWidth) / std::max(1, zone.width));
    cv::Mat roiImage;
    cv::resize(gray(zone), roiImage, cv::Size(), scale, scale, cv::INTER_AREA);
    cv::GaussianBlur(roiImage, roiImage, cv::Size(3, 3), 0);

    double otsuThreshold = -1.0;
    cv::Mat mask = buildMask(roiImage, zone, reference, params_, scale, haveRef, otsuThreshold);
    if (mask.empty()) return false;

    cv::Mat opened;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                              cv::Size(std::max(3, (params_.openKernel | 1)),
                                                       std::max(3, (params_.openKernel | 1))));
    cv::morphologyEx(mask, opened, cv::MORPH_OPEN, kernel);

    std::vector<Candidate> cands = collectCandidates(gray, zone, opened, reference, params_, scale,
                                                    haveRef, double(gray.total()));
    if (cands.empty()) return false;

    Quad detected;
    const double gate = params_.maxReprojErr * (haveRef ? 8.0 : 4.0);
    if (!pickBest(cands, gray, cam_, params_, haveRef, gate, detected)) return false;

    out = detected;
    searchRoi_ = zone;
    debugMask_ = opened;
    return true;
  };

  if (run(roi)) return true;
  if (allowFullFrame && roi != full && run(full)) return true;
  searchRoi_ = roi;
  debugMask_.release();
  return false;
}
