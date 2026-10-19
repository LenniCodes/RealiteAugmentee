#include "tracker.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/3d.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif

// ---------------------------------------------------------------- One-Euro

double OneEuroFilter::alpha(double cutoff, double dt) {
  double tau = 1.0 / (2 * CV_PI * cutoff);
  return 1.0 / (1.0 + tau / dt);
}

double OneEuroFilter::filter(double x, double dt) {
  if (!init_ || dt <= 0) {
    init_ = true;
    xPrev_ = x;
    dxPrev_ = 0;
    return x;
  }
  double dx = (x - xPrev_) / dt;
  dxPrev_ += alpha(dCutoff_, dt) * (dx - dxPrev_);
  double cutoff = minCutoff_ + beta_ * std::abs(dxPrev_);
  xPrev_ += alpha(cutoff, dt) * (x - xPrev_);
  return xPrev_;
}

// ---------------------------------------------------------------- Tracker

SheetTracker::SheetTracker(const CameraParams& cam, const TrackerParams& p) : cam_(cam), p_(p) {
  for (auto& f : filters_) f = OneEuroFilter(p_.filterMinCutoff, p_.filterBeta);
}

void SheetTracker::reset() {
  state_ = TrackState::Lost;
  coast_ = 0;
  raw_.clear();
  smooth_.clear();
  for (auto& f : filters_) f.reset();
}

// Teste si un quadrilatère est plausible pour une feuille vue en perspective.
static bool plausibleQuad(const std::vector<cv::Point>& q) {
  if (q.size() != 4 || !cv::isContourConvex(q)) return false;
  for (int i = 0; i < 4; ++i) {
    cv::Point2f a = q[(i + 3) % 4] - q[i], b = q[(i + 1) % 4] - q[i];
    double c = a.dot(b) / (cv::norm(a) * cv::norm(b) + 1e-9);
    if (std::abs(c) > 0.85) return false;  // angle < ~32° ou > ~148° : trop dégénéré
  }
  return true;
}

bool SheetTracker::detect(const cv::Mat& gray, std::vector<cv::Point2f>& out) {
  // 1) Zone de recherche : autour du quad précédent si on le suit, sinon toute l'image.
  cv::Rect full(0, 0, gray.cols, gray.rows);
  search_ = full;
  if (state_ != TrackState::Lost && !raw_.empty()) {
    cv::Rect b = cv::boundingRect(raw_);
    int mx = int(b.width * p_.roiMargin), my = int(b.height * p_.roiMargin);
    search_ = cv::Rect(b.x - mx, b.y - my, b.width + 2 * mx, b.height + 2 * my) & full;
  }

  // 2) Sous-résolution : la détection grossière n'a pas besoin de tous les pixels.
  double scale = std::min(1.0, double(p_.procWidth) / search_.width);
  cv::Mat small;
  cv::resize(gray(search_), small, cv::Size(), scale, scale, cv::INTER_AREA);

  // 3) Flou + Otsu : le seuil s'adapte tout seul à l'éclairage (feuille claire / fond).
  cv::GaussianBlur(small, small, cv::Size(5, 5), 0);
  cv::threshold(small, mask_, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  // Ouverture : supprime les petites taches et casse les "ponts" fins qui collent
  // du bruit au bord de la feuille (sinon l'enveloppe convexe n'est plus un quad).
  int k = std::max(3, (p_.openKernel | 1));
  cv::morphologyEx(mask_, mask_, cv::MORPH_OPEN,
                   cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k)));

  // 4) Contours -> quadrilatères candidats.
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  double minArea = p_.minAreaFrac * small.total();
  cv::Point2f prevCenter(-1, -1);
  if (state_ != TrackState::Lost && !raw_.empty()) {
    prevCenter = (raw_[0] + raw_[1] + raw_[2] + raw_[3]) * 0.25f;
  }

  double bestScore = -1;
  std::vector<cv::Point> best;
  int nBig = 0, nQuad = 0;
  for (auto& c : contours) {
    double area = cv::contourArea(c);
    if (area < minArea) continue;
    nBig++;
    std::vector<cv::Point> hull, quad;
    cv::convexHull(c, hull);
    double peri = cv::arcLength(hull, true);
    for (double eps = 0.02; eps <= 0.05 && quad.size() != 4; eps += 0.01)
      cv::approxPolyDP(hull, quad, eps * peri, true);
    if (!plausibleQuad(quad)) continue;
    nQuad++;
    // La feuille remplit bien son quad (un objet arrondi non) :
    if (area / cv::contourArea(quad) < 0.85) continue;

    double score = area;
    if (prevCenter.x >= 0) {  // en suivi : on préfère le candidat proche du précédent
      cv::Moments m = cv::moments(quad);
      cv::Point2f c0(float(m.m10 / m.m00 / scale + search_.x), float(m.m01 / m.m00 / scale + search_.y));
      score /= 1.0 + cv::norm(c0 - prevCenter) / 50.0;
    }
    if (score > bestScore) { bestScore = score; best = quad; }
  }
  info_ = cv::format("%d contours, %d assez grands, %d quads", int(contours.size()), nBig, nQuad);
  if (best.empty()) { if (nQuad) info_ += " (mal remplis)"; return false; }

  // 5) Retour en pleine résolution + raffinement sub-pixel.
  out.clear();
  for (auto& p : best)
    out.emplace_back(float(p.x / scale + search_.x), float(p.y / scale + search_.y));
  if (p_.subPix) {
    int win = std::max(7, std::min(11, int(std::round(3.0 / scale))));
    cv::cornerSubPix(gray, out, cv::Size(win, win), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 20, 0.03));
  }
  return true;
}

// En suivi, on choisit le décalage circulaire qui colle le mieux aux coins
// précédents : chaque coin garde son identité, la pose ne saute pas de 90/180°.
std::vector<cv::Point2f> SheetTracker::matchOrder(const std::vector<cv::Point2f>& o) const {
  if (raw_.size() != 4) return o;
  int bestShift = 0;
  double bestD = 1e18;
  for (int s = 0; s < 4; ++s) {
    double d = 0;
    for (int i = 0; i < 4; ++i) d += cv::norm(o[(i + s) % 4] - raw_[i]);
    if (d < bestD) { bestD = d; bestShift = s; }
  }
  std::vector<cv::Point2f> r(4);
  for (int i = 0; i < 4; ++i) r[i] = o[(i + bestShift) % 4];
  return r;
}

bool SheetTracker::process(const cv::Mat& frame, double dt) {
  if (frame.channels() == 3) cv::cvtColor(frame, gray_, cv::COLOR_BGR2GRAY);
  else gray_ = frame;

  std::vector<cv::Point2f> det;
  bool found = detect(gray_, det);

  cv::Mat rv, tv;
  double err = 1e9;
  if (found) {
    std::vector<cv::Point> q(det.begin(), det.end());
    std::vector<cv::Point2f> ordered = orderCorners(q);  // ordre "grand côté d'abord"
    // orderCorners arrondit en entier : on récupère les coins sub-pixel correspondants
    for (auto& p : ordered) {
      auto it = std::min_element(det.begin(), det.end(), [&](auto& a, auto& b) {
        return cv::norm(a - p) < cv::norm(b - p);
      });
      p = *it;
    }
    if (state_ != TrackState::Lost) ordered = matchOrder(ordered);

    // Validation : la forme doit être celle d'une A4 vue en perspective.
    // Seuil donné pour du 720p : on l'adapte à la résolution de la source.
    double maxErr = p_.maxReprojErr * std::max(1.0, std::max(gray_.cols, gray_.rows) / 1280.0);
    found = estimatePose(ordered, cam_, rv, tv, &err) && err < maxErr;
    info_ += cv::format(", reproj %.1f px (max %.1f)%s", err > 1e8 ? -1.0 : err, maxErr, found ? "" : " -> REJETE");
    if (found) det = ordered;
  }

  if (found) {
    if (state_ == TrackState::Lost) for (auto& f : filters_) f.reset();
    raw_ = det;
    smooth_.resize(4);
    for (int i = 0; i < 4; ++i) {
      smooth_[i].x = float(filters_[2 * i].filter(raw_[i].x, dt));
      smooth_[i].y = float(filters_[2 * i + 1].filter(raw_[i].y, dt));
    }
    // Pose finale sur les coins lissés (pas de tremblement du contenu 3D).
    double e2;
    if (estimatePose(smooth_, cam_, rv, tv, &e2)) { rvec_ = rv; tvec_ = tv; }
    reprojErr_ = err;
    state_ = TrackState::Tracking;
    coast_ = 0;
  } else if (state_ != TrackState::Lost && ++coast_ <= p_.maxCoastFrames) {
    state_ = TrackState::Coasting;  // perte courte : on garde la dernière pose
  } else {
    reset();
  }
  return hasPose();
}
