#include "tracker.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp> 
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

// ---------------------------------------------------------------- Helpers

static double polyArea(const std::vector<cv::Point2f>& q) { return std::abs(cv::contourArea(q)); }

// Distance moyenne entre deux quads, minimisée sur les décalages et le sens de parcours.
static double quadDist(const std::vector<cv::Point2f>& a, const std::vector<cv::Point2f>& b) {
  double best = 1e18;
  for (int dir = 0; dir < 2; ++dir)
    for (int s = 0; s < 4; ++s) {
      double d = 0;
      for (int i = 0; i < 4; ++i) {
        int j = dir ? ((s - i) % 4 + 4) % 4 : (s + i) % 4;
        d += cv::norm(a[i] - b[j]);
      }
      best = std::min(best, d / 4);
    }
  return best;
}

// Tri horaire à l'écran (y vers le bas), en flottants : pas d'arrondi entier.
static std::vector<cv::Point2f> sortClockwise(std::vector<cv::Point2f> pts) {
  cv::Point2f c(0, 0);
  for (auto& p : pts) c += p;
  c *= 0.25f;
  std::sort(pts.begin(), pts.end(), [&](const cv::Point2f& a, const cv::Point2f& b) {
    return std::atan2(a.y - c.y, a.x - c.x) < std::atan2(b.y - c.y, b.x - c.x);
  });
  return pts;
}

// Teste si un quadrilatère est plausible pour une feuille vue en perspective.
static bool plausibleQuad(const std::vector<cv::Point>& q) {
  if (q.size() != 4 || !cv::isContourConvex(q)) return false;
  for (int i = 0; i < 4; ++i) {
    cv::Point2f a = q[(i + 3) % 4] - q[i], b = q[(i + 1) % 4] - q[i];
    double c = a.dot(b) / (cv::norm(a) * cv::norm(b) + 1e-9);
    if (std::abs(c) > 0.85) return false;
  }
  return true;
}

// Ordre horaire uniquement (le sens inverse = voir le dos de la feuille -> retournements de
// pose), puis meilleur décalage cyclique. L'erreur de reprojection mesure à quel point le quad
// ressemble à une A4 vue en perspective : excellent test pour rejeter les faux candidats.
static bool bestA4Order(const std::vector<cv::Point2f>& q, const CameraParams& cam,
                        std::vector<cv::Point2f>& order, cv::Mat& rv, cv::Mat& tv, double& err) {
  const std::vector<cv::Point2f> cw = sortClockwise(q);
  std::array<double, 4> e;
  std::array<cv::Mat, 4> r, t;
  double best = 1e18;
  for (int s = 0; s < 4; ++s) {
    std::vector<cv::Point2f> c(4);
    for (int i = 0; i < 4; ++i) c[i] = cw[(i + s) % 4];
    double ee = 1e18;
    e[s] = estimatePose(c, cam, r[s], t[s], &ee) ? ee : 1e18;
    best = std::min(best, e[s]);
  }
  if (best >= 1e17) return false;
  // A4 symétrique à 180° : parmi les ordres quasi équivalents, coin 0 le plus en haut à gauche.
  int pick = -1;
  for (int s = 0; s < 4; ++s) {
    if (e[s] > best + 0.5) continue;
    if (pick < 0 || cw[s].x + cw[s].y < cw[pick].x + cw[pick].y) pick = s;
  }
  order.resize(4);
  for (int i = 0; i < 4; ++i) order[i] = cw[(i + pick) % 4];
  rv = r[pick];
  tv = t[pick];
  err = e[pick];
  return true;
}

static inline float sampleBilinear(const cv::Mat& g, float x, float y) {
  const int x0 = int(std::floor(x)), y0 = int(std::floor(y));
  const float fx = x - x0, fy = y - y0;
  const uchar* p0 = g.ptr<uchar>(y0);
  const uchar* p1 = g.ptr<uchar>(y0 + 1);
  return (1 - fy) * ((1 - fx) * p0[x0] + fx * p0[x0 + 1]) + fy * ((1 - fx) * p1[x0] + fx * p1[x0 + 1]);
}

// Affine les coins à partir des BORDS réels de la feuille : maximum de gradient sous-pixel le
// long de la normale de chaque côté, droite robuste (Huber puis L2 sur inliers), intersections.
// Retourne false (q inchangé) si un côté n'est pas assez soutenu par des contours nets.
static bool refineQuadEdges(const cv::Mat& gray, std::vector<cv::Point2f>& q, int R) {
  if (q.size() != 4) return false;
  const cv::Point2f ctr = (q[0] + q[1] + q[2] + q[3]) * 0.25f;
  const float size = float(std::sqrt(polyArea(q)));
  const auto inside = [&](const cv::Point2f& p) {
    return p.x >= 0 && p.y >= 0 && p.x < gray.cols - 1 && p.y < gray.rows - 1;
  };

  std::array<cv::Vec4f, 4> lines;
  std::vector<float> prof(2 * R + 1);
  for (int i = 0; i < 4; ++i) {
    const cv::Point2f a = q[i], b = q[(i + 1) % 4];
    const cv::Point2f d = b - a;
    const float len = float(cv::norm(d));
    if (len < 20.f) return false;
    const cv::Point2f dir = d * (1.f / len);
    cv::Point2f nrm(-dir.y, dir.x);
    if (nrm.dot((a + b) * 0.5f - ctr) < 0) nrm = -nrm;  // normale vers l'extérieur

    const int n = std::clamp(int(len / 6.f), 10, 40);
    std::vector<cv::Point2f> pts;
    std::vector<int> signs;
    for (int s = 0; s < n; ++s) {
      const float t = 0.12f + 0.76f * (s + 0.5f) / n;  // on évite les coins
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
        if (std::abs(g) > std::abs(bestG)) { bestG = g; bestI = j; }
      }
      if (bestI < 0 || std::abs(bestG) < 12.f) continue;  // pas de contour net ici

      float off = float(bestI - R);
      if (bestI >= 2 && bestI <= 2 * R - 2) {  // interpolation parabolique du pic
        const float gm = std::abs(prof[bestI] - prof[bestI - 2]);
        const float g0 = std::abs(bestG);
        const float gp = std::abs(prof[bestI + 2] - prof[bestI]);
        const float den = gm - 2 * g0 + gp;
        if (std::abs(den) > 1e-3f) off += std::clamp(0.5f * (gm - gp) / den, -0.5f, 0.5f);
      }
      pts.push_back(p + nrm * off);
      signs.push_back(bestG > 0 ? 1 : -1);
    }

    // Polarité cohérente (feuille claire sur fond sombre ou l'inverse) : on garde la majorité.
    const int pos = int(std::count(signs.begin(), signs.end(), 1));
    const int keep = (pos * 2 >= int(signs.size())) ? 1 : -1;
    std::vector<cv::Point2f> good;
    for (size_t j = 0; j < pts.size(); ++j)
      if (signs[j] == keep) good.push_back(pts[j]);
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

  // Le coin i est l'intersection du côté i-1 et du côté i.
  std::vector<cv::Point2f> nq(4);
  const float maxShift = 0.15f * size + float(R) + 2.f;
  for (int i = 0; i < 4; ++i) {
    const cv::Vec4f& l1 = lines[(i + 3) % 4];
    const cv::Vec4f& l2 = lines[i];
    const cv::Point2f d1(l1[0], l1[1]), p1(l1[2], l1[3]);
    const cv::Point2f d2(l2[0], l2[1]), p2(l2[2], l2[3]);
    const float den = d1.x * d2.y - d1.y * d2.x;
    if (std::abs(den) < 0.2f) return false;  // côtés quasi parallèles
    const cv::Point2f w = p2 - p1;
    const float t = (w.x * d2.y - w.y * d2.x) / den;
    nq[i] = p1 + d1 * t;
    if (cv::norm(nq[i] - q[i]) > maxShift) return false;
  }
  q = nq;
  return true;
}

// Deux passes : une large, puis une serrée autour du résultat.
static bool refineQuad(const cv::Mat& gray, std::vector<cv::Point2f>& q, int R) {
  if (!refineQuadEdges(gray, q, R)) return false;
  std::vector<cv::Point2f> q2 = q;
  if (refineQuadEdges(gray, q2, std::max(2, R / 2))) q = q2;
  return true;
}

// Flot optique LK des 4 coins entre l'image de référence et l'image courante, avec contrôle
// aller-retour. Calculé sur un RECADRAGE autour de la feuille (2x sa boîte englobante) : la
// pyramide LK n'est construite que sur ce crop, pas sur toute l'image.
static bool flowQuad(const cv::Mat& prev, const cv::Mat& cur, const std::vector<cv::Point2f>& quad,
                     std::vector<cv::Point2f>& out) {
  if (quad.size() != 4) return false;
  const cv::Rect full(0, 0, cur.cols, cur.rows);
  const cv::Rect b = cv::boundingRect(quad);
  const cv::Rect crop = cv::Rect(b.x - b.width / 2, b.y - b.height / 2, b.width * 2, b.height * 2) & full;
  if (crop.width < 64 || crop.height < 64) return false;

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
  for (int i = 0; i < 4; ++i)
    if (!st1[i] || !st2[i] || cv::norm(bwd[i] - p0[i]) > 2.0) return false;

  const double a0 = polyArea(p0), a1 = polyArea(fwd);
  double move = 0;
  for (int i = 0; i < 4; ++i) move += cv::norm(fwd[i] - p0[i]) / 4;
  if (!cv::isContourConvex(fwd) || a1 < 0.5 * a0 || a1 > 2.0 * a0 || move > 0.6 * std::sqrt(a0))
    return false;

  out.resize(4);
  for (int i = 0; i < 4; ++i) out[i] = fwd[i] + off;
  return true;
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
  pred_.clear();
  prevGray_.release();
  for (auto& f : filters_) f.reset();
}

bool SheetTracker::detect(const cv::Mat& gray, std::vector<cv::Point2f>& out) {
  const cv::Rect full(0, 0, gray.cols, gray.rows);

  // 1) Zone de recherche : autour du quad précédent (pred_) si on suit, sinon toute l'image.
  const bool haveRef = state_ != TrackState::Lost && pred_.size() == 4;
  cv::Rect roi = full;
  if (haveRef) {
    cv::Rect b = cv::boundingRect(pred_);
    int mx = int(b.width * p_.roiMargin), my = int(b.height * p_.roiMargin);
    roi = cv::Rect(b.x - mx, b.y - my, b.width + 2 * mx, b.height + 2 * my) & full;
    if (roi.area() < 400) roi = full;
  }

  struct Cand {
    std::vector<cv::Point2f> q;
    double rough;
  };

  auto run = [&](const cv::Rect& zone, std::vector<cv::Point2f>& detected,
                 cv::Mat& debugMask) -> bool {
    // 2) Sous-résolution : la précision vient de l'affinage sur les bords en pleine résolution,
    //    donc procWidth peut être petit (240-320) sans perdre en précision.
    const double scale = std::min(1.0, double(p_.procWidth) / std::max(1, zone.width));
    cv::Mat small;
    cv::resize(gray(zone), small, cv::Size(), scale, scale, cv::INTER_AREA);
    cv::GaussianBlur(small, small, cv::Size(3, 3), 0);

    const auto toFull = [&](const cv::Point& p) {  // centre de pixel -> repère image complète
      return cv::Point2f(float((p.x + 0.5) / scale - 0.5 + zone.x),
                         float((p.y + 0.5) / scale - 0.5 + zone.y));
    };
    const auto toSmall = [&](const cv::Point2f& p) {
      return cv::Point(cvRound((p.x - zone.x) * scale), cvRound((p.y - zone.y) * scale));
    };

    const int k = std::max(3, (p_.openKernel | 1));
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    const double frameArea = double(gray.total());
    const int R = std::clamp(int(std::ceil(2.5 / scale)) + 2, 4, 16);
    const double gate = p_.maxReprojErr * (haveRef ? 8.0 : 4.0);
    double otsuT = -1;

    // 3) Masques essayés UN PAR UN, avec sortie dès qu'un candidat valide est trouvé :
    //    0 = seuil "appris" (suivi seulement), 1 = Otsu, 2 = Otsu inversé.
    //    Dans le cas courant (suivi), un seul masque est calculé.
    for (int kind = haveRef ? 0 : 1; kind < 3; ++kind) {
      cv::Mat mask;
      if (kind == 0) {
        // Seuil à mi-chemin entre l'intérieur de la feuille et l'anneau de fond autour.
        cv::Point2f c(0, 0);
        for (const auto& p : pred_) c += p;
        c *= 0.25f;
        std::vector<cv::Point> pin, pout;
        for (const auto& p : pred_) {
          pin.push_back(toSmall(c + (p - c) * 0.6f));
          pout.push_back(toSmall(c + (p - c) * 1.12f));
        }
        cv::Mat in = cv::Mat::zeros(small.size(), CV_8U);
        cv::Mat ring(small.size(), CV_8U, cv::Scalar(255));
        cv::fillConvexPoly(in, pin, cv::Scalar(255));
        cv::fillConvexPoly(ring, pout, cv::Scalar(0));
        if (cv::countNonZero(in) <= 50 || cv::countNonZero(ring) <= 50) continue;
        const double mi = cv::mean(small, in)[0], mo = cv::mean(small, ring)[0];
        if (std::abs(mi - mo) <= 12) continue;
        cv::threshold(small, mask, (mi + mo) / 2, 255,
                      mi > mo ? cv::THRESH_BINARY : cv::THRESH_BINARY_INV);
      } else if (kind == 1) {
        otsuT = cv::threshold(small, mask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
      } else {
        // Otsu inversé SANS recalculer l'histogramme : on réutilise le seuil de l'étape 1.
        if (otsuT < 0) otsuT = cv::threshold(small, mask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::compare(small, otsuT, mask, cv::CMP_LE);
      }

      cv::Mat opened;
      cv::morphologyEx(mask, opened, cv::MORPH_OPEN, kernel);
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(opened, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      const double minArea = p_.minAreaFrac * opened.total();

      std::vector<Cand> cands;
      for (const auto& c : contours) {
        const double area = cv::contourArea(c);
        if (area < minArea) continue;
        // Région collée au bord de la zone = le fond (ou une feuille coupée), pas une feuille.
        const cv::Rect br = cv::boundingRect(c);
        if (br.x <= 1 || br.y <= 1 || br.br().x >= opened.cols - 1 || br.br().y >= opened.rows - 1)
          continue;

        std::vector<cv::Point> hull, quad;
        cv::convexHull(c, hull);
        const double peri = cv::arcLength(hull, true);
        for (double eps = 0.02; eps <= 0.061 && quad.size() != 4; eps += 0.01)
          cv::approxPolyDP(hull, quad, eps * peri, true);
        if (!plausibleQuad(quad)) continue;
        const double quadArea = cv::contourArea(quad);
        if (quadArea < 1e-6 || area / quadArea < 0.8) continue;

        Cand cd;
        for (const auto& p : quad) cd.q.push_back(toFull(p));
        cd.rough = polyArea(cd.q) / frameArea;
        if (haveRef)  // proximité à la position précédente, indépendante de l'échelle
          cd.rough /= 1.0 + 5.0 * quadDist(cd.q, pred_) / std::sqrt(polyArea(pred_) + 1.0);
        cands.push_back(cd);
      }
      if (cands.empty()) continue;

      // 4) On affine les meilleurs candidats et on garde celui qui ressemble le plus à une A4.
      std::sort(cands.begin(), cands.end(),
                [](const Cand& a, const Cand& b) { return a.rough > b.rough; });
      if (cands.size() > 3) cands.resize(3);

      double bestScore = -1;
      for (const auto& cd : cands) {
        std::vector<cv::Point2f> q = cd.q;
        if (p_.subPix) refineQuad(gray, q, R);
        std::vector<cv::Point2f> ord;
        cv::Mat rv, tv;
        double err = 1e9;
        if (!bestA4Order(q, cam_, ord, rv, tv, err) || err > gate) continue;
        const double score = cd.rough / (1.0 + err / std::max(0.5, p_.maxReprojErr));
        if (score > bestScore) {
          bestScore = score;
          detected = q;
        }
      }
      if (bestScore < 0) continue;
      debugMask = opened;
      return true;
    }
    return false;
  };

  cv::Mat dbg;
  if (run(roi, out, dbg)) {
    search_ = roi;
    mask_ = dbg;
    return true;
  }
  // Recherche plein cadre : coûteuse, donc seulement après quelques échecs consécutifs
  // (le flot optique dans process() couvre les pertes d'une ou deux images).
  if (roi != full && coast_ >= 2 && run(full, out, dbg)) {
    search_ = full;
    mask_ = dbg;
    return true;
  }
  search_ = roi;
  mask_ = cv::Mat();
  return false;
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

  // Référence pour la ROI, le seuil appris et le contrôle de saut : le dernier quad vu.
  pred_.clear();
  if (state_ != TrackState::Lost && raw_.size() == 4) pred_ = raw_;

  std::vector<cv::Point2f> det;
  bool viaFlow = false;
  bool found = detect(gray_, det);

  // Secours : la détection par seuillage a échoué -> flot optique (PARESSEUX : calculé
  // seulement ici, plus à chaque image), puis on valide le quad prédit sur de vrais contours.
  if (!found && state_ != TrackState::Lost && coast_ < p_.maxCoastFrames && !prevGray_.empty() &&
      prevGray_.size() == gray_.size()) {
    std::vector<cv::Point2f> fq;
    if (flowQuad(prevGray_, gray_, raw_, fq)) {
      std::vector<cv::Point2f> q = fq;
      if (refineQuad(gray_, q, 8)) {
        det = q;
        pred_ = fq;
        found = true;
        viaFlow = true;
      }
    }
  }

  cv::Mat rv, tv;
  double err = 1e9;
  if (found) {
    std::vector<cv::Point2f> cand;
    if (state_ == TrackState::Lost) {
      found = bestA4Order(det, cam_, cand, rv, tv, err) && err < p_.maxReprojErr * 4.0;
    } else {
      // Suivi : identité des coins conservée, poses aberrantes refusées.
      cand = matchOrder(sortClockwise(det));
      found = estimatePose(cand, cam_, rv, tv, &err) && err < p_.maxReprojErr * 8.0;
      if (found && pred_.size() == 4) {
        double jump = 0;
        for (int i = 0; i < 4; ++i) jump += cv::norm(cand[i] - pred_[i]) / 4;
        found = jump < 0.6 * std::sqrt(polyArea(cand));
      }
    }
    if (found) det = cand;
  }

  if (found) {
    if (state_ == TrackState::Lost) for (auto& f : filters_) f.reset();
    raw_ = det;
    gray_.copyTo(prevGray_);  // image de référence pour un éventuel flot optique
    smooth_.resize(4);
    for (int i = 0; i < 4; ++i) {
      smooth_[i].x = float(filters_[2 * i].filter(raw_[i].x, dt));
      smooth_[i].y = float(filters_[2 * i + 1].filter(raw_[i].y, dt));
    }
    // Pose finale sur les coins lissés (pas de tremblement du contenu 3D).
    double e2 = 0;
    cv::Mat rv2, tv2;
    if (estimatePose(smooth_, cam_, rv2, tv2, &e2)) { rvec_ = rv2; tvec_ = tv2; }
    else { rvec_ = rv; tvec_ = tv; }
    reprojErr_ = err;
    if (viaFlow) {
      state_ = TrackState::Coasting;  // suivi "à l'aveugle" : borné par maxCoastFrames
      ++coast_;
    } else {
      state_ = TrackState::Tracking;
      coast_ = 0;
    }
  } else if (state_ != TrackState::Lost && ++coast_ <= p_.maxCoastFrames) {
    state_ = TrackState::Coasting;  // perte courte : on garde la dernière pose
  } else {
    reset();
  }
  return hasPose();
}
