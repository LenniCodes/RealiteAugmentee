#include <pose.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <opencv2/imgproc.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/3d.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif

bool loadCameraParams(const std::string& path, CameraParams& cam, cv::Size frameSize) {
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "Impossible d'ouvrir " << path << std::endl;
    return false;
  }
  fs["camera_matrix"] >> cam.K;
  fs["distortion_coefficients"] >> cam.dist;
  if (cam.K.empty()) return false;
  if (cam.dist.empty()) cam.dist = cv::Mat::zeros(1, 5, CV_64F);
  cam.K.convertTo(cam.K, CV_64F);
  cam.dist.convertTo(cam.dist, CV_64F);

  // Le YAML est donné pour une résolution précise (ici 1280x720 : cx=640, cy=360).
  // Si la vidéo a une autre résolution, on met K à l'échelle.
  if (frameSize.area() > 0) {
    double calibW = 2.0 * cam.K.at<double>(0, 2);
    double calibH = 2.0 * cam.K.at<double>(1, 2);
    double w = frameSize.width, h = frameSize.height;
    if (std::abs(w - calibW) > 0.5 || std::abs(h - calibH) > 0.5) {
      // Mise à l'échelle UNIFORME (même facteur en x et en y), sinon la caméra est
      // "étirée" et toutes les poses sont fausses. Si le rapport largeur/hauteur diffère
      // (4:3, vidéo de téléphone en portrait...), on suppose que la focale suit le plus
      // grand côté de l'image : c'est une approximation, une calibration reste préférable.
      bool sameAspect = std::abs(w / h - calibW / calibH) < 0.01;
      double s = sameAspect ? w / calibW : std::max(w, h) / std::max(calibW, calibH);
      cam.K.at<double>(0, 0) *= s;
      cam.K.at<double>(1, 1) *= s;
      cam.K.at<double>(0, 2) = w / 2.0;
      cam.K.at<double>(1, 2) = h / 2.0;
      if (!sameAspect)
        std::cerr << "Attention : " << path << " est prevu pour " << calibW << "x" << calibH
                  << ", la source fait " << w << "x" << h
                  << " -> K approximee (calibrer la camera pour plus de precision)\n";
    }
  }
  return true;
}

std::vector<cv::Point2f> orderCorners(const std::vector<cv::Point>& quad) {
  std::vector<cv::Point2f> pts(quad.begin(), quad.end());
  cv::Point2f c(0, 0);
  for (auto& p : pts) c += p;
  c *= 0.25f;

  // Tri par angle autour du centre -> sens horaire à l'écran (axe y vers le bas).
  std::sort(pts.begin(), pts.end(), [&](const cv::Point2f& a, const cv::Point2f& b) {
    return std::atan2(a.y - c.y, a.x - c.x) < std::atan2(b.y - c.y, b.x - c.x);
  });

  // On démarre sur le coin le plus en haut à gauche (x+y minimal)...
  int start = 0;
  for (int i = 1; i < 4; ++i)
    if (pts[i].x + pts[i].y < pts[start].x + pts[start].y) start = i;
  std::rotate(pts.begin(), pts.begin() + start, pts.end());

  // ...puis on décale d'un cran si l'arête 0->1 n'est pas un grand côté.
  // (la somme des deux côtés opposés est plus robuste à la perspective)
  double e01 = cv::norm(pts[1] - pts[0]) + cv::norm(pts[3] - pts[2]);
  double e12 = cv::norm(pts[2] - pts[1]) + cv::norm(pts[0] - pts[3]);
  if (e12 > e01) std::rotate(pts.begin(), pts.begin() + 1, pts.end());
  return pts;
}

std::vector<cv::Point3f> a4ObjectPoints(float w, float h) {
  // Même ordre que orderCorners : horaire vu de la caméra, arête 0->1 = grand côté.
  // Repère : origine au centre, X le long du grand côté, Y vers le "haut" de la
  // feuille, Z = X ^ Y sort de la feuille vers la caméra.
  return {
      {-w / 2,  h / 2, 0},
      { w / 2,  h / 2, 0},
      { w / 2, -h / 2, 0},
      {-w / 2, -h / 2, 0},
  };
}

bool estimatePose(const std::vector<cv::Point2f>& imgPts, const CameraParams& cam,
                  cv::Mat& rvec, cv::Mat& tvec, double* reprojErr) {
  if (imgPts.size() != 4) return false;
  std::vector<cv::Point3f> objPts = a4ObjectPoints();

  // IPPE : solveur dédié aux cibles planes, analytique, très adapté à 4 coins.
  bool ok = cv::solvePnP(objPts, imgPts, cam.K, cam.dist, rvec, tvec, false,
                         cv::SOLVEPNP_IPPE);
  if (!ok) return false;

  if (reprojErr) {
    std::vector<cv::Point2f> proj;
    cv::projectPoints(objPts, rvec, tvec, cam.K, cam.dist, proj);
    double e = 0;
    for (size_t i = 0; i < proj.size(); ++i) e += cv::norm(proj[i] - imgPts[i]);
    *reprojErr = e / proj.size();
  }
  return tvec.at<double>(2) > 0;  // la feuille doit être devant la caméra
}

void drawCube(cv::Mat& img, const CameraParams& cam, const cv::Mat& rvec,
              const cv::Mat& tvec, float s) {
  float h = s / 2;
  std::vector<cv::Point3f> cube = {
      {-h, -h, 0}, {h, -h, 0}, {h, h, 0}, {-h, h, 0},  // base sur la feuille
      {-h, -h, s}, {h, -h, s}, {h, h, s}, {-h, h, s},  // haut (Z vers la caméra)
  };
  std::vector<cv::Point2f> p;
  cv::projectPoints(cube, rvec, tvec, cam.K, cam.dist, p);

  const cv::Scalar base(0, 200, 0), top(255, 100, 0), pillar(0, 200, 255);
  for (int i = 0; i < 4; ++i) {
    cv::line(img, p[i], p[(i + 1) % 4], base, 2, cv::LINE_AA);
    cv::line(img, p[i + 4], p[(i + 1) % 4 + 4], top, 2, cv::LINE_AA);
    cv::line(img, p[i], p[i + 4], pillar, 2, cv::LINE_AA);
  }
  // Repère : X rouge, Y vert, Z bleu
  cv::drawFrameAxes(img, cam.K, cam.dist, rvec, tvec, s * 0.8f, 3);
}
