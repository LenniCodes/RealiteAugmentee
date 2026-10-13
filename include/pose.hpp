#ifndef POSE_HPP
#define POSE_HPP

#include <opencv2/core.hpp>
#include <string>
#include <vector>

/// Paramètres intrinsèques de la caméra.
struct CameraParams {
  cv::Mat K;     ///< matrice intrinsèque 3x3 (fx, fy, cx, cy)
  cv::Mat dist;  ///< coefficients de distorsion (k1, k2, p1, p2, k3)
};

/// Charge K et dist depuis un YAML OpenCV (format de input_camera.yaml).
/// Si frameSize diffère de la résolution de calibration (2*cx, 2*cy),
/// fx, fy, cx, cy sont mis à l'échelle.
bool loadCameraParams(const std::string& path, CameraParams& cam,
                      cv::Size frameSize = cv::Size());

/// Ordonne 4 coins dans le sens horaire (image) en commençant par un coin
/// tel que l'arête 0->1 soit un grand côté de la feuille.
std::vector<cv::Point2f> orderCorners(const std::vector<cv::Point>& quad);

/// Points 3D de la feuille A4 dans son repère (mm), centrés, Z vers la caméra.
std::vector<cv::Point3f> a4ObjectPoints(float w = 297.f, float h = 210.f);

/// Estime la pose de la feuille. Renvoie false si l'estimation échoue.
bool estimatePose(const std::vector<cv::Point2f>& imgPts, const CameraParams& cam,
                  cv::Mat& rvec, cv::Mat& tvec, double* reprojErr = nullptr);

/// Dessine un cube posé sur la feuille (projection via projectPoints).
void drawCube(cv::Mat& img, const CameraParams& cam, const cv::Mat& rvec,
              const cv::Mat& tvec, float side = 100.f);

#endif
