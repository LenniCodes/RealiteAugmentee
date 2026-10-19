/// Génère une vidéo (ou une image) de test : une feuille A4 vue par la caméra de
/// input_camera.yaml, qui se déplace, s'incline et tourne d'un tour complet en 12 s.
/// La scène contient un fond texturé bruité, un éclairage variable avec un dégradé,
/// un objet blanc parasite et deux occultations (3 images puis 15 images).
///
/// Usage : gen_synthetic camera.yaml largeur hauteur nb_images sortie.(mp4|jpg)
/// Exemple : ./gen_synthetic ../input_camera.yaml 1280 720 400 ../media/synthese_720p.mp4
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "pose.hpp"

using namespace cv;

static Mat makeBackground(Size s) {
  Mat n(s / 8, CV_8UC3), b;
  randu(n, 40, 140);
  resize(n, b, s, 0, 0, INTER_CUBIC);
  GaussianBlur(b, b, Size(0, 0), 3);
  return b;
}

/// Pose vraie de la feuille à l'image i (30 fps).
static void sheetPose(int i, Mat& rvec, Mat& tvec) {
  double t = i / 30.0;
  Mat Rz, Rx;
  Rodrigues(Vec3d(0, 0, t * 2 * CV_PI / 12), Rz);  // un tour en 12 s
  Rodrigues(Vec3d(CV_PI - 0.6 - 0.25 * std::sin(t * 1.3), 0.25 * std::sin(t * 0.9), 0), Rx);
  Rodrigues(Mat(Rx * Rz), rvec);
  tvec = (Mat_<double>(3, 1) << 60 * std::sin(t * 0.7), 40 * std::sin(t * 1.1), 650 + 120 * std::sin(t * 0.5));
}

static Mat render(int i, const CameraParams& cam, const Mat& bg) {
  Mat rv, tv;
  sheetPose(i, rv, tv);
  std::vector<Point2f> corners;
  projectPoints(a4ObjectPoints(), rv, tv, cam.K, cam.dist, corners);

  Mat img = bg.clone();
  std::vector<Point> q(corners.begin(), corners.end());
  fillConvexPoly(img, q, Scalar::all(235), LINE_AA);
  circle(img, Point(int(img.cols * 0.86), int(img.rows * 0.2)), img.rows / 10, Scalar::all(230), FILLED, LINE_AA);

  // Éclairage global variable + dégradé vertical, puis bruit capteur
  double light = 0.75 + 0.25 * std::sin(i / 40.0);
  Mat gain(img.size(), CV_32F);
  for (int y = 0; y < gain.rows; ++y) gain.row(y).setTo(light * (0.75 + 0.35 * y / gain.rows));
  Mat f;
  img.convertTo(f, CV_32FC3);
  std::vector<Mat> ch;
  split(f, ch);
  for (auto& c : ch) c = c.mul(gain);
  merge(ch, f);
  Mat noise(img.size(), CV_32FC3);
  randn(noise, 0, 6);
  f += noise;
  f.convertTo(img, CV_8UC3);

  bool hidden = (i >= 200 && i < 203) || (i >= 300 && i < 315);
  if (hidden) img.setTo(Scalar::all(90));
  return img;
}

int main(int argc, char** argv) {
  if (argc < 6) {
    std::cerr << "Usage : " << argv[0] << " camera.yaml largeur hauteur nb_images sortie.(mp4|jpg)\n";
    return 1;
  }
  Size size(std::atoi(argv[2]), std::atoi(argv[3]));
  int n = std::atoi(argv[4]);
  std::string out = argv[5];
  CameraParams cam;
  if (!loadCameraParams(argv[1], cam, size)) return 1;
  Mat bg = makeBackground(size);

  std::string ext = out.substr(out.find_last_of('.') + 1);
  if (ext == "jpg" || ext == "png") return imwrite(out, render(40, cam, bg)) ? 0 : 1;

  VideoWriter vw(out, VideoWriter::fourcc('m', 'p', '4', 'v'), 30, size);
  if (!vw.isOpened()) { std::cerr << "Impossible d'écrire " << out << "\n"; return 1; }
  for (int i = 0; i < n; ++i) vw << render(i, cam, bg);
  std::cout << n << " images écrites dans " << out << "\n";
  return 0;
}
