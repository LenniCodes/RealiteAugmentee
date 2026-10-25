#include <chrono>
#include <iostream>
#include <string>
#include <cctype>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "pose.hpp"
#include "tracker.hpp"

// Source d'images unique pour vidéo, image fixe ou webcam.
struct FrameSource {
  cv::VideoCapture cap;
  cv::Mat still;  // image fixe (rejouée en boucle)
  bool live = false;

  bool open(const std::string& s) {
    bool isNumber = !s.empty() && s.find_first_not_of("0123456789") == std::string::npos;
    if (isNumber) { live = true; return cap.open(std::stoi(s)); }
    std::string ext;
    std::size_t dot = s.find_last_of('.');
    if (dot != std::string::npos && dot + 1 < s.size()) ext = s.substr(dot + 1);
    for (auto& c : ext) c = char(std::tolower(static_cast<unsigned char>(c)));
    if (ext == "jpg" || ext == "jpeg" || ext == "png" || ext == "bmp") {
      still = cv::imread(s);
      return !still.empty();
    }
    return cap.open(s);
  }
  bool read(cv::Mat& f) {
    if (!still.empty()) { f = still.clone(); return true; }
    if (cap.read(f)) return true;
    if (live) return false;
    cap.set(cv::CAP_PROP_POS_FRAMES, 0);  // vidéo : on boucle
    return cap.read(f);
  }
  cv::Size size() {
    if (!still.empty()) return still.size();
    return {int(cap.get(cv::CAP_PROP_FRAME_WIDTH)), int(cap.get(cv::CAP_PROP_FRAME_HEIGHT))};
  }
  double fps() { return still.empty() ? cap.get(cv::CAP_PROP_FPS) : 0; }
};

// Usage : ./CoursOpenCv [video|image|index_webcam] [camera.yaml]
int main(int argc, char* argv[]) {
  std::string src = argc > 1 ? argv[1] : "../input.mp4";
  std::string camPath = argc > 2 ? argv[2] : "../input_camera.yaml";

  FrameSource in;
  if (!in.open(src)) {
    std::cerr << "Impossible d'ouvrir la source : " << src << std::endl;
    return 1;
  }
  CameraParams cam;
  if (!loadCameraParams(camPath, cam, in.size())) return 1;
  std::cout << "Source " << src << " " << in.size() << "\nK =\n" << cam.K
            << "\ndist = " << cam.dist << std::endl;

  SheetTracker tracker(cam);
  double srcFps = in.fps();
  int delay = (!in.live && srcFps > 0) ? std::max(1, int(1000.0 / srcFps)) : 1;
  bool showDebug = false, paused = false;
  double avgMs = 0, avgFps = 0;
  auto last = std::chrono::steady_clock::now();
  cv::Mat frame;

  while (true) {
    if (!paused && !in.read(frame)) break;

    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last).count();
    last = now;
    // Pour une vidéo, le temps "réel" de la scène est 1/fps, pas le temps de calcul.
    double sceneDt = (!in.live && srcFps > 0) ? 1.0 / srcFps : dt;

    auto t0 = std::chrono::steady_clock::now();
    tracker.process(frame, sceneDt);
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    avgMs = 0.9 * avgMs + 0.1 * ms;
    if (dt > 0) avgFps = 0.9 * avgFps + 0.1 * (1.0 / dt);

    cv::Mat view = frame.clone();
    if (tracker.hasPose()) {
      drawCube(view, cam, tracker.rvec(), tracker.tvec(), 100.f);
      const auto& c = tracker.smoothCorners();
      for (int i = 0; i < 4; ++i) {
        cv::line(view, c[i], c[(i + 1) % 4], {0, 255, 0}, 2, cv::LINE_AA);
        cv::putText(view, std::to_string(i), c[i] + cv::Point2f(8, -8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 0, 255}, 2);
      }
    }
    const char* st = tracker.state() == TrackState::Tracking ? "TRACKING"
                   : tracker.state() == TrackState::Coasting ? "COASTING" : "LOST";
    cv::putText(view, cv::format("%s  algo %.1f ms  affichage %.0f fps  reproj %.2f px", st, avgMs,
                                 avgFps, tracker.reprojErr()),
                {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 255}, 2);
    if (showDebug) {
      cv::rectangle(view, tracker.searchRoi(), {255, 0, 255}, 1);
      if (!tracker.debugMask().empty()) cv::imshow("masque (zone de recherche)", tracker.debugMask());
    }
    cv::imshow("ReVA", view);

    int key = cv::waitKey(delay);
    if (key == 27 || key == 'q') break;
    if (key == ' ') paused = !paused;
    if (key == 'd') showDebug = !showDebug;
    if (key == 'r') tracker.reset();
  }
  return 0;
}
