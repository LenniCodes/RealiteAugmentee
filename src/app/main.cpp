#include <chrono>
#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "io/camera_loader.hpp"
#include "io/frame_source.hpp"
#include "render/overlay.hpp"
#include "tracking/sheet_tracker.hpp"

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
  int delay = (!in.live() && srcFps > 0) ? std::max(1, int(1000.0 / srcFps)) : 1;
  bool showDebug = false, paused = false;
  double avgMs = 0.0, avgFps = 0.0;
  auto last = std::chrono::steady_clock::now();
  cv::Mat frame;

  while (true) {
    if (!paused && !in.read(frame)) break;

    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last).count();
    last = now;
    double sceneDt = (!in.live() && srcFps > 0) ? 1.0 / srcFps : dt;

    auto t0 = std::chrono::steady_clock::now();
    tracker.process(frame, sceneDt);
    double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    avgMs = 0.9 * avgMs + 0.1 * ms;
    if (dt > 0) avgFps = 0.9 * avgFps + 0.1 * (1.0 / dt);

    cv::Mat view = frame.clone();
    if (tracker.hasPose()) {
      drawCube(view, cam, tracker.rvec(), tracker.tvec(), 100.f);
      drawSheetOutline(view, tracker.smoothCorners());
    }
    drawHud(view, tracker.state(), avgMs, avgFps, tracker.reprojErr());

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
