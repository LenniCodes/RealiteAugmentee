#include <chrono>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "menu.hpp"
#include "pose.hpp"
#include "tracker.hpp"

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

// ------------------------------------------------------------ source d'images

/// Source unique pour vidéo, image fixe ou webcam.
struct FrameSource {
  cv::VideoCapture cap;
  cv::Mat still;  // image fixe (rejouée en boucle)
  bool live = false;

  bool open(const std::string& s) {
    bool isNumber = !s.empty() && s.find_first_not_of("0123456789") == std::string::npos;
    if (isNumber) { live = true; return cap.open(std::stoi(s)); }
    std::string ext = fs::path(s).extension().string();
    for (auto& c : ext) c = char(std::tolower((unsigned char)c));
    if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp") {
      still = cv::imread(s);
      return !still.empty();
    }
    return cap.open(s);
  }
  /// `loop` : revenir au début en fin de vidéo (affichage) ou s'arrêter (mode test).
  bool read(cv::Mat& f, bool loop) {
    if (!still.empty()) { f = still.clone(); return true; }
    if (cap.read(f)) return true;
    if (live || !loop) return false;
    cap.set(cv::CAP_PROP_POS_FRAMES, 0);
    return cap.read(f);
  }
  cv::Size size() {
    if (!still.empty()) return still.size();
    return {int(cap.get(cv::CAP_PROP_FRAME_WIDTH)), int(cap.get(cv::CAP_PROP_FRAME_HEIGHT))};
  }
  double fps() { return still.empty() ? cap.get(cv::CAP_PROP_FPS) : 0; }
};

// ------------------------------------------------------------ session de suivi

struct SessionStats {
  int frames = 0, tracking = 0, coasting = 0, lost = 0;
  double sumMs = 0, maxMs = 0, sumErr = 0;
  cv::Size size;
  void print(const std::string& name) const {
    if (frames == 0) return;
    std::printf("%-28s %5dx%-5d %6d img  suivi %5.1f%%  perdu %5.1f%%  algo %6.2f ms (max %6.1f)  reproj %.2f px\n",
                name.substr(0, 28).c_str(), size.width, size.height, frames, 100.0 * tracking / frames,
                100.0 * lost / frames, sumMs / frames, maxMs, tracking ? sumErr / tracking : 0.0);
  }
};

enum class Action { Back, Next, Prev, Quit };

/// Dossier où enregistrer des images de diagnostic en mode --bench (vide = désactivé).
static std::string g_dumpDir;

/// Image de diagnostic : vue caméra annotée + masque seuillé, côte à côte.
static void dumpFrame(const std::string& name, int idx, const cv::Mat& frame, const SheetTracker& tr) {
  cv::Mat view = frame.clone();
  cv::rectangle(view, tr.searchRect(), {255, 0, 255}, 3);
  if (tr.state() == TrackState::Tracking)
    for (int i = 0; i < 4; ++i) cv::line(view, tr.rawCorners()[i], tr.rawCorners()[(i + 1) % 4], {0, 255, 0}, 3);
  double s = 640.0 / view.cols;
  cv::resize(view, view, cv::Size(), s, s, cv::INTER_AREA);
  const char* st = tr.state() == TrackState::Tracking ? "TRACKING" : tr.state() == TrackState::Coasting ? "COASTING" : "LOST";
  cv::putText(view, cv::format("#%d %s", idx, st), {8, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 255}, 2);
  cv::putText(view, tr.debugInfo(), {8, view.rows - 10}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {0, 255, 255}, 1);
  cv::Mat mask;
  if (!tr.debugMask().empty()) {
    double ms = double(view.rows) / tr.debugMask().rows;
    cv::resize(tr.debugMask(), mask, cv::Size(), ms, ms, cv::INTER_NEAREST);
    cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);
  } else {
    mask = cv::Mat::zeros(view.rows, 10, CV_8UC3);
  }
  cv::Mat out;
  cv::hconcat(view, mask, out);
  cv::imwrite(cv::format("%s/%s_%04d.jpg", g_dumpDir.c_str(), name.c_str(), idx), out);
}

/// Lance le suivi sur une source. `display=false` : mode test (pas de fenêtre, une seule passe).
Action runSession(const std::string& src, const std::string& camPath, bool display, SessionStats& st) {
  FrameSource in;
  if (!in.open(src)) {
    std::cerr << "Impossible d'ouvrir la source : " << src << std::endl;
    return Action::Back;
  }
  st.size = in.size();
  CameraParams cam;
  if (!loadCameraParams(camPath, cam, st.size)) return Action::Quit;

  SheetTracker tracker(cam);
  double srcFps = in.fps();
  int delay = (!in.live && srcFps > 0) ? std::max(1, int(1000.0 / srcFps)) : 1;
  bool showDebug = false, paused = false;
  double avgMs = 0, avgFps = 0;
  auto last = Clock::now();
  int maxFrames = in.still.empty() ? -1 : 30;  // image fixe en mode test : 30 passages
  const std::string win = "ReVA - " + fs::path(src).filename().string();
  cv::Mat frame;

  while (true) {
    if (!paused) {
      if (!in.read(frame, display)) break;
      if (!display && maxFrames >= 0 && st.frames >= maxFrames) break;
    }
    auto now = Clock::now();
    double dt = std::chrono::duration<double>(now - last).count();
    last = now;
    // Pour une vidéo, le temps de la scène est 1/fps, pas le temps de calcul.
    double sceneDt = (!in.live && srcFps > 0) ? 1.0 / srcFps : dt;

    auto t0 = Clock::now();
    tracker.process(frame, sceneDt);
    double ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

    if (!paused) {
      st.frames++;
      st.sumMs += ms;
      st.maxMs = std::max(st.maxMs, ms);
      switch (tracker.state()) {
        case TrackState::Tracking: st.tracking++; st.sumErr += tracker.reprojError(); break;
        case TrackState::Coasting: st.coasting++; break;
        case TrackState::Lost: st.lost++; break;
      }
    }
    if (!display) {
      // Diagnostic : ~10 images réparties dans la vidéo
      int total = in.still.empty() ? std::max(1, int(in.cap.get(cv::CAP_PROP_FRAME_COUNT))) : 1;
      bool pick = in.still.empty() ? (st.frames - 1) % std::max(1, total / 10) == 0 : st.frames == 1;
      if (!g_dumpDir.empty() && pick)
        dumpFrame(fs::path(src).stem().string(), st.frames - 1, frame, tracker);
      continue;
    }

    avgMs = 0.9 * avgMs + 0.1 * ms;
    if (dt > 0) avgFps = 0.9 * avgFps + 0.1 * (1.0 / dt);

    cv::Mat view = frame.clone();
    if (tracker.hasPose()) {
      drawCube(view, cam, tracker.rvec(), tracker.tvec(), 100.f);
      const auto& c = tracker.corners();
      for (int i = 0; i < 4; ++i) {
        cv::line(view, c[i], c[(i + 1) % 4], {0, 255, 0}, 2, cv::LINE_AA);
        cv::putText(view, std::to_string(i), c[i] + cv::Point2f(8, -8), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    {0, 0, 255}, 2);
      }
    }
    const char* state = tracker.state() == TrackState::Tracking ? "TRACKING"
                      : tracker.state() == TrackState::Coasting ? "COASTING" : "LOST";
    cv::putText(view, cv::format("%s  algo %.1f ms  affichage %.0f fps  reproj %.2f px", state, avgMs, avgFps,
                                 tracker.reprojError()),
                {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 255}, 2);
    cv::putText(view, "q: menu  n/p: suivante/precedente  espace: pause  d: debug  r: reset",
                {10, view.rows - 15}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1, cv::LINE_AA);
    if (showDebug) {
      cv::rectangle(view, tracker.searchRect(), {255, 0, 255}, 1);
      if (!tracker.debugMask().empty()) cv::imshow("masque (zone de recherche)", tracker.debugMask());
    }
    cv::imshow(win, view);

    // Cadence : on attend jusqu'à l'heure de l'image suivante. waitKey seul ne suffit pas,
    // certains backends (Qt) rendent la main avant la fin du délai.
    int key = -1;
    auto deadline = now + std::chrono::milliseconds(delay);
    do {
      auto remain = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
      key = cv::waitKeyEx(std::max<int>(1, int(remain)));
    } while (key < 0 && Clock::now() < deadline);
    Action act = Action::Back;
    bool leave = false;
    if (isKeyEscape(key) || key == 'q') leave = true;
    else if (key == 'n') { act = Action::Next; leave = true; }
    else if (key == 'p') { act = Action::Prev; leave = true; }
    else if (key == ' ') paused = !paused;
    else if (key == 'd') { showDebug = !showDebug; if (!showDebug) cv::destroyWindow("masque (zone de recherche)"); }
    else if (key == 'r') tracker.reset();
    else if (key < 0 && cv::getWindowProperty(win, cv::WND_PROP_VISIBLE) < 1) leave = true;  // fenêtre fermée
    if (leave) {
      cv::destroyAllWindows();
      return act;
    }
  }
  if (display) cv::destroyAllWindows();
  return Action::Back;
}

// ------------------------------------------------------------ programme

static void usage(const char* prog) {
  std::cout << "Usage :\n"
            << "  " << prog << " [dossier]                 menu de choix (defaut : ../media)\n"
            << "  " << prog << " <video|image|index_cam>   lance directement une source\n"
            << "  " << prog << " --bench [dossier]         teste toutes les videos du dossier, sans affichage\n"
            << "  " << prog << " --dump sortie [dossier]   comme --bench + images de diagnostic dans sortie/\n"
            << "Option : --cam fichier.yaml  (defaut : ../input_camera.yaml)\n";
}

int main(int argc, char* argv[]) {
  std::string camPath = "../input_camera.yaml", target = "../media";
  bool bench = false;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--cam" && i + 1 < argc) camPath = argv[++i];
    else if (a == "--bench") bench = true;
    else if (a == "--dump" && i + 1 < argc) { bench = true; g_dumpDir = argv[++i]; fs::create_directories(g_dumpDir); }
    else if (a == "-h" || a == "--help") { usage(argv[0]); return 0; }
    else target = a;
  }

  // Mode test : toutes les sources du dossier, une passe chacune, tableau récapitulatif.
  if (bench) {
    auto entries = scanMedia(target, false);
    if (entries.empty()) { std::cerr << "Aucune video dans " << target << std::endl; return 1; }
    std::cout << "Test de " << entries.size() << " source(s) dans " << target << "\n";
    for (auto& e : entries) {
      SessionStats st;
      runSession(e.path, camPath, false, st);
      st.print(e.label);
    }
    return 0;
  }

  // Source unique donnée en argument (ancien comportement).
  if (!fs::is_directory(target)) {
    SessionStats st;
    runSession(target, camPath, true, st);
    st.print(fs::path(target).filename().string());
    return 0;
  }

  // Menu : choisir une source, la lancer, revenir au menu.
  auto entries = scanMedia(target);
  int sel = 0;
  while (true) {
    int choice = runMenu(entries, target, sel);
    if (choice == -1) break;
    if (choice == -2) { entries = scanMedia(target); continue; }
    sel = choice;
    while (true) {
      SessionStats st;
      Action a = runSession(entries[sel].path, camPath, true, st);
      st.print(entries[sel].label);
      int n = int(entries.size());
      if (a == Action::Next) sel = (sel + 1) % n;
      else if (a == Action::Prev) sel = (sel + n - 1) % n;
      else break;
    }
  }
  return 0;
}
