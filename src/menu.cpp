#include "menu.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace fs = std::filesystem;

static std::string lower(std::string s) {
  for (auto& c : s) c = char(std::tolower((unsigned char)c));
  return s;
}

static bool isVideoExt(const std::string& e) {
  return e == ".mp4" || e == ".avi" || e == ".mov" || e == ".mkv" || e == ".webm" || e == ".m4v";
}
static bool isImageExt(const std::string& e) {
  return e == ".jpg" || e == ".jpeg" || e == ".png" || e == ".bmp";
}

// Lit la taille, les fps, le nombre d'images et une miniature (sans garder le fichier ouvert).
static void probe(MediaEntry& m) {
  if (m.isCamera) return;  // on n'allume pas la webcam juste pour le menu
  cv::Mat first;
  std::string ext = lower(fs::path(m.path).extension().string());
  if (isImageExt(ext)) {
    first = cv::imread(m.path);
  } else {
    cv::VideoCapture cap(m.path);
    if (cap.isOpened()) {
      m.fps = cap.get(cv::CAP_PROP_FPS);
      m.frames = int(cap.get(cv::CAP_PROP_FRAME_COUNT));
      cap.read(first);
    }
  }
  if (first.empty()) return;
  m.size = first.size();
  double s = 360.0 / first.cols;
  cv::resize(first, m.thumb, cv::Size(), s, s, cv::INTER_AREA);
}

std::vector<MediaEntry> scanMedia(const std::string& folder, bool addWebcam) {
  std::vector<MediaEntry> out;
  std::error_code ec;
  if (fs::is_directory(folder, ec)) {
    for (auto& f : fs::directory_iterator(folder, ec)) {
      if (!f.is_regular_file()) continue;
      std::string ext = lower(f.path().extension().string());
      if (!isVideoExt(ext) && !isImageExt(ext)) continue;
      MediaEntry m;
      m.path = f.path().string();
      m.label = f.path().filename().string();
      out.push_back(m);
    }
  }
  std::sort(out.begin(), out.end(), [](auto& a, auto& b) { return a.label < b.label; });
  for (auto& m : out) probe(m);
  if (addWebcam) {
    MediaEntry cam;
    cam.path = "0";
    cam.label = "Webcam 0 (direct)";
    cam.isCamera = true;
    out.push_back(cam);
  }
  return out;
}

// Codes des flèches selon le backend : GTK/X11 (65362...), Windows (2490368...), Qt (0x1000013...).
bool isKeyUp(int k) {
  return k == 65362 || k == 2490368 || k == 0x1000013 || k == 82 || k == 'z' || k == 'w' || k == 'k';
}
bool isKeyDown(int k) {
  return k == 65364 || k == 2621440 || k == 0x1000015 || k == 84 || k == 's' || k == 'j';
}
bool isKeyEnter(int k) { return k == 13 || k == 10 || k == 0x1000004 || k == 0x1000005 || k == 65293 || k == 65421; }
bool isKeyEscape(int k) { return k == 27 || k == 0x1000000 || k == 65307; }

// ------------------------------------------------------------------ dessin

namespace {
const int W = 1000, H = 620, LIST_X = 20, LIST_Y = 90, ROW_H = 34, LIST_W = 540;
const cv::Scalar BG(32, 28, 26), FG(235, 235, 235), DIM(150, 150, 150), ACCENT(60, 180, 255);

struct MouseState {
  int hover = -1, clicked = -1, scroll = 0;
  bool dbl = false;
};

void onMouse(int ev, int x, int y, int, void* ud) {
  auto* ms = static_cast<MouseState*>(ud);
  int row = (x >= LIST_X && x < LIST_X + LIST_W && y >= LIST_Y) ? (y - LIST_Y) / ROW_H : -1;
  ms->hover = row < 0 ? -1 : row + ms->scroll;
  if (ev == cv::EVENT_LBUTTONDOWN && row >= 0) ms->clicked = row + ms->scroll;
  if (ev == cv::EVENT_LBUTTONDBLCLK && row >= 0) { ms->clicked = row + ms->scroll; ms->dbl = true; }
}

void text(cv::Mat& img, const std::string& s, cv::Point p, double sc, cv::Scalar c, int th = 1) {
  cv::putText(img, s, p, cv::FONT_HERSHEY_SIMPLEX, sc, c, th, cv::LINE_AA);
}
}  // namespace

int runMenu(std::vector<MediaEntry>& entries, const std::string& folder, int selected) {
  const std::string win = "ReVA - choix de la source";
  cv::namedWindow(win, cv::WINDOW_AUTOSIZE);
  MouseState ms;
  cv::setMouseCallback(win, onMouse, &ms);
  int n = int(entries.size());
  int visibleRows = (H - LIST_Y - 60) / ROW_H;
  selected = std::clamp(selected, 0, std::max(0, n - 1));

  while (true) {
    // Défilement pour garder la sélection visible
    if (selected < ms.scroll) ms.scroll = selected;
    if (selected >= ms.scroll + visibleRows) ms.scroll = selected - visibleRows + 1;

    cv::Mat img(H, W, CV_8UC3, BG);
    text(img, "ReVA - Realite augmentee", {LIST_X, 40}, 0.9, FG, 2);
    text(img, "Dossier : " + folder + "   (" + std::to_string(n) + " sources)", {LIST_X, 70}, 0.5, DIM);

    if (n == 0) text(img, "Aucune video : ajoutez des .mp4/.jpg dans le dossier puis 'u'", {LIST_X, 130}, 0.6, ACCENT);

    for (int r = 0; r < visibleRows && ms.scroll + r < n; ++r) {
      int i = ms.scroll + r;
      int y = LIST_Y + r * ROW_H;
      cv::Rect row(LIST_X, y, LIST_W, ROW_H - 4);
      if (i == selected) cv::rectangle(img, row, ACCENT * 0.45, cv::FILLED);
      else if (i == ms.hover) cv::rectangle(img, row, cv::Scalar(55, 50, 48), cv::FILLED);
      std::string num = i < 9 ? std::to_string(i + 1) + "." : "  ";
      std::string lbl = entries[i].label;
      if (lbl.size() > 40) lbl = lbl.substr(0, 37) + "...";
      text(img, num + " " + lbl, {LIST_X + 10, y + 22}, 0.6, i == selected ? FG : cv::Scalar(205, 205, 205),
           i == selected ? 2 : 1);
    }
    if (ms.scroll > 0) text(img, "^", {LIST_X + LIST_W - 20, LIST_Y - 5}, 0.6, DIM);
    if (ms.scroll + visibleRows < n) text(img, "v", {LIST_X + LIST_W - 20, H - 60}, 0.6, DIM);

    // Panneau de droite : aperçu + infos de la source sélectionnée
    if (n > 0) {
      const MediaEntry& m = entries[selected];
      int px = LIST_X + LIST_W + 30, py = LIST_Y;
      if (!m.thumb.empty()) {
        cv::Rect r(px, py, m.thumb.cols, std::min(m.thumb.rows, 260));
        m.thumb(cv::Rect(0, 0, r.width, r.height)).copyTo(img(r));
        py += r.height + 30;
      } else {
        cv::rectangle(img, cv::Rect(px, py, 360, 200), cv::Scalar(55, 50, 48), cv::FILLED);
        text(img, m.isCamera ? "flux camera en direct" : "apercu indisponible", {px + 70, py + 105}, 0.6, DIM);
        py += 230;
      }
      if (m.isCamera) {
        text(img, "Camera index " + m.path, {px, py}, 0.55, FG);
      } else {
        text(img, cv::format("%d x %d", m.size.width, m.size.height), {px, py}, 0.55, FG);
        if (m.fps > 0) {
          text(img, cv::format("%.1f fps, %d images (%.1f s)", m.fps, m.frames, m.frames / m.fps),
               {px, py + 26}, 0.55, FG);
        } else if (m.frames == 0 && !m.thumb.empty()) {
          text(img, "image fixe", {px, py + 26}, 0.55, FG);
        } else if (m.thumb.empty()) {
          text(img, "fichier illisible", {px, py + 26}, 0.55, cv::Scalar(80, 80, 255));
        }
      }
    }

    text(img, "Haut/Bas ou clic : choisir   Entree / double-clic : lancer   1-9 : lancer direct   "
              "u : rescanner   q : quitter",
         {LIST_X, H - 20}, 0.45, DIM);
    cv::imshow(win, img);

    int k = cv::waitKeyEx(30);
    if (ms.clicked >= 0 && ms.clicked < n) {
      selected = ms.clicked;
      ms.clicked = -1;
      if (ms.dbl) { ms.dbl = false; cv::destroyWindow(win); return selected; }
    }
    if (k < 0) {
      if (cv::getWindowProperty(win, cv::WND_PROP_VISIBLE) < 1) return -1;  // fenêtre fermée
      continue;
    }
    if (k == 'q' || isKeyEscape(k)) { cv::destroyWindow(win); return -1; }
    if (k == 'u') { cv::destroyWindow(win); return -2; }
    if (isKeyUp(k)) selected = std::max(0, selected - 1);
    else if (isKeyDown(k)) selected = std::min(n - 1, selected + 1);
    else if (isKeyEnter(k) && n > 0) { cv::destroyWindow(win); return selected; }
    else if (k >= '1' && k <= '9' && k - '1' < n) { cv::destroyWindow(win); return k - '1'; }
  }
}
