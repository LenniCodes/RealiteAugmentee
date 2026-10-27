#include "io/frame_source.hpp"

#include <opencv2/imgcodecs.hpp>

#include <cctype>

bool FrameSource::open(const std::string& path) {
  if (path.empty()) return false;
  bool isNumber = !path.empty() && path.find_first_not_of("0123456789") == std::string::npos;
  if (isNumber) {
    live_ = true;
    return cap_.open(std::stoi(path));
  }

  std::string ext;
  std::size_t dot = path.find_last_of('.');
  if (dot != std::string::npos && dot + 1 < path.size()) ext = path.substr(dot + 1);
  for (auto& c : ext) c = char(std::tolower(static_cast<unsigned char>(c)));

  if (ext == "jpg" || ext == "jpeg" || ext == "png" || ext == "bmp") {
    still_ = cv::imread(path);
    return !still_.empty();
  }

  live_ = false;
  return cap_.open(path);
}

bool FrameSource::read(cv::Mat& frame) {
  if (!still_.empty()) {
    frame = still_.clone();
    return true;
  }
  if (cap_.read(frame)) return true;
  if (live_) return false;
  cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
  return cap_.read(frame);
}

cv::Size FrameSource::size() const {
  if (!still_.empty()) return still_.size();
  return {int(cap_.get(cv::CAP_PROP_FRAME_WIDTH)), int(cap_.get(cv::CAP_PROP_FRAME_HEIGHT))};
}

double FrameSource::fps() const {
  return still_.empty() ? cap_.get(cv::CAP_PROP_FPS) : 0.0;
}

bool FrameSource::live() const {
  return live_;
}
