#ifndef IO_FRAME_SOURCE_HPP
#define IO_FRAME_SOURCE_HPP

#include <opencv2/videoio.hpp>
#include <string>

class FrameSource {
 public:
  bool open(const std::string& path);
  bool read(cv::Mat& frame);
  cv::Size size() const;
  double fps() const;
  bool live() const;

 private:
  cv::VideoCapture cap_;
  cv::Mat still_;
  bool live_ = false;
};

#endif
