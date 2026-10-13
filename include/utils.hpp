#ifndef UTILS
#define UTILS

#include <opencv2/core.hpp>

cv::Mat toGrayScale(const cv::Mat input);
cv::Rect findPaperSheet(cv::Mat image, std::vector<cv::Rect>& rects, cv::Mat& roi);

#endif
