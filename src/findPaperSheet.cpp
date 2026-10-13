#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include "utils.hpp"


cv::Rect findPaperSheet(cv::Mat image, std::vector<cv::Rect>& rects, cv::Mat& roi){
  image = toGrayScale(image);
  cv::Mat thresh; 
  cv::threshold(image, thresh, 180, 255, cv::THRESH_BINARY);
  std::vector<std::vector<cv::Point> > regions;
  std::vector<cv::Rect> mser_bbox;
  cv::Ptr<cv::MSER> mser = cv::MSER::create(5,40000,500000);
  mser->detectRegions(thresh, regions, mser_bbox);
  if (regions.empty()) return cv::Rect();  // pas de feuille : évite le crash sur [0]
  cv::Rect rect = mser_bbox[0];
  std::vector<cv::Point> biggest_region = regions[0];
  
  rects.insert(rects.end(),mser_bbox.begin(),mser_bbox.end());
  
  for (int i = 1; i < regions.size(); i++)
  {
    if (regions[i].size() > biggest_region.size()){
      rect = mser_bbox[i];
      biggest_region = regions[i];
    }
  }
  thresh(rect).copyTo(roi);
  return rect;
}
