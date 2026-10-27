#ifndef RENDER_OVERLAY_HPP
#define RENDER_OVERLAY_HPP

#include "core/types.hpp"

void drawCube(cv::Mat& img, const CameraParams& cam, const cv::Mat& rvec,
              const cv::Mat& tvec, float size = 100.f);
void drawSheetOutline(cv::Mat& img, const Quad& corners);
void drawHud(cv::Mat& img, TrackState state, double algoMs, double fps,
             double reprojErr);

#endif
