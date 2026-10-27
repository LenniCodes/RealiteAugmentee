#ifndef GEOMETRY_POSE_ESTIMATOR_HPP
#define GEOMETRY_POSE_ESTIMATOR_HPP

#include "core/types.hpp"

std::vector<cv::Point3f> a4ObjectPoints(float w = 297.f, float h = 210.f);
bool estimatePose(const Quad& imgPts, const CameraParams& cam, cv::Mat& rvec,
                  cv::Mat& tvec, double* reprojErr = nullptr);
bool bestA4Order(const Quad& q, const CameraParams& cam, Quad& order,
                 cv::Mat& rvec, cv::Mat& tvec, double& err);

#endif
