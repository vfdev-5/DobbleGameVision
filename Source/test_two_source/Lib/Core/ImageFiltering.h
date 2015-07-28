#ifndef IMAGEFILTERING_H
#define IMAGEFILTERING_H

// Opencv
#include <opencv2/core.hpp>

// Project
#include "LibExport.h"

namespace ImageFiltering
{

//******************************************************************************************

cv::Mat DGV_DLL_EXPORT fftShift(const cv::Mat & input);

void DGV_DLL_EXPORT freqFilter(const cv::Mat & input, cv::Mat & output, const cv::Mat & freqMask=cv::Mat::ones(10, 10, CV_32F), bool inside=true);

void DGV_DLL_EXPORT enhance(const cv::Mat & input, cv::Mat & output, double strength = 0.25);

cv::Mat DGV_DLL_EXPORT getGaussianKernel2D(const cv::Size & size, double sigmaX, double sigmaY);

cv::Mat DGV_DLL_EXPORT getCircleKernel2D(const cv::Size & size, int value=255, int type=CV_32F);

inline cv::Mat DGV_DLL_EXPORT getCircleKernel2D(int width, int height, int value=255, int type=CV_32F)
{ return getCircleKernel2D(cv::Size(width, height), value, type); }

void DGV_DLL_EXPORT simplify(const cv::Mat & src, cv::Mat & dst, double f);

void DGV_DLL_EXPORT detectCircles(const cv::Mat & image, std::vector<cv::Vec3f> & output, int minRadius, int maxRadius, double threshold=0.5);

//******************************************************************************************

}

#endif // IMAGEFILTERING_H
