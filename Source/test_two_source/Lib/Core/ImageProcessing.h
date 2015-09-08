#ifndef IMAGEFILTERING_H
#define IMAGEFILTERING_H

// Qt
#include <QVector>

// Opencv
#include <opencv2/core.hpp>

// Project
#include "LibExport.h"

namespace ImageProcessing
{

// Filtering methods
//******************************************************************************************

cv::Mat DGV_DLL_EXPORT fftShift(const cv::Mat & input);

void DGV_DLL_EXPORT freqFilter(const cv::Mat & input, cv::Mat & output, const cv::Mat & freqMask=cv::Mat::ones(10, 10, CV_32F), bool inside=true);

void DGV_DLL_EXPORT enhance(const cv::Mat & input, cv::Mat & output, double strength = 0.25);

cv::Mat DGV_DLL_EXPORT getGaussianKernel2D(const cv::Size & size, double sigmaX, double sigmaY);

cv::Mat DGV_DLL_EXPORT getCircleKernel2D(const cv::Size & size, double value=255.0, int type=CV_32F);

inline cv::Mat DGV_DLL_EXPORT getCircleKernel2D(int width, int height, double value=255.0, int type=CV_32F)
{ return getCircleKernel2D(cv::Size(width, height), value, type); }

void DGV_DLL_EXPORT simplify(const cv::Mat & src, cv::Mat & dst, double f);

void DGV_DLL_EXPORT nonlinearDiffusionFiltering(const cv::Mat & input, cv::Mat & output);

// Detection methods
//******************************************************************************************

void DGV_DLL_EXPORT detectCircles(const cv::Mat & image, std::vector<cv::Vec3f> & output, int minRadius, int maxRadius, double threshold=0.5);

cv::Mat DGV_DLL_EXPORT getObjectMask(const cv::Size &size, const std::vector<cv::Point> & contour);

enum DetectedObjectType {
    ANY=0,
    CIRCLE_LIKE=1,
};

void DGV_DLL_EXPORT detectObjects(const cv::Mat & image, QVector<std::vector<cv::Point> > * objectContours,
                                  double minSizeRatio=0.1, double maxSizeRatio=0.7,
                                  DetectedObjectType type=ANY, bool verbose=false);


//******************************************************************************************

}

#endif // IMAGEFILTERING_H