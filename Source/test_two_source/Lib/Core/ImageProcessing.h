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

void DGV_DLL_EXPORT enhance(const cv::Mat & input, cv::Mat & output, double strength = 0.25, bool laplacianOnly=true);

cv::Mat DGV_DLL_EXPORT getGaussianKernel2D(const cv::Size & size, double sigmaX, double sigmaY);

cv::Mat DGV_DLL_EXPORT getCircleKernel2D(const cv::Size & size, double value=255.0, int type=CV_32F);

inline cv::Mat DGV_DLL_EXPORT getCircleKernel2D(int width, int height, double value=255.0, int type=CV_32F)
{ return getCircleKernel2D(cv::Size(width, height), value, type); }

void DGV_DLL_EXPORT simplify(const cv::Mat & src, cv::Mat & dst, double f);

#ifdef HAS_3RDPARTY
void DGV_DLL_EXPORT nonlinearDiffusionFiltering(const cv::Mat & input, cv::Mat & output);
#endif

void DGV_DLL_EXPORT edgeStrength(const cv::Mat & input, cv::Mat & output, int ksize=3);


// Detection methods
//******************************************************************************************

typedef QVector<std::vector<cv::Point> > Contours;

void DGV_DLL_EXPORT detectCircles(const cv::Mat & image, std::vector<cv::Vec3f> & output, int minRadius, int maxRadius, double threshold=0.5);

cv::Mat DGV_DLL_EXPORT getObjectMask(const cv::Size &size, const std::vector<cv::Point> & contour);

enum DetectedObjectType {
    ANY=0,
    ELLIPSE_LIKE=1,
    NOT_ELLIPSE_LIKE=2,
};

void DGV_DLL_EXPORT detectObjects(const cv::Mat & image,
                                  Contours * objectContours,
                                  double minSizeRatio=0.0, double maxSizeRatio=1.0,
                                  const cv::Mat & mask=cv::Mat(), DetectedObjectType type=ANY, double param=0.0,
                                  bool verbose=false);



//******************************************************************************************

}

#endif // IMAGEFILTERING_H
