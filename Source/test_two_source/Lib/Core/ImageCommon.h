#ifndef IMAGECOMMON_H
#define IMAGECOMMON_H

// std
#include <vector>

// Qt
#include <QString>

// Opencv
#include <opencv2/core.hpp>

// Project
#include "LibExport.h"

namespace ImageCommon
{

//******************************************************************************************

cv::Mat DGV_DLL_EXPORT displayMat(const cv::Mat & inputImage, bool showMinMax=false, const QString &windowName=QString(), bool waitKey=true, bool rescale=true);

cv::Mat DGV_DLL_EXPORT displayContourBRect(const std::vector< std::vector<cv::Point> > & contours, const cv::Mat & background);
cv::Mat DGV_DLL_EXPORT displayContour(const std::vector< std::vector<cv::Point> > & contours, const cv::Mat & background, bool oneColor=true, bool fillContours=false);

cv::Mat DGV_DLL_EXPORT displayContour(const std::vector< std::vector<cv::Point> > & contours, const std::vector<cv::Vec4i> & hierarchy, const cv::Mat & background);


cv::Mat DGV_DLL_EXPORT displayCircles(const std::vector<cv::Vec3f> & circles, const cv::Mat & background);

bool DGV_DLL_EXPORT isCircleLike(const std::vector<cv::Point> & contour, double tol=1e-2);

//******************************************************************************************

}

#endif // IMAGECOMMON_H