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
cv::Mat DGV_DLL_EXPORT displayContours(const std::vector< std::vector<cv::Point> > & contours, const cv::Mat & background=cv::Mat(),
                                      bool oneColor=true, bool fillContours=false, const QString &winname="Contours");

cv::Mat DGV_DLL_EXPORT displayContours(const std::vector< std::vector<cv::Point> > & contours, const std::vector<cv::Vec4i> & hierarchy, const cv::Mat & background);


cv::Mat DGV_DLL_EXPORT displayCircles(const std::vector<cv::Vec3f> & circles, const cv::Mat & background);

bool DGV_DLL_EXPORT isEllipseLike(const std::vector<cv::Point> & contour, double tol=1e-2);

bool DGV_DLL_EXPORT isCircleLike(const std::vector<cv::Point> & contour, double tol=1e-2);

//******************************************************************************************

void DGV_DLL_EXPORT printMat(const cv::Mat & inputImage, const QString &windowName=QString(), int limit=10);

}

#endif // IMAGECOMMON_H
