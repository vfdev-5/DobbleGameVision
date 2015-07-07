#ifndef IMAGECOMMON_H
#define IMAGECOMMON_H

// std
#include <vector>

// Qt
#include <QString>

// Opencv
#include <opencv2/core.hpp>

namespace ImageCommon
{

//******************************************************************************************

cv::Mat displayMat(const cv::Mat & inputImage, bool showMinMax=false, const QString &windowName=QString(), bool waitKey=true, bool rescale=true);

cv::Mat displayContourBRect(const std::vector< std::vector<cv::Point> > & contours, const cv::Mat & background);
cv::Mat displayContour(const std::vector< std::vector<cv::Point> > & contours, const cv::Mat & background, bool oneColor=true);

cv::Mat displayContour(const std::vector< std::vector<cv::Point> > & contours, const std::vector<cv::Vec4i> & hierarchy, const cv::Mat & background);


cv::Mat displayCircles(const std::vector<cv::Vec3f> & circles, const cv::Mat & background);

bool isCircleLike(const std::vector<cv::Point> & contour, double tol=1e-2);

//******************************************************************************************

}

#endif // IMAGECOMMON_H
