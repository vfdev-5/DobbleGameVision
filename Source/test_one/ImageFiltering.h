#ifndef IMAGEFILTERING_H
#define IMAGEFILTERING_H

// Opencv
#include <opencv2/core.hpp>

namespace ImageFiltering
{

//******************************************************************************************

cv::Mat fftShift(const cv::Mat & input);
void freqFilter(const cv::Mat & input, cv::Mat & output, const cv::Mat & freqMask=cv::Mat::ones(10, 10, CV_32F), bool inside=true);

cv::Mat getGaussianKernel2D(const cv::Size & size, double sigmaX, double sigmaY);

cv::Mat getCircleKernel2D(const cv::Size & size, int value=255, int type=CV_32F);
inline cv::Mat getCircleKernel2D(int width, int height, int value=255, int type=CV_32F)
{ return getCircleKernel2D(cv::Size(width, height), value); }


void simplify(const cv::Mat & src, cv::Mat & dst, double f);

void detectCircles(const cv::Mat & image, std::vector<cv::Vec3f> & output, int minRadius, int maxRadius, double threshold=0.5);



//******************************************************************************************

}

#endif // IMAGEFILTERING_H
