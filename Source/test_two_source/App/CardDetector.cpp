
// Opencv
#include <opencv2/imgproc.hpp>

// Project
#include "CardDetector.h"
#include "Core/ImageCommon.h"
#include "Core/ImageFiltering.h"

namespace DGV
{

//******************************************************************************************

CardDetector::CardDetector(int minSize, int maxSize, bool verbose) :
    _minSize(minSize),
    _maxSize(maxSize),
    _verbose(verbose)
{
}

//******************************************************************************************

QVector<cv::Mat> CardDetector::detectCards(const cv::Mat &src)
{
    cv::Mat procImage;
    // to gray scale and smooth
    if (src.channels() > 1)
    {
        cv::Mat out;
        cv::cvtColor(src, out, cv::COLOR_BGR2GRAY);
        procImage = out;
    }
    else
    {
        src.copyTo(procImage);
    }

    double fs = 0.20;
    cv::Size size(fs*procImage.cols, fs*procImage.rows);
    double sigmaX = size.width*0.25;
    double sigmaY = size.height*0.25;
    cv::Mat freqMask = ImageFiltering::getGaussianKernel2D(size, sigmaX, sigmaY);
    ImageFiltering::freqFilter(procImage, procImage, freqMask);

    if (_verbose) ImageCommon::displayMat(procImage, true, "Low freq image");

    //    double f = 5.0;
    //    ImageFiltering::simplify(procImage, procImage, f);
    //    ImageCommon::displayMat(procImage, true, "Resized");

    // Derivate
    cv::Mat t1, t2, t3;
    cv::Scharr(procImage, t1, CV_32F, 1, 0);
    cv::Scharr(procImage, t2, CV_32F, 0, 1);
    if (_verbose) ImageCommon::displayMat(t1, true, "Scharr x");
    if (_verbose) ImageCommon::displayMat(t2, true, "Scharr y");

    cv::magnitude(t1, t2, t3);
    double minVal, maxVal;
    cv::minMaxLoc(t3, &minVal, &maxVal);
    double a = 255.0/(maxVal-minVal);
    double b = -255.0 * minVal/(maxVal-minVal);
    t3.convertTo(procImage, CV_8U, a, b);
    if (_verbose) ImageCommon::displayMat(procImage, true, "Contours");


    // Threshold
    double t = 255*0.4;
    cv::threshold(procImage, procImage, t, 255, cv::THRESH_BINARY);
    if (_verbose) ImageCommon::displayMat(procImage, true, "Threshold");

    // Morpho
    cv::Mat k1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
    //    cv::Mat k2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7,7));
    cv::morphologyEx(procImage, procImage, cv::MORPH_OPEN, k1);
    //    cv::morphologyEx(procImage, procImage, cv::MORPH_CLOSE, k2);
    if (_verbose) ImageCommon::displayMat(procImage, true, "Morpho");


    // Find contours
    std::vector< std::vector<cv::Point> > contours;
    std::vector< std::vector<cv::Point> > out;
    cv::findContours(procImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    int minArea = _minSize*_minSize;
    int maxArea = _maxSize*_maxSize;
    if (_verbose) SD_TRACE(QString("Min area : %1").arg(minArea));
    if (_verbose) SD_TRACE(QString("Contours count : %1").arg(contours.size()));
    for (size_t i=0;i<contours.size();i++)
    {
        std::vector<cv::Point> contour = contours[i];
        cv::Rect brect = cv::boundingRect(contour);
        int a = brect.area();
        if (a > minArea &&
                a < maxArea &&
                ImageCommon::isCircleLike(contour))
        {
            out.push_back(contour);
        }
    }

    if (_verbose) SD_TRACE(QString("Selected contours count : %1").arg(out.size()));
    if (_verbose) ImageCommon::displayContour(contours, procImage);


    QVector<cv::Mat> cards(out.size());

    double marginFactor = 0.03;
    for (size_t i=0; i<out.size(); i++)
    {
        cv::Rect brect = cv::boundingRect(out[i]);

        brect.x += brect.width * marginFactor;
        brect.y += brect.height * marginFactor;
        brect.width -= brect.width * marginFactor * 2;
        brect.height -= brect.height * marginFactor * 2;

        cv::Mat mask = ImageFiltering::getCircleKernel2D(brect.size(), 1, CV_8U);
        std::vector<cv::Mat> mx(src.channels());
        for (int j=0;j<src.channels();j++)
        {
            mx[j] = mask;
        }
        cv::Mat t, mask2;
        cv::merge(mx, mask2);
        t = src(brect).mul(mask2);
        t.copyTo(cards[i]);
        if (_verbose) ImageCommon::displayMat(cards[i], true, QString("Card %1").arg(i));
    }

    return cards;
}

//******************************************************************************************

cv::Mat CardDetector::uniformSize(const cv::Mat &src, int sizeX, int sizeY)
{
    cv::Mat out;
    cv::Size uniSize(sizeX, sizeY);
    if (sizeY==0)
        uniSize.height = sizeX;

    cv::resize(src, out,uniSize);
    return out;
}

//******************************************************************************************

QVector<cv::Mat> CardDetector::uniformSize(const QVector<cv::Mat> &src, int sizeX, int sizeY)
{
    QVector<cv::Mat> out(src.size());
    for (int count=0; count<src.size(); count++)
    {
        const cv::Mat & mat = src[count];
        out[count] = uniformSize(mat, sizeX, sizeY);
    }
    return out;
}

//******************************************************************************************

}
