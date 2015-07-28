
// Qt
#include <qmath.h>

// Opencv
#include <opencv2/imgproc.hpp>

// Project
#include "CardDetector.h"
#include "Core/ImageCommon.h"
#include "Core/ImageFiltering.h"

namespace DGV
{

struct Compare : public std::binary_function<std::vector<cv::Point>, std::vector<cv::Point>, bool>
{
    enum Type {Less, Greater};
    Compare(Type type) :
        _type(type)
    {
//        switch (_type) {
//        case Less: _func = &std::operator<; break;
//        case Greater: _func = &std::operator>; break;
//        default: _func = &std::operator<; break;
//        }
    }
    bool operator() (const std::vector<cv::Point> & c1, const std::vector<cv::Point> c2) const
    {
        cv::Rect b1 = cv::boundingRect(c1);
        cv::Rect b2 = cv::boundingRect(c2);
//        return (*_func)(b1.area(), b2.area());
        return _type == Greater ? b1.area() > b2.area() : b1.area() < b2.area();
    }
protected:
    Type _type;
//    bool (*_func)(const int &, const int &);
};

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

    double marginFactor = 0.02;
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
        // cut a little the circle boundaries
        int cut = 1;
        cv::Rect r(cut,cut,brect.width-2*cut, brect.height-2*cut);
        t(r).copyTo(cards[i]);
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

QVector<cv::Mat> CardDetector::uniformSize(const QVector<cv::Mat> &cards, int sizeX, int sizeY)
{
    QVector<cv::Mat> out(cards.size());
    for (int count=0; count<cards.size(); count++)
    {
        const cv::Mat & mat = cards[count];
        out[count] = uniformSize(mat, sizeX, sizeY);
    }
    return out;
}

//******************************************************************************************
/*!
 * \brief CardDetector::extractObjects Extracts objects from the card
 * \param card
 * \return
 */
void CardDetector::extractObjects(const cv::Mat &card, QVector<std::vector<cv::Point> > * objectContours, QVector<cv::Mat> * objectMasks)
{
    if (!objectContours)
    {
        SD_TRACE("CardDetector::extractObjects : ObjectContours is null");
        return;
    }

    cv::Size uniSize = card.size();
    cv::Mat procImage;
    card.copyTo(procImage);
    if (procImage.channels() > 1)
    {
        cv::cvtColor(procImage, procImage, cv::COLOR_BGR2GRAY);
    }

    // Median filter
    cv::medianBlur(procImage, procImage, 5);
    if (_verbose) ImageCommon::displayMat(procImage, true, "Median");

//    // Enhance contours :
//    ImageFiltering::enhance(procImage, procImage);
//    if (_verbose) ImageCommon::displayMat(procImage, true, "Enhance");

    // Canny
    cv::Canny(procImage, procImage, 20, 150);
    if (_verbose) ImageCommon::displayMat(procImage, true, "Canny");

    // Morpho
    cv::Mat k1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
    cv::morphologyEx(procImage, procImage, cv::MORPH_CLOSE, k1, cv::Point(1, 1), 2);
    if (_verbose) ImageCommon::displayMat(procImage, true, "Morpho");

    // Find contours
    std::vector< std::vector<cv::Point> > contours;
    cv::findContours(procImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    objectContours->clear();
    objectContours->resize(contours.size());


    int maxArea = 0.25 * M_PI * uniSize.width * uniSize.height;
    int minArea = 16;
    //    int minArea = 0.25 * M_PI * objMinSize*objMinSize;
    int roiRadius = 0.45 * uniSize.width;
    if (_verbose) SD_TRACE(QString("Roi radius : %1").arg(roiRadius));
    //    int maxLength = 0.95*uniSize.width * M_PI;
    if (_verbose) SD_TRACE(QString("Contours count : %1").arg(contours.size()));

    int count=0;
    for (size_t i=0;i<contours.size();i++)
    {
        std::vector<cv::Point> contour = contours[i];
        //        double p = cv::arcLength(contour, true);
        //        if (p < maxLength)
        //        {
        //            double a = cv::contourArea(contour, true);
        cv::Rect brect = cv::boundingRect(contour);
        //                SD_TRACE4("Contour brect: %1, %2, %3, %4", brect.x, brect.y, brect.br().x, brect.br().y);
        int a = brect.area();
        int dx = brect.tl().x + brect.width/2 - uniSize.width/2;
        int dy = brect.tl().y + brect.height/2 - uniSize.height/2;
        int maxdim = qMax(brect.width, brect.height);
        // Select contour such that :
        // a) bounding rect of the contour larger min area and smaller 1/4 of card size
        // b) distance between center of the contour and the card center is smaller than card radius
        // c) max dimension of contour is smaller than card radius
        // d) contour brect should not touch (+/- 1 pixel) image boundaries
        if (a > minArea && a < maxArea &&
                dx*dx + dy*dy < roiRadius*roiRadius &&
                maxdim < roiRadius &&
                brect.x > 1 && brect.y > 1 &&
                brect.br().x < uniSize.width-2 &&  brect.br().y < uniSize.height)
        {
            (*objectContours)[count].swap(contour);
            count++;
        }
        //        }
    }
    objectContours->resize(count);

    // order by size (descending)
    std::sort(objectContours->begin(), objectContours->end(), Compare(Compare::Greater));

    if (_verbose) SD_TRACE(QString("Selected contours count : %1").arg(count));
    if (_verbose) ImageCommon::displayContour(objectContours->toStdVector(), card, false, true);

    // Draw filled contours as segmented image:
    if (objectMasks) {
        objectMasks->clear();
        objectMasks->resize(count);
        std::vector<std::vector<cv::Point> > out = objectContours->toStdVector();
        for(int idx=0; idx < count; idx++)
        {
            (*objectMasks)[idx] = cv::Mat(procImage.rows, procImage.cols, CV_8U, cv::Scalar::all(0));
            cv::Scalar color( 255 );
            cv::drawContours( (*objectMasks)[idx], out, idx, color, 1);
        }
    }
}

//******************************************************************************************

}
