#ifndef CARDDETECTOR_H
#define CARDDETECTOR_H

// Std
#include <vector>

// Qt
#include <QVector>

// Opencv
#include <opencv2/core.hpp>

// Project
#include "Core/Global.h"

namespace DGV
{

//******************************************************************************************

class CardDetector
{
    PROPERTY_ACCESSORS(double, minSizeRatio, getMinSizeRatio, setMinSizeRatio)
    PROPERTY_ACCESSORS(double, maxSizeRatio, getMaxSizeRatio, setMaxSizeRatio)
    PROPERTY_ACCESSORS(bool, verbose, isVerbose, setVerbose)
public:
    CardDetector(double minSizeRatio, double maxSizeRatio, bool verbose=false);

    QVector<cv::Mat> detectCards(const cv::Mat & src);
    cv::Mat uniformSize(const cv::Mat & card, int sizeX, int sizeY=0);
    QVector<cv::Mat> uniformSize(const QVector<cv::Mat> & cards, int sizeX, int sizeY=0);
    void extractObjects(const cv::Mat & card, QVector<std::vector<cv::Point> > * objectContours, QVector<cv::Mat> *objectMasks=0);
    cv::Mat getObject(const cv::Mat & card, const std::vector<cv::Point> & contour);
    cv::Mat getObjectMask(const cv::Mat & card, const std::vector<cv::Point> & contour);

protected:

};

//******************************************************************************************

}

#endif // CARDDETECTOR_H
