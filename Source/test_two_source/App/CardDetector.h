#ifndef CARDDETECTOR_H
#define CARDDETECTOR_H

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
    PROPERTY_ACCESSORS(int, minSize, getMinSize, setMinSize)
    PROPERTY_ACCESSORS(int, maxSize, getMaxSize, setMaxSize)
    PROPERTY_ACCESSORS(bool, verbose, isVerbose, setVerbose)
public:
    CardDetector(int minSize, int maxSize, bool verbose=false);

    QVector<cv::Mat> detectCards(const cv::Mat & src);
    cv::Mat uniformSize(const cv::Mat & src, int sizeX, int sizeY=0);
    QVector<cv::Mat> uniformSize(const QVector<cv::Mat> & src, int sizeX, int sizeY=0);


protected:

};

//******************************************************************************************

}

#endif // CARDDETECTOR_H
