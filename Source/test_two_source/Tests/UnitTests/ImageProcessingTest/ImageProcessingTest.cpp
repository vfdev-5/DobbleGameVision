
// Std
#include <vector>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Tests
#include "../../Common.h"
#include "Core/Global.h"
#include "Core/ImageCommon.h"
#include "Core/ImageProcessing.h"
#include "ImageProcessingTest.h"


namespace Tests
{

#define VERBOSE false

//*************************************************************************

void ImageProcessingTest::detectObjectsTest1()
{

    cv::Mat in = generateSimpleGeometries();

    in.convertTo(in, CV_32F);
    cv::Mat noise(in.rows, in.cols, in.type());
//    cv::randu(noise, 170, 90);
    cv::randn(noise, 100, 25);
    in = in + noise;
    ImageCommon::convertTo8U(in, in);

//    ImageCommon::displayMat(in, true);

    // Detect all objects :
    ImageProcessing::Contours objects;

    double minSizeRatio(0.0);
    double maxSizeRatio(0.95);
    ImageProcessing::DetectedObjectType type = ImageProcessing::ANY;
    cv::Mat mask = cv::Mat();


    ImageProcessing::detectObjects(in, &objects,
                                   minSizeRatio, maxSizeRatio,
                                   mask, type, 0.0,
                                   false);
//    SD_TRACE1("Object count = %1", objects.size());
    QVERIFY(8 == objects.size());

    // Detect all ellipse-like objects:
    type = ImageProcessing::ELLIPSE_LIKE;
    ImageProcessing::detectObjects(in, &objects,
                                   minSizeRatio, maxSizeRatio,
                                   mask, type, 0.7,
                                   false);

//    SD_TRACE1("Object count = %1", objects.size());
    QVERIFY(4 == objects.size());

    type = ImageProcessing::NOT_ELLIPSE_LIKE;
    ImageProcessing::detectObjects(in, &objects,
                                   minSizeRatio, maxSizeRatio,
                                   mask, type, 0.7,
                                   false);

//    SD_TRACE1("Object count = %1", objects.size());
    QVERIFY(4 == objects.size());



//    // DEBUG
//    ImageCommon::displayContours(objects.toStdVector(), in, false, true);

//    ImageProcessing::Contours::iterator it = objects.begin();
//    for (int i=0;it!=objects.end();++it, i++)
//    {
//        std::vector< std::vector<cv::Point> > testContours;
//        testContours.push_back(*it);
//        ImageCommon::displayContours(testContours, in);
//    }

}

//*************************************************************************

void ImageProcessingTest::detectObjectsTest2()
{
    cv::Mat in = generateEllipseLikeGeometries();

    in.convertTo(in, CV_32F);
    cv::Mat noise(in.rows, in.cols, in.type());
//    cv::randu(noise, 170, 90);
    cv::randn(noise, 100, 25);
    in = in + noise;
    ImageCommon::convertTo8U(in, in);

//    ImageCommon::displayMat(in, true);

    // Detect all objects :
    ImageProcessing::Contours objects;

    double minSizeRatio(0.0);
    double maxSizeRatio(0.95);
    ImageProcessing::DetectedObjectType type = ImageProcessing::ANY;
    cv::Mat mask = cv::Mat();

    ImageProcessing::detectObjects(in, &objects,
                                   minSizeRatio, maxSizeRatio,
                                   mask, type, 0.0,
                                   false);
//    SD_TRACE1("Object count = %1", objects.size());
    QVERIFY(6 == objects.size());

    // Detect all ellipse-like objects:
    type = ImageProcessing::ELLIPSE_LIKE;
    ImageProcessing::detectObjects(in, &objects,
                                   minSizeRatio, maxSizeRatio,
                                   mask, type, 0.7,
                                   false);

//    SD_TRACE1("Object count = %1", objects.size());
    QVERIFY(3 == objects.size());

    type = ImageProcessing::NOT_ELLIPSE_LIKE;
    ImageProcessing::detectObjects(in, &objects,
                                   minSizeRatio, maxSizeRatio,
                                   mask, type, 0.7,
                                   false);

//    SD_TRACE1("Object count = %1", objects.size());
    QVERIFY(3 == objects.size());

}

//*************************************************************************

}

QTEST_MAIN(Tests::ImageProcessingTest)
