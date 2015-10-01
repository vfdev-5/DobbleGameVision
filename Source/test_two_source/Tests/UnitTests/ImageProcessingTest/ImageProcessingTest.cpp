
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
/*
void ImageProcessingTest::meanShiftTest()
{
//    cv::Mat in = generateBigObjects();

//    in.convertTo(in, CV_32F);
//    cv::Mat noise(in.rows, in.cols, in.type());
//    cv::randn(noise, 100, 25);
//    in = in + noise;
//    ImageCommon::convertTo8U(in, in);

//    ImageCommon::displayMat(in, true);

//    // Apply meanshift
//    double spatRadius = 15.0;
//    double colorRadius = 25.0;
//    int maxLevel=1;
//    cv::Mat out, in3c[] = {in, in, in}, in3;

//    cv::merge(in3c, 3, in3);

//    ImageCommon::displayMat(in3, true, "in3");

//    cv::pyrMeanShiftFiltering(in3, out, spatRadius, colorRadius, maxLevel);

//    ImageCommon::displayMat(out, true, "Meanshift");


//    // VERY SLOW

}
*/
//*************************************************************************
/*
void ImageProcessingTest::freqFilterTest()
{

    cv::Mat inImage = generateBigObjects();
    addNoise(inImage);

    ImageCommon::displayMat(inImage, true);


    // Hypothesis on frequency/object-size dependency
    // unit pulse (pulse duration = tau on total time T)-> cardinal sinus : sinc(pi*tau*f)
    // low-pass filter with fcut = 1.0/tau
    // fcut_index = fcut / df, df = 1.0/T


    int size[5] = {2, 3, 4, 5, 7};

    for (int i=0;i<5;i++)
    {
        cv::Mat procImg;

        cv::Size s(inImage.cols/size[i], inImage.rows/size[i]);
        SD_TRACE2("Freq mask size : %1, %2", s.width, s.height);
        cv::Mat freqMask = ImageProcessing::getCircleKernel2D(s, 1.0);

        ImageProcessing::freqFilter(inImage, procImg, freqMask);


        ImageCommon::displayMat(procImg, true, QString("FreqFiltering image : size=%1").arg(size[i]));
    }

}
*/
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

void ImageProcessingTest::detectObjectsTest3()
{

    cv::Mat in = generateBigObjects();
    addNoise(in);

    ImageCommon::displayMat(in, true);


    // Detect all objects :
    ImageProcessing::Contours objects;
    double minSizeRatio(0.2);
    double maxSizeRatio(0.95);
    ImageProcessing::DetectedObjectType type = ImageProcessing::ELLIPSE_LIKE;
    cv::Mat mask = cv::Mat();

    ImageProcessing::detectObjects(in, &objects,
                                   minSizeRatio, maxSizeRatio,
                                   mask, type, 0.7,
                                   true);

    //    SD_TRACE1("Object count = %1", objects.size());
//    QVERIFY(2 == objects.size());



}

//*************************************************************************

}

QTEST_MAIN(Tests::ImageProcessingTest)
