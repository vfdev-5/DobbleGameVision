
// Std
#include <vector>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Tests
#include "../../Common.h"
#include "Core/Global.h"
#include "Core/ImageCommon.h"
#include "ImageCommonTest.h"


namespace Tests
{

#define VERBOSE false

//*************************************************************************

void ImageCommonTest::isEllipseLikeTest()
{
    // create contours
    cv::Mat in = generateSimpleGeometries(), inCopy;
    in.copyTo(inCopy);

    std::vector< std::vector<cv::Point> > contours;
    cv::findContours(inCopy, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    bool result[] = {false, true, false, false, true, true, false, false};
    std::vector< std::vector<cv::Point> >::iterator it = contours.begin();
    for (int i=0;it!=contours.end();++it, i++)
    {
        std::vector< std::vector<cv::Point> > testContours;
        testContours.push_back(*it);
//        ImageCommon::displayContours(testContours, in);
        QVERIFY(result[i] == ImageCommon::isEllipseLike(*it));
    }

}

//*************************************************************************

void ImageCommonTest::isEllipseLike2Test1()
{
    // create contours
    cv::Mat in = generateSimpleGeometries(), inCopy;
    in.copyTo(inCopy);

    std::vector< std::vector<cv::Point> > contours;
    cv::findContours(inCopy, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    bool result[] = {false, true, false, false, true, true, false, true};
    std::vector< std::vector<cv::Point> >::iterator it = contours.begin();
    for (int i=0;it!=contours.end();++it, i++)
    {
        std::vector< std::vector<cv::Point> > testContours;
        testContours.push_back(*it);
//        ImageCommon::displayContours(testContours, in);
//        SD_TRACE1("Contour is ellipse like : %1", ImageCommon::isEllipseLike2(*it, 0.5));
        QVERIFY(result[i] == ImageCommon::isEllipseLike2(*it, 0.5));
    }
}

//*************************************************************************

void ImageCommonTest::isEllipseLike2Test2()
{
    // create contours
    cv::Mat in = generateEllipseLikeGeometries(), inCopy;
    in.copyTo(inCopy);

    std::vector< std::vector<cv::Point> > contours;
    cv::findContours(inCopy, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    bool result[] = {false, true, true, true, false, false};
    std::vector< std::vector<cv::Point> >::iterator it = contours.begin();
    for (int i=0;it!=contours.end();++it, i++)
    {
        std::vector< std::vector<cv::Point> > testContours;
        testContours.push_back(*it);
//        ImageCommon::displayContours(testContours, in);
//        SD_TRACE1("Contour is ellipse like : %1", ImageCommon::isEllipseLike2(*it, 0.7));
        QVERIFY(result[i] == ImageCommon::isEllipseLike2(*it, 0.7));
    }

}

//*************************************************************************

void ImageCommonTest::intersectWithEllipseTest()
{
    // NOTHING IS TESTED

    std::vector<cv::Point> points, output;
    int xs = 10;
    int ys = 20;
    for (int i=0;i<100;i++)
    {
        for (int j=0;j<100;j++)
        {
            points.push_back(cv::Point( xs + i, ys + j));
        }
    }

    ImageCommon::intersectWithEllipse(cv::Point(50,50), 30.0, 20.0, 10.0, points, output);

    std::vector< std::vector<cv::Point> > testContours;
    testContours.push_back(output);
//    ImageCommon::displayContours(testContours);
}

//*************************************************************************

}

QTEST_MAIN(Tests::ImageCommonTest)
