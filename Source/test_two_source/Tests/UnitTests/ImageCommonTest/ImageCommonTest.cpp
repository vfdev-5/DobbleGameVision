
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

//void ImageCommonTest::isCircleLikeTest()
//{
//    // create contours
//    cv::Mat in = generateSimpleGeometries(), inCopy;
//    in.copyTo(inCopy);

//    std::vector< std::vector<cv::Point> > contours;
//    cv::findContours(inCopy, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

////    bool result[] = {false, true, false, false, true, true, false, false};
//    std::vector< std::vector<cv::Point> >::iterator it = contours.begin();
//    for (int i=0;it!=contours.end();++it, i++)
//    {
//        std::vector< std::vector<cv::Point> > testContours;
//        testContours.push_back(*it);
//        ImageCommon::displayContours(testContours, in);
////        QVERIFY(result[i] == ImageCommon::isCircleLike(*it));
//        bool result = ImageCommon::isCircleLike(*it);
//        if (result)
//        {
//            SD_TRACE("Contous isCircleLikeTest : true");
//        }
//        else
//        {
//            SD_TRACE("Contous isCircleLikeTest : false");
//        }
//    }
//    QVERIFY(true);
//}

//*************************************************************************

}

QTEST_MAIN(Tests::ImageCommonTest)
