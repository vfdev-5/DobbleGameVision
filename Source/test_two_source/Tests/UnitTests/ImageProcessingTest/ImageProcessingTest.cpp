
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

void ImageProcessingTest::detectObjectsTest()
{

    cv::Mat in = generateSimpleGeometries();
    cv::Mat noise(in.rows, in.cols, in.type());
    cv::randu(noise, 170, 90);
    in = in + noise;

    ImageCommon::displayMat(in);


    cv::Mat t;
    ImageProcessing::edgeStrength(in, t);
    ImageCommon::displayMat(t, true, "edge strength");

//    cv::Mat t1, t2, t3;
//    int ksize = 3;
//    if (in.depth() < CV_32F)
//    {
//        in.convertTo(t1, CV_32F);
//    }
//    else
//    {
//        t1 = in;
//    }

//    cv::blur(t1, t1, cv::Size(ksize, ksize));
//    cv::Mat k(ksize, ksize, CV_8U, cv::Scalar::all(1));
//    cv::dilate(t1, t2, k);
//    cv::erode(t1, t3, k);
//    t2 = t1 - t2;
//    t3 = t1 - t3;

//    ImageCommon::displayMat(t3, true, "blur - erode");
//    ImageCommon::displayMat(t2, true, "blur - dilate");

    in = t;

    ImageProcessing::Contours objects;

    double minSizeRatio(0.05);
    double maxSizeRatio(0.85);
    ImageProcessing::DetectedObjectType type = ImageProcessing::ANY;
    cv::Mat mask = cv::Mat();
    bool verbose = true;

    ImageProcessing::detectObjects(in, &objects,
                                   minSizeRatio, maxSizeRatio,
                                   type, mask,
                                   verbose);

    SD_TRACE1("Object count = %1", objects.size());
//    QVERIFY(8==objects.size());
    ImageCommon::displayContours(objects.toStdVector(), in, false, true);

//    ImageProcessing::Contours::iterator it = objects.begin();
//    for (int i=0;it!=objects.end();++it, i++)
//    {
//        std::vector< std::vector<cv::Point> > testContours;
//        testContours.push_back(*it);
//        ImageCommon::displayContours(testContours, in);

//    }


}

//*************************************************************************

}

QTEST_MAIN(Tests::ImageProcessingTest)
