
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
    in.convertTo(in, CV_32F);

    cv::Mat noise(in.rows, in.cols, in.type());
//    cv::randu(noise, 170, 90);
    cv::randn(noise, 100, 25);
    in = in + noise;

    ImageCommon::displayMat(in, true);

    // Enhance contours
    cv::Mat t;
    ImageProcessing::edgeStrength(in, t, 5);
//    ImageCommon::displayMat(t, true, "edge strength");
    in = in.mul(t);
    ImageCommon::convertTo8U(in, in);
    ImageCommon::displayMat(in, true, "In enchanced");


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

//    QVERIFY(16 == objects.size());

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

}

QTEST_MAIN(Tests::ImageProcessingTest)
