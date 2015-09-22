
// Std
#include <vector>

// Qt
#include <QDir>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Tests
#include "../../Common.h"
#include "Core/Global.h"
#include "Core/ImageCommon.h"
#include "Core/ImageProcessing.h"
#include "AppTest.h"


namespace Tests
{

//*************************************************************************

void AppTest::initTestCase()
{
    // ----- LOAD IMAGES FROM PATH
    _path = QString("/home/vfdev/Documents/DobbleGameVision_source/Data/Train/");
    QDir d(_path);

    QVERIFY(d.exists());

    QStringList files = d.entryList(QStringList() << "*.jpg" << "*.png" << "*.tif", QDir::Files);

    QVERIFY(!files.isEmpty());

#if 0
    _filesToOpen = files;
#else
    _filesToOpen = QStringList() << files[0] << files[1] << files[2] << files[3];
//    _filesToOpen = QStringList() << files[3];
#endif


}

//*************************************************************************

void AppTest::detectCardsTest()
{

    // Assume that a card should be larger than 1/8 of the image size and smaller than 1.0
    double cardSizeMinRatio = 0.2;
    double cardSizeMaxRatio = 1.0;

    // Loop on files :
    int results[] = {3,2};
    foreach (QString file, _filesToOpen)
    {
        SD_TRACE1("Open file '%1'", file);
        QString f = _path + "/" + file;
        cv::Mat inImage = cv::imread(f.toStdString(), cv::IMREAD_GRAYSCALE);

        ImageCommon::displayMat(inImage, true, "Input image");

        // Resize image
        cv::Mat procImage = inImage;
        int dim = qMax(procImage.rows, procImage.cols);
        int limit = 700;
        if (dim > limit)
        {
            cv::Mat out;
            double f = limit* 1.0 / dim;
            cv::resize(procImage, out, cv::Size(), f, f);
            procImage = out;
        }

        cv::Mat i0;
        procImage.copyTo(i0);

        // ---- FIND CARDS

        ImageProcessing::Contours cardContours;
        ImageProcessing::detectObjects(procImage, &cardContours,
                                       cardSizeMinRatio, cardSizeMaxRatio,
                                       cv::Mat(),
                                       ImageProcessing::ELLIPSE_LIKE, 1.5,
                                       true);

        SD_TRACE1("Object count = %1", cardContours.size());
//        QVERIFY();

    }


//    cv::Mat in = generateSimpleGeometries();

//    in.convertTo(in, CV_32F);
//    cv::Mat noise(in.rows, in.cols, in.type());
////    cv::randu(noise, 170, 90);
//    cv::randn(noise, 100, 25);
//    in = in + noise;
//    ImageCommon::convertTo8U(in, in);

////    ImageCommon::displayMat(in, true);

//    // Detect all objects :
//    ImageProcessing::Contours objects;

//    double minSizeRatio(0.0);
//    double maxSizeRatio(0.95);
//    ImageProcessing::DetectedObjectType type = ImageProcessing::ANY;
//    cv::Mat mask = cv::Mat();


//    ImageProcessing::detectObjects(in, &objects,
//                                   minSizeRatio, maxSizeRatio,
//                                   mask, type, 0.0,
//                                   false);
////    SD_TRACE1("Object count = %1", objects.size());
//    QVERIFY(8 == objects.size());

//    // Detect all ellipse-like objects:
//    type = ImageProcessing::ELLIPSE_LIKE;
//    ImageProcessing::detectObjects(in, &objects,
//                                   minSizeRatio, maxSizeRatio,
//                                   mask, type, 0.7,
//                                   false);

////    SD_TRACE1("Object count = %1", objects.size());
//    QVERIFY(4 == objects.size());

//    type = ImageProcessing::NOT_ELLIPSE_LIKE;
//    ImageProcessing::detectObjects(in, &objects,
//                                   minSizeRatio, maxSizeRatio,
//                                   mask, type, 0.7,
//                                   false);

////    SD_TRACE1("Object count = %1", objects.size());
//    QVERIFY(4 == objects.size());



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

}

QTEST_MAIN(Tests::AppTest)
