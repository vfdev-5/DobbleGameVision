
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
    double cardSizeMinRatio = 0.15;
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
//        SD_TRACE1("Object count = %1", cardContours.size());
//        QVERIFY();

        // ----- test
//        cv::Mat img2F;
//        procImage.convertTo(img2F, CV_32F);
//        cv::dft(img2F, img2F, cv::DFT_COMPLEX_OUTPUT | cv::DFT_SCALE);
//        img2F = ImageProcessing::fftShift(img2F);
//        ImageCommon::displayMat(img2F, true, "2d fft");

//        int sx = 0.15*procImage.cols;
//        int sy = 0.15*procImage.rows;
//        cv::Mat freqMask = ImageProcessing::getCutGaussianKernel2D(sx, sy, 0.0, 0.0, 0.3);
//        ImageCommon::displayMat(freqMask, true, "freqMask");
//        ImageProcessing::freqFilter(procImage, procImage, freqMask, true, true);
//        ImageCommon::displayMat(procImage, true, "fft filteres");

    }

}

//*************************************************************************

}

QTEST_MAIN(Tests::AppTest)
