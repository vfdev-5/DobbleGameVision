
// Std
#include <iostream>
#include <vector>

// Qt
#include <QString>
#include <QDir>
#include <QFile>

// Opencv
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/shape.hpp>
#include <opencv2/features2d.hpp>

// Project
#include "Core/Global.h"
#include "Core/ImageCommon.h"
#include "Core/ImageProcessing.h"


bool VERBOSE = true;

void help()
{
    SD_TRACE("Usage : FreqFiltering_Example image_data_path");
    SD_TRACE("  where image_data_path is a path with *.jpg, *.png, *.tif images");
    SD_TRACE("Example : FreqFiltering_Example C:/Temp/");
}

int main(int argc, char** argv)
{

    if (argc != 2)
    {
        help();
        return 0;
    }

    // ----- LOAD IMAGES FROM PATH
    QString path = QString(argv[1]);
    QDir d(path);
    if (!d.exists())
    {
        SD_TRACE1("Provided path '%1' is not found", path);
        return 1;
    }

    QStringList files = d.entryList(QStringList() << "*.jpg" << "*.png" << "*.tif", QDir::Files);
    if (files.isEmpty())
    {
        SD_TRACE1("No images found at path '%1'", path);
        help();
        return 1;
    }

#if 0
    QStringList filesToOpen = files;
#else
    QStringList filesToOpen = QStringList() << files[0];
#endif

    // Loop on files :
    foreach (QString file, filesToOpen)
    {
        SD_TRACE1("Open file '%1'", file);
        QString f = path + "/" + file;
        cv::Mat inImage = cv::imread(f.toStdString(), cv::IMREAD_GRAYSCALE);

        ImageCommon::displayMat(inImage, true, "Input image");

        // Hypothesis on frequency/object-size dependency
        // unit pulse (pulse duration = tau on total time T)-> cardinal sinus : sinc(pi*tau*f)
        // low-pass filter with fcut = 1.0/tau
        // fcut_index = fcut / df, df = 1.0/T


        int size[5] = {10, 20, 40, 80, 100};

        for (int i=0;i<5;i++)
        {
            cv::Mat procImg;

            cv::Size s(inImage.cols/size[i], inImage.rows/size[i]);
            SD_TRACE2("Freq mask size : %1, %2", s.width, s.height);
            cv::Mat freqMask = ImageProcessing::getCircleKernel2D(s, 1.0);

            ImageProcessing::freqFilter(inImage, procImg, freqMask);
            ImageCommon::displayMat(procImg, true, QString("FreqFiltering image : size=%1").arg(size[i]));
        }

//        // Compare with median filter
//        cv::medianBlur(inImage, procImg, 11);
//        ImageCommon::displayMat(procImg, true, "Median Filter image");

    }

}


// WRITE IMAGE:
//{
//    QString path = QString(argv[1]);
//    QDir d(path);
//    if (!d.exists())
//    {
//        SD_TRACE1("Provided path '%1' is not found", path);
//        return 1;
//    }

//    cv::Mat in(450, 450, CV_8U, cv::Scalar::all(0));

//    {

//        cv::Mat c1(40, 40, CV_8U, cv::Scalar::all(255));
//        cv::Mat c2(20, 40, CV_8U, cv::Scalar::all(255));
//        cv::Mat c3(20, 20, CV_8U, cv::Scalar::all(255));
//        cv::Mat c4(80, 80, CV_8U, cv::Scalar::all(255));

//        c1.copyTo(in(cv::Rect(50, 60, 40, 40)));
//        c2.copyTo(in(cv::Rect(100, 20, 40, 20)));
//        c3.copyTo(in(cv::Rect(20, 100, 20, 20)));
//        c4.copyTo(in(cv::Rect(100,100,80,80)));

//        //        cv::Mat noise(in.rows, in.cols, in.type());
//        //        cv::Mat noise2(in.rows, in.cols, in.type());
//        //        cv::randn(noise, 150, 15);
//        //        cv::randn(noise2, 122, 20);
//        //        in = in - noise + noise2;
//    }
//    ImageCommon::displayMat(in, true, "Squares");
//    QString f = path + "/squares.png";
//    cv::imwrite(f.toStdString(), in);
//}
