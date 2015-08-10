
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
#include "Core/ImageFiltering.h"


bool VERBOSE = true;

void help()
{
    SD_TRACE("Usage : Sandbox_NonlinearDiffusionFiltering_Example image_data_path");
    SD_TRACE("  where image_data_path is a path with *.jpg, *.png, *.tif images");
    SD_TRACE("Example : NonlinearDiffusionFiltering_Example C:/Temp/");
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
    QStringList filesToOpen = QStringList() << files[3];
#endif

    // Loop on files :
    foreach (QString file, filesToOpen)
    {
        SD_TRACE1("Open file '%1'", file);
        QString f = path + "/" + file;
        cv::Mat inImage = cv::imread(f.toStdString(), cv::IMREAD_GRAYSCALE);

        ImageCommon::displayMat(inImage, true, "Input image");

        cv::Mat procImg;
        ImageFiltering::nonlinearDiffusionFiltering(inImage, procImg);

//        ImageCommon::displayMat(procImg, true, "NDFiltering image");

    }

}
