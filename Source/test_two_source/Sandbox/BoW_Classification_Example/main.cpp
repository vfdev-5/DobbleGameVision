
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
    SD_TRACE("Usage : BoWC_Example train_data_path test_data_path");
    SD_TRACE("  where image_data_path is a path with *.jpg, *.png, *.tif images");
    SD_TRACE("Example : BoWC_Example C:/Temp/");
}


QStringList fetchFiles(const QString & path)
{
    QStringList out;
    QDir d(path);
    if (!d.exists())
    {
        SD_TRACE1("Provided path '%1' is not found", path);
        return out;
    }

    QStringList files = d.entryList(QStringList() << "*.jpg" << "*.png" << "*.tif", QDir::Files);
    if (files.isEmpty())
    {
        SD_TRACE1("No images found at path '%1'", path);
        return out;
    }

#if 0
    out = files;
#else
    out = QStringList() << files[0] << files[1] << files[2];
#endif

    return out;
}



int main(int argc, char** argv)
{

    if (argc != 3)
    {
        help();
        return 0;
    }


    // ------------ TRAINING PHASE
    {
        // ----- LOAD IMAGES FROM PATH
        QString path(argv[1]);
        QStringList filesToOpen = fetchFiles(path);
        if (filesToOpen.empty())
        {
            help();
            return 1;
        }


        // Create feature detector and descriptor extractor:
        cv::Ptr<cv::ORB> detector = cv::ORB::create(150);
        cv::Mat trainingDescriptors(0, detector->descriptorSize(), detector->descriptorType());

        std::cout << "Training descriptor size & type : " << detector->descriptorSize() << ", " << detector->descriptorType() << std::endl;


        std::vector<cv::KeyPoint> keypoints;
        // Loop on files :
        std::cout << "Extract features from training images" << std::endl;
        foreach (QString file, filesToOpen)
        {
            SD_TRACE1("Open file '%1'", file);
            QString f = path + "/" + file;
            cv::Mat inImage = cv::imread(f.toStdString(), cv::IMREAD_GRAYSCALE);
            cv::Mat descriptors;
            detector->detectAndCompute(inImage, cv::Mat(), keypoints, descriptors);
            trainingDescriptors.push_back(descriptors);
            cv::Mat imageWithKeyPoints;
            cv::drawKeypoints(inImage, keypoints, imageWithKeyPoints);
//            ImageCommon::displayMat(imageWithKeyPoints, true, "Input image with keypoints");
            keypoints.clear();
        }

        std::cout << "Total number of descriptors : " << trainingDescriptors.rows << std::endl;

        trainingDescriptors.convertTo(trainingDescriptors, CV_32F);
        ImageCommon::printMat(trainingDescriptors, "trainingDescriptors");

        cv::BOWKMeansTrainer bowTrainer(15);
        bowTrainer.add(trainingDescriptors);
        cv::Mat vocabulary = bowTrainer.cluster();
        //ImageCommon::displayMat(vocabulary, true, "Vocabulary");


        // Train



    }

    // ------------ TESTING PHASE
    {
        // ----- LOAD IMAGES FROM PATH
        QString path(argv[1]);
        QStringList filesToOpen = fetchFiles(path);
        if (filesToOpen.empty())
        {
            help();
            return 1;
        }
    }




    return 0;
}
