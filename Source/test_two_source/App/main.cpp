
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

// Project
#include "CardDetector.h"
#include "BasicPairsDetector.h"
#include "Core/Global.h"
#include "Core/ImageCommon.h"
#include "Core/ImageProcessing.h"



bool VERBOSE = true;

/*!
 * OK - Find circles and mask their content = Find cards
 * OK - Rectify circles and its content = Rectify card geometry
 * OK - Extract objects from each circle = Extract objects from each card
 *  -- Features : Hu moments
 *  -- Or directly use 'Match Shapes' function
 * - Compare objects
*/

void help()
{
    SD_TRACE("Usage : DGVApp image_data_path");
    SD_TRACE("  where image_data_path is a path with *.jpg, *.png, *.tif images");
    SD_TRACE("Example : DGVApp C:/Temp/");
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
    QStringList filesToOpen = QStringList() << files[1];
#endif

    double cardSizeMinRatio = 0.0;
    double cardSizeMaxRatio = 1.0;
    DGV::CardDetector cardDetector(cardSizeMinRatio, cardSizeMaxRatio, VERBOSE);

    // Loop on files :
    foreach (QString file, filesToOpen)
    {
        SD_TRACE1("Open file '%1'", file);
        QString f = path + "/" + file;
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

        QVector<std::vector<cv::Point> > cardContours;
        ImageProcessing::detectObjects(procImage, &cardContours,
                                       0.1, 0.7,
                                       ImageProcessing::ANY,
                                       true);

        return 0;

        QVector<cv::Mat> cards = cardDetector.detectCards(procImage);
        if (cards.isEmpty())
        {
            SD_TRACE("No cards found");
            return 0;
        }

        // ---- UNIFY SIZE OF THE CARDS
        int uniDim = qMax(procImage.rows, procImage.cols)*(cardSizeMinRatio + cardSizeMaxRatio)/2.0;


        SD_TRACE(QString("Uniform size : %1, %2").arg(uniDim).arg(uniDim));
        QVector<cv::Mat> uniCards = cardDetector.uniformSize(cards, uniDim);

        // ---- EXTRACT OBJECTS AND MATCH SHAPES BETWEEN TWO CARD

        DGV::PairsDetector * pairsDetector = new DGV::BasicPairsDetector(0.29, 10, false);

        // VERBOSE = true;
        while (!uniCards.isEmpty())
        {

            // TAKE ONE CARD
            cv::Mat cardOne = uniCards.takeFirst();
            if (uniCards.isEmpty()) break;

            QVector<std::vector<cv::Point> > oContours1;
            cardDetector.extractObjects(cardOne, &oContours1);
            ImageCommon::displayContour(oContours1.toStdVector(), cardOne, false, true, "Card 1");

            // TAKE ANOTHER CARD
            foreach (cv::Mat cardTwo, uniCards)
            {

                QVector<std::vector<cv::Point> > oContours2;
                cardDetector.extractObjects(cardTwo, &oContours2);
                ImageCommon::displayContour(oContours2.toStdVector(), cardTwo, false, true, "Card 2");


                // LOOP ON THE OBJECTS FROM THE CARD ONE:
                StartTimer("Compare two cards");
                int matchIndices[2] = {-1, -1};
                for (int i=0;i<oContours1.size();i++)
                {
                    cv::Mat objectOneMask = cardDetector.getObjectMask(cardOne, oContours1[i]);
                    if (!pairsDetector->setupRefObject(cardOne, objectOneMask))
                    {
                        SD_TRACE1("Failed to setup reference object %1", i);
                        continue;
                    }

                    bool matchFound=false;

                    // LOOP ON THE OBJECTS OF THE CARD TWO
                    for (int j=0;j<oContours2.size();j++)
                    {

                        cv::Mat objectTwoMask = cardDetector.getObjectMask(cardTwo, oContours2[j]);
                        matchFound = pairsDetector->matchWithRefObject(cardTwo, objectTwoMask);


                        // IF MATCH IS FOUND -> NO NEED TO COMPARE THESE CARDS
                        if (matchFound)
                        {
                            matchIndices[0] = i;
                            matchIndices[1] = j;
                            break;
                        }
                    }
                    // Stop compare other objects if match is found
                    if (matchFound)
                    {
                        break;
                    }
                }
                StopTimer();

                if (matchIndices[0] >= 0 && matchIndices[1] >= 0)
                {
                    SD_TRACE2("Match found between object %1 on the 1st card and object %2 on the second card", matchIndices[0], matchIndices[1]);
                    cv::Mat objectOneMask = cardDetector.getObjectMask(cardOne, oContours1[matchIndices[0]]);
                    cv::Mat objectTwoMask = cardDetector.getObjectMask(cardTwo, oContours2[matchIndices[1]]);
                    ImageCommon::displayMat(cardOne.mul(objectOneMask), false, "Matched object 1", false);
                    ImageCommon::displayMat(cardTwo.mul(objectTwoMask), false, "Matched object 2", true);
                } else {
                    SD_TRACE("No matches found");
                }
            }
        }

        delete pairsDetector;

        return 0;
    }

}

// *******************************************************
// ***********
// *********** TESTS TO REMOVE ***************************
// ***********
// *******************************************************


#if 0
// IMAGE SIMULATION
cv::Mat in(450, 450, CV_8U, cv::Scalar::all(0));

{

cv::Mat c1 = ImageProcessing::getCircleKernel2D(cv::Size(40, 40));
cv::Mat c2 = ImageProcessing::getCircleKernel2D(cv::Size(50, 40));
cv::Mat c3 = ImageProcessing::getCircleKernel2D(cv::Size(60, 70));
cv::Mat c4 = ImageProcessing::getCircleKernel2D(cv::Size(55, 55));

c1.copyTo(in(cv::Rect(50, 60, 40, 40)));
c2.copyTo(in(cv::Rect(100, 20, 50, 40)));
c3.copyTo(in(cv::Rect(20, 100, 60, 70)));
c4.copyTo(in(cv::Rect(100,100,55,55)));

cv::Mat noise(in.rows, in.cols, in.type());
cv::Mat noise2(in.rows, in.cols, in.type());
cv::randn(noise, 150, 15);
cv::randn(noise2, 122, 20);

in = in - noise + noise2;
}
ImageCommon::displayMat(in, true, "Circle");


cv::Mat mask = ImageProcessing::detectCircles(in, 55);

//    cv::Laplacian(in, in, CV_8U);
//    ImageCommon::displayMat(in, true, "Laplacian");

//    cv::blur(in, in, cv::Size(5,5));
//    ImageCommon::displayMat(in, true, "Blur");

//    cv::Mat t1, t2, out;
//    cv::Mat templ = ImageProcessing::getCircleKernel2D(cv::Size(55, 55));
//    cv::matchTemplate(in,templ,t1,cv::TM_CCOEFF_NORMED);
//    ImageCommon::displayMat(t1, true, "Corr");
//    cv::threshold(t1, t2, 0.5, 255, CV_THRESH_BINARY);

//    t2.convertTo(out, CV_8U);
////    out = cv::Mat(in.rows, in.cols, in.type(), cv::Scalar::all(0));
////    cv::Rect r(templ.cols/2, templ.rows/2, t1.cols, t1.rows);
////    t2.copyTo(out(r));

//    ImageCommon::displayMat(out, true, "Corr");

//    std::vector< std::vector<cv::Point> > contours;
//    cv::Point offset(templ.cols/2, templ.rows/2);
//    cv::findContours(out, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, offset);

//    cv::Mat mask = cv::Mat(in.rows, in.cols, in.type(), cv::Scalar::all(0));

//    for (uint i=0; i<contours.size();i++)
//    {
//        std::vector<cv::Point> contour = contours[i];
//        cv::Rect brect = cv::boundingRect(contour);
//        cv::Rect r(brect.tl().x, brect.tl().y, templ.cols, templ.rows);
//        cv::Mat m = templ/255 * (i+1);
//        m.copyTo(mask(r));
//    }

ImageCommon::displayMat(mask, true, "mask");

ImageCommon::displayMat(in.mul(mask), true, "Circle detection");
return 0;
#endif

#if 0
// Sharpen
cv::Mat t1, t2;

cv::Laplacian(procImage, t1, CV_8U,3);
ImageCommon::displayMat(t1, true, "Laplacian");


int size = 5*80;
double sigma = size*0.25;
//    cv::Mat freqMask = cv::Mat::ones(240, 240, CV_32F);
cv::Mat freqMask = ImageProcessing::getGaussianKernel2D(cv::Size(size, size), sigma, sigma);
ImageProcessing::freqFilter(t1, t2, freqMask);
ImageCommon::displayMat(t2, true, "Low freq image");

//    int extSize = 2*80;
//    double extSigma = extSize*0.25;
//    cv::Mat freqMaskExt = ImageProcessing::getGaussianKernel2D(cv::Size(extSize, extSize), extSigma, extSigma);
//    cv::pow(freqMaskExt,2.0,freqMaskExt);
//    ImageProcessing::freqFilter(t1, t2, freqMaskExt, false);
//    ImageCommon::displayMat(t2, true, "High freq image");

procImage = t2 - 0.5*t1;
ImageCommon::displayMat(procImage, true, "Image sharpening");

double minVal, maxVal;
cv::minMaxLoc(procImage, &minVal,&maxVal);
//    double tVal = minVal + 0.1 * (maxVal-minVal);
//    cv::threshold(procImage, procImage, tVal, 255, CV_THRESH_BINARY_INV);
//    ImageCommon::displayMat(procImage, true, "Thresholded");

//    cv::Mat t3;
//    procImage.convertTo(t3, CV_32F, 1.0/(maxVal-minVal), -minVal/(maxVal-minVal));
cv::absdiff(maxVal, procImage, procImage);
ImageCommon::displayMat(procImage, true, "Inverted Image sharpening");

#endif

#if 0

// Find cards = find circles and mask their content
// Smooth
cv::blur(procImage, procImage, cv::Size(3,3));
ImageCommon::displayMat(procImage, true, "Blurred");

std::vector<cv::Vec3f> circles;
int minDist = cardSizeMin;
cv::HoughCircles(procImage, circles, cv::HOUGH_GRADIENT,
                 1, minDist, 150, 70, cardSizeMin/2, cardSizeMax);

// display result :
cv::Mat img;
procImage.copyTo(img);
for( size_t i = 0; i < circles.size(); i++ )
{
    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    int radius = cvRound(circles[i][2]);
    // draw the circle center
    cv::circle( img, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
    // draw the circle outline
    cv::circle( img, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
}
cv::imshow( "circles", img );
cv::waitKey(0);

#endif


#if 0
cv::Mat out;
cv::Mat cardTemplate = ImageProcessing::getCircleKernel2D(cardSizeMin, cardSizeMin);
cv::matchTemplate(procImage, cardTemplate,out,cv::TM_CCORR_NORMED);

ImageCommon::displayMat(out, true, "Match template");

#endif


#if 0

std::vector<cv::Vec3f> circles;
ImageProcessing::detectCircles(procImage, circles, cardSizeMin/2, cardSizeMax/2, 0.4);

// display result :
cv::Mat img;
procImage.copyTo(img);
for( size_t i = 0; i < circles.size(); i++ )
{
    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    int radius = cvRound(circles[i][2]);
    // draw the circle center
    cv::circle( img, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
    // draw the circle outline
    cv::circle( img, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
}
cv::imshow( "circles", img );
cv::waitKey(0);

#endif
