
// Std
#include <iostream>
#include <vector>

// Qt
#include <QString>
#include <qmath.h>

// Opencv
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// Project
#include "Global.h"
#include "ImageCommon.h"
#include "ImageFiltering.h"

QString PATH = "/Users/vfomin/Documents/Qt_examples/DoubleGameVision/Data";

bool VERBOSE = false;


/*!
 * OK - Find circles and mask their content = Find cards
 * OK - Rectify circles and its content = Rectify card geometry
 * - Extract objects from each circle = Extract objects from each card
 *  -- Features : Hu moments
 *
 * - Compare objects
*/




QVector<cv::Mat> detectCards(const cv::Mat & src, int cardSizeMin, int cardSizeMax, bool verbose);

bool less(const std::vector<cv::Point> & c1, const std::vector<cv::Point> & c2)
{
    cv::Rect b1 = cv::boundingRect(c1);
    cv::Rect b2 = cv::boundingRect(c2);
    return b1.area() < b2.area();
}


int main()
{


    // ----- LOAD IMAGES

//    cv::Mat inImage = cv::imread(PATH.toStdString() + "/IMG_20150602_185233.jpg", cv::IMREAD_GRAYSCALE);
//    cv::Mat inImage = cv::imread(PATH.toStdString() + "/IMG_20150602_185401.jpg", cv::IMREAD_GRAYSCALE);
//    cv::Mat inImage = cv::imread(PATH.toStdString() + "/IMG_20150602_185420.jpg", cv::IMREAD_GRAYSCALE);
    cv::Mat inImage = cv::imread(PATH.toStdString() + "/IMG_20150602_185429.jpg",cv::IMREAD_GRAYSCALE);

    ImageCommon::displayMat(inImage, true, "Input image");



    int cardSizeMin = 100;
    int cardSizeMax = 400;

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


    QVector<cv::Mat> cards = detectCards(procImage, cardSizeMin, cardSizeMax, VERBOSE);
    if (cards.isEmpty())
    {
        SD_ERR("No cards found");
        return 0;
    }

// DEBUG:
//    for (int count=0; count<cards.size();count++)
//    {
//        const cv::Mat & card = cards.at(count);
//        ImageCommon::displayMat(card, true, QString("Card %1").arg(count+1));
//    }

    // ---- UNIFY SIZE OF THE CARDS
    int nbCards = cards.size();
    int maxDim = 250;
//    int maxDim = qMax(cards[0].cols, cards[0].rows);
//    for (int count=1; count<nbCards;count++)
//    {
//        const cv::Mat & card = cards.at(count);
//        if (maxDim < card.cols)
//        {
//            maxDim = card.cols;
//        }
//        if (maxDim < card.rows)
//        {
//            maxDim = card.rows;
//        }
//    }

    cv::Size uniSize(maxDim, maxDim);

    SD_TRACE(QString("Max size : %1, %2").arg(uniSize.width).arg(uniSize.height));

    // resize cards :
    QVector<cv::Mat> uniCards(nbCards);
    for (int count=0; count<nbCards;count++)
    {
        cv::resize(cards[count], uniCards[count],uniSize);
//        ImageCommon::displayMat(uniCards[count], true, QString("Card %1").arg(count+1));
    }

    // ---- EXTRACT OBJECT FEATURES FROM EACH CARD
    cv::Mat card = uniCards[1];
    ImageCommon::displayMat(card, true, QString("Card"));

    {

//    int objMinSize = uniSize.width*0.1;

    VERBOSE = true;
    cv::Mat procImage;
    card.copyTo(procImage);

    if (procImage.channels() > 1)
    {
        cv::cvtColor(procImage, procImage, cv::COLOR_BGR2GRAY);
    }

    // Canny
    cv::Canny(procImage, procImage, 20, 150);
    if (VERBOSE) ImageCommon::displayMat(procImage, true, "Canny");

    // Morpho
    cv::Mat k1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
    cv::morphologyEx(procImage, procImage, cv::MORPH_CLOSE, k1);
    if (VERBOSE) ImageCommon::displayMat(procImage, true, "Morpho");


    // Find contours
    std::vector< std::vector<cv::Point> > contours;
    std::vector< std::vector<cv::Point> > out;
    cv::findContours(procImage, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    int maxArea = 0.25 * M_PI * uniSize.width * uniSize.height;
    int minArea = 16;
//    int minArea = 0.25 * M_PI * objMinSize*objMinSize;
    int roiRadius = 0.45 * uniSize.width;
    if (VERBOSE) SD_TRACE(QString("Roi radius : %1").arg(roiRadius));
//    int maxLength = 0.95*uniSize.width * M_PI;

    if (VERBOSE) SD_TRACE(QString("Contours count : %1").arg(contours.size()));
    for (size_t i=0;i<contours.size();i++)
    {
        std::vector<cv::Point> contour = contours[i];
//        double p = cv::arcLength(contour, true);
//        if (p < maxLength)
//        {
//            double a = cv::contourArea(contour, true);
            cv::Rect brect = cv::boundingRect(contour);
            int a = brect.area();
            int dx = brect.tl().x + brect.width/2 - uniSize.width/2;
            int dy = brect.tl().y + brect.height/2 - uniSize.height/2;
            int maxdim = qMax(brect.width, brect.height);
            if (a > minArea && a < maxArea &&
                    dx*dx + dy*dy < roiRadius*roiRadius &&
                    maxdim < roiRadius)
            {
                out.push_back(contour);
            }

//        }
    }

    if (VERBOSE) SD_TRACE(QString("Selected contours count : %1").arg(out.size()));
    if (VERBOSE) ImageCommon::displayContour(out, card, false);

    // sort by size :
//    std::sort(out.begin(), out.end(), less);

//    QList< std::vector<double> > objectFeatures;
//    QList<QRect> objects;
//    for (size_t i=out.size()-1;i>0;i--)
//    {
//        cv::Moments ms = cv::moments(out[i]);
//        double hu[7];
//        cv::HuMoments(ms, hu);
//        cv::Rect b = cv::boundingRect(out[i]);
//        SD_TRACE(QString("Contour area : %1").arg(b.area()));
//    }


    }




    return 0;
}

QVector<cv::Mat> detectCards(const cv::Mat & src, int cardSizeMin, int cardSizeMax, bool verbose)
{
    cv::Mat procImage;
    // to gray scale and smooth
    if (src.channels() > 1)
    {
        cv::Mat out;
        cv::cvtColor(src, out, cv::COLOR_BGR2GRAY);
        procImage = out;
    }
    else
    {
        src.copyTo(procImage);
    }


    double fs = 0.20;
    cv::Size size(fs*procImage.cols, fs*procImage.rows);
    double sigmaX = size.width*0.25;
    double sigmaY = size.height*0.25;
    cv::Mat freqMask = ImageFiltering::getGaussianKernel2D(size, sigmaX, sigmaY);
    ImageFiltering::freqFilter(procImage, procImage, freqMask);

    if (VERBOSE) ImageCommon::displayMat(procImage, true, "Low freq image");

//    double f = 5.0;
//    ImageFiltering::simplify(procImage, procImage, f);
//    ImageCommon::displayMat(procImage, true, "Resized");


    // Derivate
    cv::Mat t1, t2, t3;
    cv::Scharr(procImage, t1, CV_32F, 1, 0);
    cv::Scharr(procImage, t2, CV_32F, 0, 1);
    if (VERBOSE) ImageCommon::displayMat(t1, true, "Scharr x");
    if (VERBOSE) ImageCommon::displayMat(t2, true, "Scharr y");

    cv::magnitude(t1, t2, t3);
    double minVal, maxVal;
    cv::minMaxLoc(t3, &minVal, &maxVal);
    double a = 255.0/(maxVal-minVal);
    double b = -255.0 * minVal/(maxVal-minVal);
    t3.convertTo(procImage, CV_8U, a, b);
    if (VERBOSE) ImageCommon::displayMat(procImage, true, "Contours");


    // Threshold
    double t = 255*0.4;
    cv::threshold(procImage, procImage, t, 255, cv::THRESH_BINARY);
    if (VERBOSE) ImageCommon::displayMat(procImage, true, "Threshold");

    // Morpho
    cv::Mat k1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
//    cv::Mat k2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7,7));
    cv::morphologyEx(procImage, procImage, cv::MORPH_OPEN, k1);
//    cv::morphologyEx(procImage, procImage, cv::MORPH_CLOSE, k2);
    if (VERBOSE) ImageCommon::displayMat(procImage, true, "Morpho");


    // Find contours
    std::vector< std::vector<cv::Point> > contours;
    std::vector< std::vector<cv::Point> > out;
    cv::findContours(procImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    int minArea = cardSizeMin*cardSizeMin;
    int maxArea = cardSizeMax*cardSizeMax;
    if (VERBOSE) SD_TRACE(QString("Min area : %1").arg(minArea));
    if (VERBOSE) SD_TRACE(QString("Contours count : %1").arg(contours.size()));
    for (size_t i=0;i<contours.size();i++)
    {
        std::vector<cv::Point> contour = contours[i];
        cv::Rect brect = cv::boundingRect(contour);
        int a = brect.area();
        if (a > minArea &&
                a < maxArea &&
                ImageCommon::isCircleLike(contour))
        {
            out.push_back(contour);
        }
    }

    if (VERBOSE) SD_TRACE(QString("Selected contours count : %1").arg(out.size()));
    if (VERBOSE) ImageCommon::displayContour(contours, procImage);


    QVector<cv::Mat> cards(out.size());

    double marginFactor = 0.03;
    for (size_t i=0; i<out.size(); i++)
    {
        cv::Rect brect = cv::boundingRect(out[i]);

        brect.x += brect.width * marginFactor;
        brect.y += brect.height * marginFactor;
        brect.width -= brect.width * marginFactor * 2;
        brect.height -= brect.height * marginFactor * 2;

        cv::Mat mask = ImageFiltering::getCircleKernel2D(brect.size(), 1, CV_8U);
        std::vector<cv::Mat> mx(src.channels());
        for (int i=0;i<src.channels();i++)
        {
            mx[i] = mask;
        }
        cv::Mat t, mask2;
        cv::merge(mx, mask2);
        t = src(brect).mul(mask2);
        t.copyTo(cards[i]);
        if (VERBOSE) ImageCommon::displayMat(cards[i], true, QString("Card %1").arg(i));
    }

    return cards;
}






#if 0
    // IMAGE SIMULATION
    cv::Mat in(450, 450, CV_8U, cv::Scalar::all(0));

    {

        cv::Mat c1 = ImageFiltering::getCircleKernel2D(cv::Size(40, 40));
        cv::Mat c2 = ImageFiltering::getCircleKernel2D(cv::Size(50, 40));
        cv::Mat c3 = ImageFiltering::getCircleKernel2D(cv::Size(60, 70));
        cv::Mat c4 = ImageFiltering::getCircleKernel2D(cv::Size(55, 55));

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


    cv::Mat mask = ImageFiltering::detectCircles(in, 55);

//    cv::Laplacian(in, in, CV_8U);
//    ImageCommon::displayMat(in, true, "Laplacian");

//    cv::blur(in, in, cv::Size(5,5));
//    ImageCommon::displayMat(in, true, "Blur");

//    cv::Mat t1, t2, out;
//    cv::Mat templ = ImageFiltering::getCircleKernel2D(cv::Size(55, 55));
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
    cv::Mat freqMask = ImageFiltering::getGaussianKernel2D(cv::Size(size, size), sigma, sigma);
    ImageFiltering::freqFilter(t1, t2, freqMask);
    ImageCommon::displayMat(t2, true, "Low freq image");

    //    int extSize = 2*80;
    //    double extSigma = extSize*0.25;
    //    cv::Mat freqMaskExt = ImageFiltering::getGaussianKernel2D(cv::Size(extSize, extSize), extSigma, extSigma);
    //    cv::pow(freqMaskExt,2.0,freqMaskExt);
    //    ImageFiltering::freqFilter(t1, t2, freqMaskExt, false);
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
    cv::Mat cardTemplate = ImageFiltering::getCircleKernel2D(cardSizeMin, cardSizeMin);
    cv::matchTemplate(procImage, cardTemplate,out,cv::TM_CCORR_NORMED);

    ImageCommon::displayMat(out, true, "Match template");

#endif


#if 0

    std::vector<cv::Vec3f> circles;
    ImageFiltering::detectCircles(procImage, circles, cardSizeMin/2, cardSizeMax/2, 0.4);

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
