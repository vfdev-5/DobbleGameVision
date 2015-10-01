
// Opencv
#include <opencv2/imgproc.hpp>

// Project
#include "Core/ImageCommon.h"

// Tests
#include "Common.h"
#include "Core/Global.h"

namespace Tests {

//*************************************************************************

cv::Mat generateSimpleGeometries()
{

    cv::Mat out(500, 600, CV_8U, cv::Scalar::all(0));
    cv::circle(out, cv::Point(20, 20), 15, cv::Scalar::all(150), CV_FILLED);

    cv::circle(out, cv::Point(60, 60), 30, cv::Scalar::all(160), CV_FILLED);

    cv::circle(out, cv::Point(180, 180), 60, cv::Scalar::all(170), CV_FILLED);


    cv::rectangle(out, cv::Rect(500, 405, 50, 35), cv::Scalar::all(120), CV_FILLED);

    cv::ellipse(out, cv::Point(100, 350), cv::Size(40, 70), 30, 0, 360, cv::Scalar::all(200), CV_FILLED);

    cv::line(out, cv::Point(260, 30), cv::Point(340, 56), cv::Scalar::all(100), 10);

    cv::rectangle(out, cv::Rect(250, 250, 100, 42), cv::Scalar::all(75), CV_FILLED);
    cv::rectangle(out, cv::Rect(270, 220, 51, 110), cv::Scalar::all(75), CV_FILLED);

    cv::ellipse(out, cv::Point(500, 150), cv::Size(100, 120), 30, 23, 69, cv::Scalar::all(120), CV_FILLED);

    return out;
}

//*************************************************************************

cv::Mat generateEllipseLikeGeometries()
{
    cv::Mat out(500, 600, CV_8U, cv::Scalar::all(0));

    cv::circle(out, cv::Point(80, 80), 15, cv::Scalar::all(150), CV_FILLED);
    cv::line(out, cv::Point(65, 65), cv::Point(95, 95), cv::Scalar::all(150), 5);
    cv::line(out, cv::Point(80, 65), cv::Point(80, 80), cv::Scalar::all(150), 7);
    cv::line(out, cv::Point(80, 80), cv::Point(60, 95), cv::Scalar::all(150), 3);


    cv::circle(out, cv::Point(400, 60), 30, cv::Scalar::all(160), CV_FILLED);
    cv::line(out, cv::Point(370, 25), cv::Point(430, 95), cv::Scalar::all(160), 5);
    cv::line(out, cv::Point(400, 20), cv::Point(400, 60), cv::Scalar::all(160), 7);
    cv::line(out, cv::Point(400, 60), cv::Point(380, 100), cv::Scalar::all(160), 3);


    cv::circle(out, cv::Point(180, 310), 60, cv::Scalar::all(170), CV_FILLED);
    cv::line(out, cv::Point(180, 240), cv::Point(180, 380), cv::Scalar::all(170), 5);
    cv::line(out, cv::Point(110, 310), cv::Point(250, 310), cv::Scalar::all(170), 5);


    cv::ellipse(out, cv::Point(400, 250), cv::Size(40, 70), -30, 0, 360, cv::Scalar::all(200), CV_FILLED);
    cv::line(out, cv::Point(400, 180), cv::Point(400, 310), cv::Scalar::all(200), 5);
    cv::line(out, cv::Point(350, 250), cv::Point(450, 250), cv::Scalar::all(200), 5);

    cv::ellipse(out, cv::Point(200, 190), cv::Size(72, 35), 10, 0, 360, cv::Scalar::all(200), CV_FILLED);

    cv::ellipse(out, cv::Point(320, 400), cv::Size(72, 35), 30, 0, 360, cv::Scalar::all(200), CV_FILLED);
    cv::ellipse(out, cv::Point(320, 400), cv::Size(35, 72), 40, 0, 360, cv::Scalar::all(200), CV_FILLED);

    return out;
}

//*************************************************************************

cv::Mat generateBigObjects()
{
    cv::Mat out(500, 600, CV_8U, cv::Scalar::all(70));

    // 2 big ellipses
    cv::ellipse(out, cv::Point(430, 350), cv::Size(70, 120), -30, 0, 360, cv::Scalar::all(200), CV_FILLED);
    cv::ellipse(out, cv::Point(200, 190), cv::Size(110, 95), 10, 0, 360, cv::Scalar::all(10), CV_FILLED);

    // random small objects
    int x, y;
    for (int i=0; i<200; i++)
    {
        x=i*2.5 + i/2 + i/20;
        y=qrand() & 500;
        cv::circle(out, cv::Point(x, y), 5 + qrand()&15, cv::Scalar::all(150), CV_FILLED);

        cv::line(out, cv::Point(y,x), cv::Point(y, x+10), cv::Scalar::all(250), 3);
    }

    return out;

}

//*************************************************************************

void addNoise(cv::Mat &image)
{
    if (image.channels() > 1)
    {
        SD_TRACE("addNoise : image should have single channel");
        return;
    }
    int initDepth=image.depth();
    if (initDepth < CV_32F)
        image.convertTo(image, CV_32F);

    cv::Mat noise(image.rows, image.cols, image.type());
    cv::randn(noise, 100, 25);
    image = image + noise;

    if (initDepth == CV_8U)
        ImageCommon::convertTo8U(image, image);
}

//*************************************************************************

//cv::Mat generateOtherGeometries()
//{

//    cv::Mat out(500, 600, CV_8U, cv::Scalar::all(0));
//    cv::circle(out, cv::Point(20, 20), 15, cv::Scalar::all(150), CV_FILLED);

//    cv::circle(out, cv::Point(60, 60), 30, cv::Scalar::all(160), CV_FILLED);

//    cv::circle(out, cv::Point(180, 180), 60, cv::Scalar::all(170), CV_FILLED);


//    cv::rectangle(out, cv::Rect(500, 405, 50, 35), cv::Scalar::all(120), CV_FILLED);

//    cv::ellipse(out, cv::Point(100, 350), cv::Size(40, 70), 30, 0, 360, cv::Scalar::all(200), CV_FILLED);

//    cv::line(out, cv::Point(260, 30), cv::Point(340, 56), cv::Scalar::all(100), 10);

//    cv::rectangle(out, cv::Rect(250, 250, 100, 42), cv::Scalar::all(75), CV_FILLED);
//    cv::rectangle(out, cv::Rect(270, 220, 51, 110), cv::Scalar::all(75), CV_FILLED);

//    cv::ellipse(out, cv::Point(500, 150), cv::Size(100, 120), 30, 23, 69, cv::Scalar::all(120), CV_FILLED);

//    return out;
//}

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

//*************************************************************************

}

