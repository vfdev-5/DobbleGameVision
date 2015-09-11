
// Opencv
#include <opencv2/imgproc.hpp>

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

