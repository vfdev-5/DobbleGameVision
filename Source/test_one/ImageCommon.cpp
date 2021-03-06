
// Std
#include <vector>

// Qt
#include <QMap>

// Opencv
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// Project
#include "Global.h"
#include "ImageCommon.h"

namespace ImageCommon {

//******************************************************************************************

cv::Mat displayMat(const cv::Mat & inputImage0, bool showMinMax, const QString & windowName, bool waitKey, bool rescale)
{
    QString windowNameS;
    if (windowName.isEmpty())
        windowNameS = "Input_Image";
    else
        windowNameS = windowName;

    if (inputImage0.empty())
        return cv::Mat();

    int depth = inputImage0.elemSize1();

    cv::Mat inputImage;
    if (inputImage0.depth() != CV_32F && inputImage0.depth() != CV_64F)
    {
        inputImage0.convertTo(inputImage, CV_32F);
    }
    else
        inputImage = inputImage0;

    int nbBands = inputImage0.channels();
    QMap<int,int> mapping;
    if (nbBands == 1)
    {
        mapping.insert(1, 1);
        mapping.insert(2, 1);
        mapping.insert(3, 1);
    }
    else if (nbBands == 2)
    {
        mapping.insert(1, 1);
        mapping.insert(2, 1);
        mapping.insert(3, 1);

        std::vector<cv::Mat> ic(nbBands);
        cv::split(inputImage, &ic[0]);
        cv::magnitude(ic[0], ic[1], inputImage);
        nbBands=1;
    }
    else if (nbBands >= 3)
    {
        mapping.insert(1, 3);
        mapping.insert(2, 2);
        mapping.insert(3, 1);
    }



    if (rescale)
    {
        // resize image if max dimension is larger displayLimit=1000 pixels
        int displayLimit=800;
        int maxdim = inputImage.rows > inputImage.cols ? inputImage.rows : inputImage.cols;
        if (maxdim > displayLimit)
        {
            SD_TRACE( "displayMat : " + windowNameS + ", Image size is rescaled" );
            double f = maxdim * 1.0 / displayLimit;
            cv::resize(inputImage, inputImage, cv::Size(0,0), 1.0/f, 1.0/f);
        }
    }

    cv::Mat outputImage8U;
    std::vector<cv::Mat> iChannels(nbBands);
    std::vector<cv::Mat> oChannels(mapping.size());
    cv::split(inputImage, &iChannels[0]);

    // show image size :
    if (showMinMax)
    {
        SD_TRACE( QString( "Image \'" + windowNameS + "\' has size : %1, %2 and nbBands = %3. Depth (bytes) = %4" )
                  .arg( inputImage0.cols )
                  .arg( inputImage0.rows )
                  .arg( inputImage0.channels())
                  .arg( depth ));
    }

    // compute min/max values & render:
    std::vector<double> min(nbBands), max(nbBands);
    std::vector<double> nmin(nbBands), nmax(nbBands);
    std::vector<bool> minMaxComputed;
    minMaxComputed.resize(nbBands, false);
    for (int i=0; i < mapping.size(); i ++)
    {
       int index = mapping.value(i+1) - 1;
       if (!minMaxComputed[index])
       {
           cv::minMaxLoc(iChannels[index], &min[index], &max[index]);
           cv::Scalar mean, std;
           cv::meanStdDev(iChannels[index], mean, std);
           nmin[index] = mean.val[0] - 3.0*std.val[0];
           nmin[index] = (nmin[index] < min[index]) ? min[index] : nmin[index];
           nmax[index] = mean.val[0] + 3.0*std.val[0];
           nmax[index] = (nmax[index] > max[index]) ? max[index] : nmax[index];
       }
       if (showMinMax)
       {
           SD_TRACE( QString( "Image " + windowNameS + ", min/max : %1, %2" ).arg( min[index] ).arg( max[index] ) );
           SD_TRACE( QString( "Image " + windowNameS + ", min/max using mean/std : %1, %2").arg( nmin[index] ).arg(  nmax[index] ) );
       }
       double a(1.0);
       double b(0.0);

       if (nmin[index] < nmax[index])
       {
           a = 255.0 / ( nmax[index] - nmin[index] );
           b = - 255.0 * nmin[index] / ( nmax[index] - nmin[index] );
       }
       iChannels[index].convertTo(oChannels[i], CV_8U, a, b);
    }

    cv::merge(oChannels, outputImage8U);
    cv::imshow(windowNameS.toStdString().c_str(), outputImage8U);

    if (waitKey)
        cv::waitKey(0);

    return outputImage8U;

}

//******************************************************************************************

cv::Mat displayContourBRect(const std::vector<std::vector<cv::Point> > &contours, const cv::Mat &background)
{
    cv::Mat t, img;
    if (background.channels() == 1)
    {
        background.copyTo(t);
        cv::Mat m[] = {t, t, t};
        cv::merge(m, 3, img);
    }
    else if (background.channels() == 3)
    {
        background.copyTo(img);
    }
    else
    {
        SD_TRACE("Background image should have 1 or 3 channels");
        return img;
    }

    for( size_t i = 0; i < contours.size(); i++ )
    {
        cv::Rect r = cv::boundingRect(contours[i]);
        // draw the circle center
        cv::rectangle( img, r.tl(), r.br(), cv::Scalar(0,255,0));
    }
    return ImageCommon::displayMat(img, true, "Circles");
}

//******************************************************************************************

cv::Mat displayCircles(const std::vector<cv::Vec3f> &circles, const cv::Mat &background)
{
    cv::Mat t, img;
    if (background.channels() == 1)
    {
        background.copyTo(t);
        cv::Mat m[] = {t, t, t};
        cv::merge(m, 3, img);
    }
    else if (background.channels() == 3)
    {
        background.copyTo(img);
    }
    else
    {
        SD_TRACE("Background image should have 1 or 3 channels");
        return img;
    }

    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // draw the circle center
        cv::circle( img, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        // draw the circle outline
        cv::circle( img, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
    }
    return ImageCommon::displayMat(img, false, "Circles");

}

//******************************************************************************************

cv::Mat displayContour(const std::vector<std::vector<cv::Point> > &contours, const cv::Mat &background, bool oneColor)
{
    cv::Mat t, img;
    if (background.channels() == 1)
    {
        background.copyTo(t);
        cv::Mat m[] = {t, t, t};
        cv::merge(m, 3, img);
    }
    else if (background.channels() == 3)
    {
        background.copyTo(img);
    }
    else
    {
        SD_TRACE("Background image should have 1 or 3 channels");
        return img;
    }

    if (oneColor)
    {
        cv::drawContours(img, contours, -1, cv::Scalar(0, 255, 0));
    }
    else
    {
        int idx = 0;
        for( ; idx < contours.size(); idx++)
        {
            cv::Scalar color( rand()&255, rand()&255, rand()&255 );
            cv::drawContours( img, contours, idx, color);
        }
    }
    return ImageCommon::displayMat(img, true, "Contours");

}

//******************************************************************************************

cv::Mat displayContour(const std::vector<std::vector<cv::Point> > &contours, const std::vector<cv::Vec4i> &hierarchy, const cv::Mat &background)
{
    cv::Mat t, img;
    if (background.channels() == 1)
    {
        background.copyTo(t);
        cv::Mat m[] = {t, t, t};
        cv::merge(m, 3, img);
    }
    else if (background.channels() == 3)
    {
        background.copyTo(img);
    }
    else
    {
        SD_TRACE("Background image should have 1 or 3 channels");
        return img;
    }

    int idx = 0;
    for( ; idx>=0; idx = hierarchy[idx][0])
    {
        cv::Scalar color( rand()&255, rand()&255, rand()&255 );
        cv::drawContours( img, contours, idx, color, cv::FILLED, cv::LINE_8, hierarchy);
    }
    return ImageCommon::displayMat(img, true, "Contours");
}

//******************************************************************************************
/*!
 * \brief isCircleLike Tests if contour is circle like.
 * \param contour
 * \return true if ratio = length(contour)/area(contour) ~ pi*(a+b)/(pi*a*b) with the tolerance
 */
bool isCircleLike(const std::vector<cv::Point> &contour, double tol)
{
    cv::Rect brect = cv::boundingRect(contour);
    double l = cv::arcLength(contour, true);
    double a = cv::contourArea(contour);
    double r = l/a;
    double r2 = 2.0*(brect.width+brect.height)/(brect.width*brect.height);
    return qAbs(r-r2) < tol;

}

//******************************************************************************************

}
