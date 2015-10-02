
// Std
#include <vector>

// Qt
#include <QMap>
#include <qmath.h>

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
    int inputDepth = inputImage0.depth();
    if (inputDepth != CV_32F && inputDepth != CV_64F)
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
        if (inputDepth != CV_8U)
        {
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
        }
        else
        {
            nmin[index] = min[index] = 0;
            nmax[index] = max[index] = 255;
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
    if (!background.empty())
    {
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
    }
    else
    {
        //        cv::Rect br = cv::boundingRect();
        img = cv::Mat(500, 500, CV_8UC3, cv::Scalar::all(0));
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
    if (!background.empty())
    {
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
    }
    else
    {
        //        cv::Rect br = cv::boundingRect();
        img = cv::Mat(500, 500, CV_8UC3, cv::Scalar::all(0));
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

cv::Mat displayContours(const std::vector<std::vector<cv::Point> > &contours, const cv::Mat &background, bool oneColor, bool fillContours, const QString & winname)
{
    cv::Mat t, img;
    if (!background.empty())
    {
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
    }
    else
    {
//        cv::Rect br = cv::boundingRect();
        img = cv::Mat(500, 500, CV_8UC3, cv::Scalar::all(0));
    }

    if (oneColor)
    {
        cv::drawContours(img, contours, -1, cv::Scalar(0, 255, 0), fillContours ? CV_FILLED : 1);
    }
    else
    {
        int idx = 0;
        for( ; idx < contours.size(); idx++)
        {
            cv::Scalar color( 2*idx + rand()&255, rand()&255, rand()&255 );
            cv::drawContours( img, contours, idx, color, fillContours ? CV_FILLED : 1);
        }
    }
    return ImageCommon::displayMat(img, false, winname);

}

//******************************************************************************************

cv::Mat displayContours(const std::vector<std::vector<cv::Point> > &contours, const std::vector<cv::Vec4i> &hierarchy, const cv::Mat &background)
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
 * \brief isEllipseLike Tests if contour is ellipse like
 * \param contour
 * \param tol
 * \return true if ratio = length(contour)/area(contour) ~ pi*(a+b)/(pi*a*b) with the tolerance, where a=brect(contour).width and b=brect(contour).height
 *  and length(contour) ~ pi*(a+b) or area(contour) ~ pi*a*b
 */
bool isEllipseLike(const std::vector<cv::Point> &contour, double tol)
{
    cv::Rect brect = cv::boundingRect(contour);
    double l = cv::arcLength(contour, true);
    double a = cv::contourArea(contour);
    double r = l/a;
    double r2 = 2.0*(brect.width+brect.height)/(brect.width*brect.height);
    double l2 = M_PI*(brect.width+brect.height)*0.5;
    double a2 = M_PI*(brect.width*brect.height)*0.25;
    return (qAbs(r-r2) < tol) && ((qAbs(l-l2) < 5.0*tol*l2) || (qAbs(a-a2) < 5.0*tol*a2));
}

//******************************************************************************

void intersectWithRect(const std::vector<cv::Point> &rect, const std::vector<cv::Point> &contour2, std::vector<cv::Point> &output)
{
    for (int i=0;i<contour2.size();i++)
    {
        if (cv::pointPolygonTest(rect, contour2[i], false)>=0)
        {
            output.push_back(contour2[i]);
        }
    }
}

//******************************************************************************

/*!
 * \brief intersectWithEllipse
 * \param center
 * \param a
 * \param b
 * \param angle in degrees
 * \param contour2
 * \param output
 *
 * Point (x,y) is inside the ellipse (xc,yc,a,b,angle) if
 *
 * deltaX^2 * A^2 + deltaY^2 * B^2 + 2 * deltaX * deltaY * C < 1
 * where
 *  deltaX = x-xc
 *  deltaY = y-yc
 *  A^2 = cos^2(angle)/a^2 + sin^2(angle)/b^2
 *  B^2 = sin^2(angle)/a^2 + cos^2(angle)/b^2
 *  C = cos(angle) * sin(angle) * (1/a^2 - 1/b^2)
 *
 */
void intersectWithEllipse(const cv::Point & center, double a, double b, double angle, const std::vector<cv::Point> &contour2, std::vector<cv::Point> &output)
{
    double deltaX = 0;
    double deltaY = 0;
    double c = qCos(M_PI * angle / 180.0);
    double s = qSin(M_PI * angle / 180.0);
    double A2 = c*c/(a*a) + s*s/(b*b);
    double B2 = s*s/(a*a) + c*c/(b*b);
    double C = c*s * (1.0/(a*a) - 1.0/(b*b));


    for (int i=0;i<contour2.size();i++)
    {
        deltaX = contour2[i].x - center.x;
        deltaY = contour2[i].y - center.y;
        if (deltaX*deltaX*A2 + deltaY*deltaY*B2 + 2.0*deltaX*deltaY*C <= 1.0)
        {
            output.push_back(contour2[i]);
        }
    }
}

//******************************************************************************
/*!
 * \brief isEllipseLike2
 * \param contour
 * \param tol is measured between 0 and inf. The value 0 means the complete match between the contour and fitted ellipse.
 * Large values of tol accept the defects of the contours
 * \return true if contour 'looks' like an ellipse
 *
 * Idea :
 * 1) Compute moments and obtain center and eigenvalues of principal axes
 * 2) Intersect the  contour with a theoretical ellipse -> contour without defects
 * 3) Compute length and area of the intersected contour
 * 4) Compare the parameters (length, area, length/area) with the theoretical ellipse
 *
 */
bool isEllipseLike2(const std::vector<cv::Point> &contour, double tol)
{
    cv::Moments mts = cv::moments(contour);

//    SD_TRACE4("Moments : m00=%1, mu02'=%2, mu20'=%3, mu11'=%4", mts.m00, mts.mu02/mts.m00, mts.mu20/mts.m00, mts.mu11/mts.m00);
//    SD_TRACE4("Moments : mu12=%1, mu21=%2, mu30=%3, mu03=%4", mts.mu12, mts.mu21, mts.mu03, mts.mu30);

    double mu20 = mts.mu20/mts.m00, mu02 = mts.mu02/mts.m00, mu11 = mts.mu11/mts.m00;
    double theta = 0.5*qAtan2(2.0*mu11,mu20-mu02)*180.0/M_PI;

//    SD_TRACE1("Theta : %1", theta);
    double v1, v2;
    double x, y;

    x = mts.m10/mts.m00;
    y = mts.m01/mts.m00;

    v1 = 0.5*(mu20 + mu02) + 0.5*qSqrt(4*mu11*mu11 + (mu20-mu02)*(mu20-mu02));
    v1 = 2.03*qSqrt(v1);
    v2 = 0.5*(mu20 + mu02) - 0.5*qSqrt(4*mu11*mu11 + (mu20-mu02)*(mu20-mu02));
    v2 = 2.03*qSqrt(v2);

//    SD_TRACE2("x,y: %1, %2", x, y);
//    SD_TRACE2("Eigen values : %1, %2", v1, v2);

    std::vector<cv::Point> intersection;

    intersectWithEllipse(cv::Point(x,y), v1+0.5, v2+0.5, theta, contour, intersection);

//    // DEBUG
//    std::vector< std::vector<cv::Point> > testContours;
//    testContours.push_back(contour);
//    if (intersection.size() > 0)
//        testContours.push_back(intersection);
//    ImageCommon::displayContours(testContours, cv::Mat(), false);
//    // DEBUG


    double l = cv::arcLength(intersection, true);
    double a = cv::contourArea(intersection);
    double r = l/a;

    double r2 = (v1+v2)/(v1*v2);
    double l2 = M_PI*(v1+v2);
    double a2 = M_PI*(v1*v2);

    double ntol = (1.0 + tol*5.0)*0.01;
    double f=2.0;

//    SD_TRACE2("Params A) : qAbs(r-r2)=%1, ntol=%2, ", qAbs(r-r2), ntol);
//    SD_TRACE4("Params B) : qAbs(l-l2)=%1, f*ntol*l2=%2, qAbs(a-a2)=%3, f*ntol*a2=%4", qAbs(l-l2), f*ntol*l2, qAbs(a-a2), f*ntol*a2);

    return (qAbs(r-r2) < ntol) && ((qAbs(l-l2) < f*ntol*l2) && (qAbs(a-a2) < f*ntol*a2));

}

//******************************************************************************
///*!
// * \brief isCircleLike Tests if contour is circle like or ratio length/area is about 2/R and
// * \param contour
// * \param tol
// * \return true if ratio = length(contour)/area(contour) ~ 2/R with the tolerance, where R=0.5*mean(brect(contour).width + brect(contour).height)
// * and area(contour) ~ pi*R^2
// */
//bool isCircleLike(const std::vector<cv::Point> &contour, double tol)
//{
//    cv::Rect brect = cv::boundingRect(contour);
//    double l = cv::arcLength(contour, true);
//    double a = cv::contourArea(contour);
//    double r = l/a;
//    double R = 0.25*(brect.width+brect.height);
//    double r2 = 2.0/R;
//    double a2 = M_PI*R*R;
//    return (qAbs(r-r2) < tol) && (qAbs(a-a2) < 5.0*tol*a2);
//}

//******************************************************************************************

template<typename T>
void printPixel(const cv::Mat & singleBand, int x, int y)
{

    std::cout << singleBand.at<T>(y,x);
}

// Print matrix data:
void printMat(const cv::Mat & inputImage0, const QString &windowName, int limit)
{

    cv::Mat inputImage;
    if (inputImage0.depth() != CV_32F)
    {
        inputImage0.convertTo(inputImage, CV_32F);
    }
    else
        inputImage = inputImage0;

    QString t = windowName.isEmpty() ? "inputImage" : windowName;
    SD_TRACE(QString("------ Print matrix : ") + t);
    int w = inputImage.cols;
    int h = inputImage.rows;
    SD_TRACE("Size : " + QString::number(w) + ", " + QString::number(h));

    w = w > limit ? limit : w;
    h = h > limit ? limit : h;
    int nbBands = inputImage.channels();
    std::vector<cv::Mat> iChannels(nbBands);
    cv::split(inputImage, &iChannels[0]);

    for (int i=0; i<h; i++)
    {
        for (int j=0; j<w; j++)
        {
            std::cout << "(";
            for (int k=0; k<nbBands; k++)
            {
                printPixel<float>(iChannels[k], j, i);
                std::cout << " ";
            }
            std::cout << ")";
        }
        std::cout << std::endl;
    }
    std::cout << "------"<< std::endl;


}

//******************************************************************************

void convertTo8U(const cv::Mat &input, cv::Mat &output)
{

    int nbOfChannels = input.channels();

    std::vector<cv::Mat> iChannels(nbOfChannels);
    std::vector<cv::Mat> oChannels(nbOfChannels);
    cv::split(input, &iChannels[0]);

    for (int i=0; i<nbOfChannels; i++)
    {
        double minVal, maxVal;
        cv::minMaxLoc(iChannels[i], &minVal, &maxVal);
        double a, b;
        a = 255.0 /(maxVal - minVal);
        b = -255.0 * minVal / (maxVal - minVal);
        iChannels[i].convertTo(oChannels[i], CV_8U, a, b);
    }
    cv::merge(oChannels, output);



}

//******************************************************************************
/*!
 * \brief normalize method to normalize single channel image (with depth >= 32F) between minVal and maxVal
 * \param inputF
 * \return
 */
cv::Mat normalize(const cv::Mat &inputF, double minVal, double maxVal)
{
    if (inputF.channels() > 1 || inputF.depth() < CV_32F)
        return cv::Mat();

    double oldMinVal, oldMaxVal;
    cv::minMaxLoc(inputF, &oldMinVal, &oldMaxVal);

    return (maxVal-minVal) * (inputF - oldMinVal)/(oldMaxVal - oldMinVal) + minVal;
}

//******************************************************************************
\
}
