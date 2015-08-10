
// Std
#include <math.h>

// Qt
#include <qmath.h>

// Opencv
#include <opencv2/imgproc.hpp>

// Project
#include "Global.h"
#include "ImageCommon.h"
#include "ImageFiltering.h"

namespace ImageFiltering
{

//******************************************************************************************

void freqFilter(const cv::Mat &input, cv::Mat &output, const cv::Mat &freqMask, bool inside)
{
    if (freqMask.type() != CV_32F)
    {
        SD_TRACE("freqFilter : freqMask is not of type CV_32F");
        return;
    }
    if (input.channels() > 2 )
    {
        SD_TRACE("freqFilter : input  should have 1 or 2 channels");
        return;
    }

    cv::Mat img2F;
    input.convertTo(img2F, CV_32F);
    cv::dft(img2F, img2F, cv::DFT_COMPLEX_OUTPUT | cv::DFT_SCALE);

    img2F = fftShift(img2F);
    //    ImageCommon::displayMat(img2F, true, "2d fft");

    int w = img2F.cols;
    int h = img2F.rows;
    cv::Rect r(w/2 - freqMask.cols/2, h/2 - freqMask.rows/2, freqMask.cols, freqMask.rows);

    cv::Mat img2F2;
    if (inside)
    {
        img2F2 = cv::Mat(img2F.rows, img2F.cols, CV_32FC2, cv::Scalar::all(0));
        cv::Mat mask;
        cv::Mat m[] = {freqMask, freqMask};
        cv::merge(m, 2, mask);
        cv::Mat t = img2F(r).mul(mask);
        t.copyTo(img2F2(r));
    }
    else
    {
        img2F.copyTo(img2F2);
        cv::Mat mask, umask;
        cv::absdiff(1.0, freqMask, mask);
        cv::Mat m[] = {mask, mask};
        cv::merge(m, 2, umask);
        cv::Mat t = img2F(r).mul(umask);
        t.copyTo(img2F2(r));
    }

    //    ImageCommon::displayMat(img2F2, true, "2d fft");

    img2F2 = fftShift(img2F2);
    cv::idft(img2F2,img2F2);

    std::vector<cv::Mat> ic(2);
    cv::split(img2F2, &ic[0]);
    cv::Mat tOut;
    cv::magnitude(ic[0], ic[1], tOut);

    if (tOut.depth() != input.depth())
    {
        tOut.convertTo(output, input.depth());
    }
    else
    {
        output = tOut;
    }
}

//******************************************************************************************

cv::Mat fftShift(const cv::Mat &input)
{
    int w = input.cols;
    int h = input.rows;

    cv::Rect r1(0,0,w/2,h/2), r11(w-w/2,h-h/2,w/2,h/2);
    cv::Rect r2(w/2,0,w-w/2,h/2), r22(0,h-h/2,w-w/2,h/2);
    cv::Rect r3(0,h/2,w/2,h-h/2), r33(w-w/2,0,w/2,h-h/2);
    cv::Rect r4(w/2,h/2,w-w/2,h-h/2), r44(0,0,w-w/2,h-h/2);

    cv::Mat out = cv::Mat::zeros(h,w,input.type());
    input(r1).copyTo(out(r11));
    input(r2).copyTo(out(r22));
    input(r3).copyTo(out(r33));
    input(r4).copyTo(out(r44));

    return out;

}

//******************************************************************************************

cv::Mat getGaussianKernel2D(const cv::Size &size, double sigmaX, double sigmaY)
{
    cv::Mat out(size.height, size.width, CV_32F, cv::Scalar::all(0.0));
    cv::Mat k1 = cv::getGaussianKernel(size.height, sigmaY, CV_32F);
    cv::Mat k2 = cv::getGaussianKernel(size.width, sigmaX, CV_32F);

    double maxValue=0.0;
    for (int i=0;i<size.height;i++)
    {
        double v = k1.at<float>(i);
        for (int j=0;j<size.width;j++)
        {
            double v2 = v*k2.at<float>(j);
            out.at<float>(i,j) = v2;
            if (v2 > maxValue) maxValue = v2;
        }
    }

    out /= maxValue;

    return out;

}

//******************************************************************************************
/*!
 * \brief getCircleKernel2D generates a circle mask of type (CV_8U or CV_32F)
 * \param size
 * \param value is the mask value, default 255
 * \return
 */
cv::Mat getCircleKernel2D(const cv::Size &size, int value, int type)
{
    int w2 = size.width;
    int h2 = size.height;
    int wf = qFloor(size.width*0.5);
    int hf = qFloor(size.height*0.5);

    cv::Mat out(size.height, size.width, CV_32F, cv::Scalar::all(value));

    double c = 0;
    for (int i=0;i<hf;i++)
    {
        c = 1.0 - (i-hf)*(i-hf)*1.0/(hf*hf);
        for (int j=0;j<wf;j++)
        {
            if ((j-wf)*(j-wf)*1.0/(wf*wf) > c)
            {
                out.at<float>(i,j) = 0;
                out.at<float>(h2-1-i,j) = 0;
                out.at<float>(h2-1-i,w2-1-j) = 0;
                out.at<float>(i,w2-1-j) = 0;
            }
        }
    }
    if (type == CV_8U)
    {
        cv::Mat t;
        out.convertTo(t, CV_8U);
        out = t;
    }
    return out;
}

//******************************************************************************************
/*!
 * \brief detectCircles detects circles of minimum radius in the image
 * \param image is a single channel CV_8U or CV_32F
 * \param minRadius
 * \return vector of 3d points. A point represents circle center and radius (x,y,r)
 *
 * Algorithm
 * 0) Blur image
 * 1) Match template to a circle template image
 * 2) Threshold image with value = threshold <=> we alse detects larger circles
 * 3) Find contours -> generate templates on found detections
 *
 */
void detectCircles(const cv::Mat &image, std::vector<cv::Vec3f> & output, int minRadius, int maxRadius, double threshold)
{
    int depth = image.depth();
    if (image.channels() > 1 || (depth != CV_8U && depth != CV_32F))
        return;

    cv::Mat procImage;
    if (depth != CV_32F)
        image.convertTo(procImage, CV_32F);
    else
        procImage = image;

    // Blur
    cv::blur(procImage, procImage, cv::Size(5,5));


    // Match a circle template
    cv::Mat t1, t2;
    //    cv::Mat templ = ImageFiltering::getCircleKernel2D(maxRadius*2, maxRadius*2, 255);
    cv::Mat templ = ImageFiltering::getCircleKernel2D(maxRadius*2, maxRadius*2, 255);


    ImageCommon::displayMat(templ, true, "Template");
    cv::matchTemplate(procImage,templ,t1,cv::TM_CCOEFF_NORMED);
    ImageCommon::displayMat(t1, true, "Corr");

    // Threshold
    cv::threshold(t1, t2, threshold, 255.0, CV_THRESH_BINARY);
    ImageCommon::displayMat(t2, true, "Corr");

    // Find contours
    t2.convertTo(t1, CV_8U);
    std::vector< std::vector<cv::Point> > contours;
    cv::Point offset(templ.cols/2, templ.rows/2);
    cv::findContours(t1, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, offset);

    // Create output
    for (uint i=0; i<contours.size();i++)
    {
        std::vector<cv::Point> contour = contours[i];
        cv::Rect brect = cv::boundingRect(contour);
        float radius = 0.25*(templ.cols + templ.rows);
        output.push_back(cv::Vec3f(brect.tl().x + brect.width * 0.5,
                                   brect.tl().y + brect.height * 0.5,
                                   radius ));
    }
}

//******************************************************************************************

void simplify(const cv::Mat &src, cv::Mat &dst, double f)
{
    cv::resize(src, dst, cv::Size(), 1.0/f, 1.0/f,cv::INTER_NEAREST);
    cv::medianBlur(dst, dst, 3);
    cv::resize(dst, dst, cv::Size(), f, f, cv::INTER_LINEAR);
}

//******************************************************************************************
/*!
 * \brief enhance image contrast using cv::Scharr derivative
 * \param input
 * \param output
 */
void enhance(const cv::Mat &input, cv::Mat &output, double strength)
{
    cv::Mat t1, t2, t3;
    cv::Sobel(input, t1, CV_32F, 1, 0);
    cv::Sobel(input, t2, CV_32F, 0, 1);
    cv::Laplacian(input, t3, CV_32F, 3);
    //    ImageCommon::displayMat(t3, true, "Laplacian");
    cv::magnitude(t1, t2, t1);
    //    ImageCommon::displayMat(t1, true, "Sobel");
    input.convertTo(t2, CV_32F);
    t2 -= strength*(t1 + t3);
    t2.convertTo(output, CV_8U);
}

//******************************************************************************************
/*!
 * \brief nonlinearDiffusionFiltering
 * \param input
 * \param output
 */
void nonlinearDiffusionFiltering(const cv::Mat &input, cv::Mat &output)
{

    cv::Mat img32F;
    if ( input.depth() == CV_32F )
        img32F = input;
    else if ( input.depth() == CV_8U )
        input.convertTo(img32F, CV_32F, 1.0 / 255.0, 0);
    else if ( input.depth() == CV_16U )
        input.convertTo(img32F, CV_32F, 1.0 / 65535.0, 0);



}

//******************************************************************************************

}
