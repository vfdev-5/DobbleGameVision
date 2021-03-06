
// Std
#include <math.h>

// Qt
#include <qmath.h>

// Opencv
#include <opencv2/imgproc.hpp>

// cvplot
#ifdef HAS_CVPLOT2
#include "cvplot.h"
#endif

// Project
#include "Global.h"
#include "ImageCommon.h"
#include "ImageProcessing.h"
#include "3rdparty/AKAZEFeatures.h"




namespace ImageProcessing
{

//******************************************************************************************
/*!
 * \brief The Compare struct to compare contours by their area
 */
struct Compare : public std::binary_function<std::vector<cv::Point>, std::vector<cv::Point>, bool>
{
    enum Type {Less, Greater};
    Compare(Type type) :
        _type(type)
    {}
    bool operator() (const std::vector<cv::Point> & c1, const std::vector<cv::Point> c2) const
    {
        cv::Rect b1 = cv::boundingRect(c1);
        cv::Rect b2 = cv::boundingRect(c2);
        return _type == Greater ? b1.area() < b2.area() : b1.area() > b2.area();
    }
protected:
    Type _type;
};

//******************************************************************************************
/*!
 * \brief freqFilter
 * \param input image (1 or 2 channels)
 * \param output
 * \param freqMask should zero-one binary mask of type CV_32F
 * \param inside
 */
void freqFilter(const cv::Mat &input, cv::Mat &output, const cv::Mat &freqMask, bool inside, bool verbose)
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
    if (verbose) ImageCommon::displayMat(img2F, true, "2d fft");

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

    if (verbose) ImageCommon::displayMat(img2F2, true, "2d fft");

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
/*!
 * \brief getGaussianKernel2D generates a gaussian mask of type CV_32F
 * \param size
 * \param sigmaX, sigmaY
 * \return mask with values between 0.0 and 1.0
 */
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
 * \brief getCutGaussianKernel2D generates a cut gaussian mask of type CV_32F. A profile looks like smoothed windows
 * \param size
 * \param sigmaX
 * \param sigmaY
 * \param cut between [0.0, 1.0]. The value 0.0 corresponds to fully zero mask and 1.0 corresponds to a gaussian kernel mask without any cut
 * \return mask with values between 0.0 and 1.0
 */
cv::Mat getCutGaussianKernel2D(const cv::Size &size, double sigmaX, double sigmaY, double cut)
{
    cv::Mat out = getGaussianKernel2D(size, sigmaX, sigmaY);
    cv::threshold(out, out, cut, 1.0, CV_THRESH_TRUNC);
    out /= cut;
    return out;
}


//******************************************************************************************
/*!
 * \brief getCircleKernel2D generates a circle mask of type (CV_8U or CV_32F)
 * \param size
 * \param value is the mask value, default 255
 * \return
 */
cv::Mat getCircleKernel2D(const cv::Size &size, double value, int type)
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
    //    cv::Mat templ = ImageProcessing::getCircleKernel2D(maxRadius*2, maxRadius*2, 255);
    cv::Mat templ = ImageProcessing::getCircleKernel2D(maxRadius*2, maxRadius*2, 255);


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
void enhance(const cv::Mat &input, cv::Mat &output, double strength, bool laplacianOnly)
{
    cv::Mat t1, t2, t3;
    cv::Laplacian(input, t3, CV_32F, 3);
    //    ImageCommon::displayMat(t3, true, "Laplacian");
    if (!laplacianOnly)
    {
        cv::Scharr(input, t1, CV_32F, 1, 0);
        cv::Scharr(input, t2, CV_32F, 0, 1);
        cv::magnitude(t1, t2, t1);
        //    ImageCommon::displayMat(t1, true, "Sobel");
    }
    input.convertTo(t2, CV_32F);
    t2 -= strength*t3;
    if (!laplacianOnly) t2 -= strength*t1;
    ImageCommon::convertTo8U(t2, output);
//    t2.convertTo(output, CV_8U);
}


#ifdef HAS_3RDPARTY
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

    CV_Assert( ! img32F.empty() );


    cv::AKAZEOptions options;
    options.img_width = img32F.cols;
    options.img_height = img32F.rows;
    options.omax = 1;
    options.nsublevels = 12;

    cv::AKAZEFeatures ndf(options);
    ndf.Create_Nonlinear_Scale_Space(img32F);

    std::vector<cv::Mat> evolution;
    ndf.getNDEvolution(evolution);

    //    for (int i=0; i<evolution.size(); i++)
    //    {
    //        ImageCommon::displayMat(evolution[i], true, QString("Nonlinear filtered image : %1").arg(i));
    //    }

    evolution[evolution.size()-1].copyTo(output);

}
#endif

//******************************************************************************************
/*!
 * \brief edgeStrength Basic algorithm to get enhance the edges
 * \param input single band image of any depth
 * \param ksize
 * \param output image is between 0 and 255.0 of input depth
 * Algorithm :
 *  i1 = blur(input)
 *  i2 = dilate(i1)
 *  i3 = erode(i1)
 *  out = min(i1 - i2, i1 - i3)
 */
void edgeStrength(const cv::Mat &input, cv::Mat &output, int ksize)
{
    cv::Mat t1, t2, t3;
    if (input.depth() < CV_32F)
    {
        input.convertTo(t1, CV_32F);
    }
    else
    {
        t1 = input;
    }

    // blur
    if (ksize > 1)
        cv::blur(t1, t1, cv::Size(ksize, ksize));


    // dilate/erode
    if (ksize == 1)
        ksize = 3;
    cv::Mat k(ksize, ksize, CV_8U, cv::Scalar::all(1));
    cv::dilate(t1, t2, k);
    cv::erode(t1, t3, k);

    // min(blur - dilate, blur - erode)
    t2 = t1 - t2;
    t3 = t1 - t3;

    cv::max(t2, t3, t1);


    double minVal, maxVal;
    cv::minMaxLoc(t1, &minVal, &maxVal);
    double a, b;
    a = 255.0 /(maxVal - minVal);
    b = -255.0 * minVal / (maxVal - minVal);

//    if (input.depth() < CV_32F)
//    {
        t1.convertTo(output, input.depth(), a, b);
//    }
//    else
//    {
//    }

}


//******************************************************************************************
/*!
 * \brief detectObjects Method to detect objects on the image
 * \param image input image of type CV_8U (8 bits single channel)
 * \param objectContours output contours
 * \param minSizeRatio Minimal size ratio of detected objects, as a factor of the mean of the image dimensions
 * \param maxSizeRatio Maximal size ratio of detected objects, as a factor of the mean of the image dimensions
 * \param mask Binary mask matrix ({0,1}) indicates where to search the objects. Object should entirely be in the mask zone, otherwise it is rejected
 * \param type Object type to detect. Possible values : ANY (any type of objects), ELLIPSE_LIKE (ellipse like objects: length/area ~ (a+b)/(a*b) and length ~ pi*(a+b) )
 * \param param A parameter to define ellipse like tolerance for {ELLIPSE_LIKE, NOT_ELLIPSE_LIKE } type. See ImageCommon::isEllipseLike2
 * \param verbose Option to display intermediate processing result
 *
 *
 * Idea :
 *
 *  1) Remove small image details smaller than minimum size
 *      - Median blur of size prop to min size
 *      - Resize of factor prop to min size
 *      - Enhance edges
 *  2) Detect contours
 *      - Canny
 *      - Morpho
 *      - Find contours
 *  3) Select contours
 *      - Apply selection criteria
 *
 */

#define USE_RESIZE

void detectObjects(const cv::Mat &image, Contours *objectContours,
                   double minSizeRatio, double maxSizeRatio, const cv::Mat &mask,
                   DetectedObjectType type, double param,
                   bool verbose)
{
    if (image.type() != CV_8U) {
        SD_TRACE("detectObjects : Input image should a 8 bits single channel matrix");
        return;
    }

    if (!objectContours)
    {
        SD_TRACE("detectObjects : ObjectContours is null");
        return;
    }

    cv::Size size = image.size();
    cv::Mat procImage;
    image.copyTo(procImage);

    int imageDim = (image.cols + image.rows)/2;
    if (verbose) SD_TRACE1("Detected object min size : %1", imageDim*minSizeRatio);
    if (verbose) SD_TRACE1("Detected object max size : %1", imageDim*maxSizeRatio);


    // ***** Remove small image details smaller than minimum size *****

    // Big initial median blur of min object size
    int objectMinSize = imageDim*minSizeRatio;
    bool useResize=false;

#if 0
    if (objectMinSize > 50)
    {
        int objectblurSize = objectMinSize/3 - 1;
        if (objectblurSize % 2 == 0) objectblurSize++;
        cv::medianBlur(procImage, procImage, objectblurSize);
        if (verbose) ImageCommon::displayMat(procImage, true, QString("Median blur, ksize=%1").arg(objectblurSize));
        useResize=true;
    }
    else
    {
        // Median blur
        int medianBlurSize = 5;
        cv::medianBlur(procImage, procImage, medianBlurSize);
        if (verbose) ImageCommon::displayMat(procImage, true, QString("Median blur, ksize=%1").arg(medianBlurSize));
    }
#endif


#if 1

    // Median blur
    int medianBlurSize = 5;
    cv::medianBlur(procImage, procImage, medianBlurSize);
    if (verbose) ImageCommon::displayMat(procImage, true, QString("Median blur, ksize=%1").arg(medianBlurSize));

    if (objectMinSize > 0)
    {
        // Hypothesis on frequency/object-size dependency
        // unit pulse (pulse duration = tau on total time T)-> cardinal sinus : sinc(pi*tau*f)
        // low-pass filter with fcut = 1.0/tau
        // fcut_index = fcut / df, df = 1.0/T
        //
        // T = image.size, tau = minObjectSize
        // fcut_index = image.size * (1/minObjectSize)
        int sx = (10.0/objectMinSize)*procImage.cols;
        int sy = (10.0/objectMinSize)*procImage.rows;
        SD_TRACE2("FFT filter size : %1, %2", sx, sy);
        cv::Mat freqMask = ImageProcessing::getCutGaussianKernel2D(sx, sy, 0.0, 0.0, 0.25);
        ImageProcessing::freqFilter(procImage, procImage, freqMask, true);
        if (verbose) ImageCommon::displayMat(procImage, true, QString("fft filtered"));
    }


#endif






#ifdef USE_RESIZE
    cv::Size initSize=procImage.size();
    if (useResize)
    {
        double f=objectMinSize*0.02;
        cv::resize(procImage, procImage, cv::Size(), 1.0/f, 1.0/f, cv::INTER_NEAREST);
    }
#endif

#if 0
    // Enhance contours
    cv::Mat t;
    procImage.convertTo(procImage, CV_32F);
    int esSize = 3;
    ImageProcessing::edgeStrength(procImage, t, esSize);
    if (verbose) ImageCommon::displayMat(t, true, QString("Edge strength"));

    //    double minVal, maxVal;
    //    cv::minMaxLoc(t, &minVal, &maxVal);
    //    double thr = (maxVal - minVal)*0.75 + minVal;
    //    cv::threshold(t, t, thr, 1.0, cv::THRESH_BINARY);
    //    if (verbose) ImageCommon::displayMat(t, true, QString("Edge strength thresholded : %1").arg(thr));

    procImage = procImage - 0.25*t;
    ImageCommon::convertTo8U(procImage, procImage);
    if (verbose) ImageCommon::displayMat(procImage, true, "Enchanced");
#endif


#if 0
    cv::Mat m;
    int d=15;
    double sigmaSpace=15.0;
    double sigmaColor=50.0;
    cv::bilateralFilter(procImage, m, d, sigmaColor, sigmaSpace);
    procImage = m;
    if (verbose) ImageCommon::displayMat(procImage, true, QString("Bilateral : %1").arg(sigmaSpace));
#endif



#if 1
    {

        procImage.convertTo(procImage, CV_32F);
        procImage = ImageCommon::normalize(procImage, 0.001);


        if (verbose) ImageCommon::displayMat(procImage, true, "Initial");

#ifdef HAS_CVPLOT2
//        if (verbose)
//        {
//            cv::Mat m;
//            procImage.copyTo(m);
//            for (int d=80; d<procImage.cols && d<procImage.rows; d+=110)
//            {
//                cv::Mat cut1 = procImage.row(d);
//                cv::Mat cut2 = procImage.col(d);
//                CvPlot::plot("Initial cut 1", cut1);
//                CvPlot::plot("Initial cut 2", cut2);
//                cv::line(m, cv::Point(d, 0), cv::Point(d, procImage.rows), cv::Scalar(1.0));
//                cv::line(m, cv::Point(0, d), cv::Point(procImage.cols, d), cv::Scalar(1.0));
//            }
//            if (verbose) ImageCommon::displayMat(m, true, "Initial out");
//        }
#endif

        double v = 0.20;
        cv::threshold(procImage, procImage, 1.0 - v, 1.0, CV_THRESH_TRUNC);
        cv::threshold(procImage, procImage, v, 0.0, CV_THRESH_TOZERO);

//        cv::Mat t1, t2;
//        cv::threshold(procImage, t1, 0.75, 1.0, CV_THRESH_TRUNC);
//        cv::pow(procImage, 0.33, t1);
//        if (verbose) ImageCommon::displayMat(t1, true, "Power High values");
//        cv::Laplacian(t1, t1, t1.depth(), 5);
//        t1 = ImageCommon::normalize(t1, 0.001);
//        if (verbose) ImageCommon::displayMat(t1, true, "Power High values Edges");

//        cv::pow(procImage, 3.0, t2);
//        cv::threshold(procImage, t2, 0.25, 0.0, CV_THRESH_TOZERO);
//        if (verbose) ImageCommon::displayMat(t2, true, "Power Low values");
//        cv::Laplacian(t2, t2, t2.depth(), 5);
//        t2 = ImageCommon::normalize(t2, 0.001);
//        if (verbose) ImageCommon::displayMat(t2, true, "Power Low values Edges");


//        t1 = t1 + t2;
//        procImage= 0.5*t1 + 0.5*t2;

//        t1 = procImage;
//        t1 = ImageCommon::normalize(t1);
//        if (verbose) ImageCommon::displayMat(t1, true, "t1");
//        t2 = 1.0/procImage;
//        t2 = ImageCommon::normalize(t2);
//        if (verbose) ImageCommon::displayMat(t2, true, "t2");
//        if (verbose) ImageCommon::displayMat(t1 + t2, true, "t1 + t2");
//        cv::pow(t1 + t2, 0.5, procImage);
//        cv::Laplacian(t1, t2, t1.depth(), 5);
//        if (verbose) ImageCommon::displayMat(t2, true, "Edges");


        procImage = ImageCommon::normalize(procImage, 0.001);
        ImageCommon::convertTo8U(procImage, procImage);

#ifdef HAS_CVPLOT2
//        if (verbose)
//        {
//            cv::Mat m;
//            procImage.copyTo(m);
//            for (int d=80; d<procImage.cols && d<procImage.rows; d+=110)
//            {
//                cv::Mat cut1 = procImage.row(d);
//                cv::Mat cut2 = procImage.col(d);
//                CvPlot::plot("Enchanced cut 1", cut1);
//                CvPlot::plot("Enchanced cut 2", cut2);
//                cv::line(m, cv::Point(d, 0), cv::Point(d, procImage.rows), cv::Scalar(255.0));
//                cv::line(m, cv::Point(0, d), cv::Point(procImage.cols, d), cv::Scalar(255.0));
//            }
//            if (verbose) ImageCommon::displayMat(m, true, "Initial out");
//        }
#endif

        if (verbose) ImageCommon::displayMat(procImage, true, "Enchanced");

    }
#endif


    // ***** Detect contours *****

    // Canny
    //  1 Apply Gaussian filter to smooth the image in order to remove the noise
    //  2 Find the intensity gradients of the image
    //  3 Apply non-maximum suppression to get rid of spurious response to edge detection
    //  4 Apply double threshold to determine potential edges
    // If the edge pixel’s gradient value is higher than the high threshold value, they are marked as STRONG edge pixels.
    // If the edge pixel’s gradient value is smaller than the high threshold value and larger than the low threshold value,
    // they are marked as WEAK edge pixels. If the pixel value is smaller than the low threshold value, they will be suppressed.
    //  5 Track edge by hysteresis: Finalize the detection of edges by suppressing all the other edges that are weak and not connected to strong edges.

    int t1 = 70; // 50; // 20
    int t2 = 200; // 150; // 150
    cv::Canny(procImage, procImage, t1, t2);

    if (verbose) ImageCommon::displayMat(procImage, true, QString("Canny : %1, %2").arg(t1).arg(t2));



    //    cv::Mat in3, in3c[] = {procImage, procImage, procImage};
    //    cv::merge(in3c, 3, in3);
    //    procImage.convertTo(procImage, CV_32S, 1.0/255.0);

    //    cv::watershed(in3, procImage);
    //    if (verbose) ImageCommon::displayMat(procImage, true, QString("Watershed"));



#ifdef USE_RESIZE
    if (useResize)
    {
        cv::resize(procImage, procImage, initSize, 0, 0, cv::INTER_NEAREST);
        if (verbose) ImageCommon::displayMat(procImage, true, "Normal size");
    }
#endif

    // Morpho
    cv::Mat k1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));

#if 1
    // This chain is good to englobe external contours
    cv::morphologyEx(procImage, procImage, cv::MORPH_DILATE, k1, cv::Point(1, 1), 1);
    //    if (verbose) ImageCommon::displayMat(procImage, true, "Morpho dilate");
    //    cv::morphologyEx(procImage, procImage, cv::MORPH_CLOSE, k1, cv::Point(1, 1), canSmooth ? 2 : 1);
    cv::morphologyEx(procImage, procImage, cv::MORPH_CLOSE, k1, cv::Point(1, 1), 1);
    //    if (verbose) ImageCommon::displayMat(procImage, true, "Morpho dilate + close");
    cv::morphologyEx(procImage, procImage, cv::MORPH_ERODE, k1, cv::Point(1, 1), 1);
#else
    // CLOSE seems to be useless. Main contours are far one from another
    cv::morphologyEx(procImage, procImage, cv::MORPH_CLOSE, k1, cv::Point(1, 1), 1);
    //    if (verbose) ImageCommon::displayMat(procImage, true, "Morpho close");
    cv::morphologyEx(procImage, procImage, cv::MORPH_DILATE, k1, cv::Point(1, 1), 1);
#endif

    if (verbose) ImageCommon::displayMat(procImage, true, "Morpho");


    // RETURN TO REMOVE
//    return;
    // RETURN TO REMOVE



    // Apply mask if required
    if (!mask.empty())
    {
        procImage = procImage.mul(mask);
    }


    // Find contours
    std::vector< std::vector<cv::Point> > contours;
    std::vector< cv::Vec4i > hierarchy;
    cv::findContours(procImage, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
    objectContours->clear();
    objectContours->resize(contours.size());


    // ***** Select contours *****

    // Define constraints:
    int totalArea = size.width * size.height;
    int maxArea = maxSizeRatio*maxSizeRatio*totalArea;
    int minArea = minSizeRatio*minSizeRatio*totalArea;

    //    int roiRadius = 0.45 * size.width;
    //    if (verbose) SD_TRACE(QString("Roi radius : %1").arg(roiRadius));
    //    int maxLength = 0.95*size.width * M_PI;
    if (verbose) SD_TRACE(QString("Contours count : %1").arg(contours.size()));

    int count=0;
    for (size_t i=0;i<contours.size();i++)
    {
        std::vector<cv::Point> contour = contours[i];
        cv::Vec4i contourHierarchy = hierarchy[i];

        // neglect contours with a parents
        if (contourHierarchy[3] >= 0)
            continue;

        cv::Rect brect = cv::boundingRect(contour);
        int a = brect.area();
        //        int dx = brect.tl().x + brect.width/2 - size.width/2;
        //        int dy = brect.tl().y + brect.height/2 - size.height/2;
        //        int maxdim = qMax(brect.width, brect.height);

        // Select contour such that :
        // a) bounding rect of the contour larger min area and smaller than max area

        // REMOVE b) distance between center of the contour and the card center is smaller than card radius

        // REMOVE c) max dimension of contour is smaller than card radius

        // REMOVE d) contour brect should not touch (+/- 1 pixel) image boundaries

        if (a > minArea && a < maxArea
                //                dx*dx + dy*dy < roiRadius*roiRadius &&
                //                maxdim < roiRadius &&
                //                brect.x > 1 && brect.y > 1 &&
                //                brect.br().x < size.width-2 &&  brect.br().y < size.height-2)
                )
        {

            if (verbose)
            {
                std::vector< std::vector<cv::Point> > tstContours;
                tstContours.push_back(contour);
                ImageCommon::displayContours(tstContours, image, false, true);
            }

            if (type == ANY)
            {
                (*objectContours)[count].swap(contour);
                count++;
            }
            else
            {
                bool isEllipseLike = ImageCommon::isEllipseLike2(contour, param);
                if ((isEllipseLike && type == ELLIPSE_LIKE) ||
                        (type == NOT_ELLIPSE_LIKE && !isEllipseLike))
                {
                    (*objectContours)[count].swap(contour);
                    count++;
                }
            }
        }
    }
    objectContours->resize(count);

    // order by size (descending)
    std::sort(objectContours->begin(), objectContours->end(), Compare(Compare::Less));

    if (verbose) SD_TRACE(QString("Selected contours count : %1").arg(count));
    if (verbose) ImageCommon::displayContours(objectContours->toStdVector(), image, false, true);


}

//******************************************************************************************

cv::Mat getObjectMask(const cv::Size &size, const std::vector<cv::Point> & contour)
{
    cv::Mat objectMask = cv::Mat(size.height, size.width, CV_8U, cv::Scalar::all(0));
    cv::Scalar color( 1 );
    std::vector<std::vector<cv::Point> > contours;
    contours.push_back(contour);
    cv::drawContours( objectMask, contours, 0, color, CV_FILLED);
    return objectMask;
}

//******************************************************************************************

void detectObjects2(const cv::Mat &image, Contours *objectContours, double minSizeRatio, double maxSizeRatio, const cv::Mat &mask, DetectedObjectType type, double param, bool verbose)
{

    if (image.type() != CV_8U) {
        SD_TRACE("detectObjects : Input image should a 8 bits single channel matrix");
        return;
    }

    if (!objectContours)
    {
        SD_TRACE("detectObjects : ObjectContours is null");
        return;
    }

    cv::Size size = image.size();
    cv::Mat procImage;
    image.copyTo(procImage);

    int imageDim = (image.cols + image.rows)/2;
    if (verbose) SD_TRACE1("Detected object min size : %1", imageDim*minSizeRatio);
    if (verbose) SD_TRACE1("Detected object max size : %1", imageDim*maxSizeRatio);




    int sx = 0.15*procImage.cols;
    int sy = 0.15*procImage.rows;
    cv::Mat freqMask = ImageProcessing::getCutGaussianKernel2D(sx, sy, 0.0, 0.0, 0.3);
    ImageCommon::displayMat(freqMask, true, "freqMask");
    ImageProcessing::freqFilter(procImage, procImage, freqMask, true, true);
    ImageCommon::displayMat(procImage, true, "fft filteres");



}


//******************************************************************************************

}
