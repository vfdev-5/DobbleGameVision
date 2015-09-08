// Opencv
#include <opencv2/imgproc.hpp>

// Project
#include "BasicPairsDetector.h"
#include "Core/Global.h"
#include "Core/ImageCommon.h"

namespace DGV
{

//******************************************************************************************

BasicPairsDetector::BasicPairsDetector(float goodDistance, int goodMatchesMinLimit, bool verbose) :
    _goodDistance(goodDistance),
    _goodMatchesMinLimit(goodMatchesMinLimit),
    PairsDetector(verbose)
{
#if 1
        _extractor = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_KAZE, 1, 3, 0.0001, 4, 4);
        _matcher = cv::DescriptorMatcher::create("FlannBased");
#else
        _extractor = cv::KAZE::create();
        _matcher = cv::DescriptorMatcher::create("FlannBased");
        //float goodDistance = 0.25;
#endif

}

//******************************************************************************************
/*!
 * \brief BasicPairsDetector::setupRefObject setups the reference object
 * \param image a matrix with the object inside. If no mask provided, whole image will be considered as the reference object
 * \param mask (optional) object mask
 */
bool BasicPairsDetector::setupRefObject(const cv::Mat &image, const cv::Mat & mask)
{
    cv::Mat descriptors = computeDescriptors(image, mask, _refKeyPoints);
    if (descriptors.empty()) {
        SD_TRACE("BasicPairsDetector::setupRefObject : descriptors matrix is empty");
        return false;
    }

    _matcher->clear();
    _matcher->add(descriptors);
    _matcher->train();

    return true;
}

//******************************************************************************************

bool BasicPairsDetector::matchWithRefObject(const cv::Mat &image, const cv::Mat &mask)
{
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors = computeDescriptors(image, mask, keypoints);
    if (descriptors.empty()) {
        SD_TRACE("BasicPairsDetector::matchWithRefObject : descriptors matrix is empty");
        return false;
    }


    std::vector<cv::DMatch> matchedKeypoints;
    _matcher->match(descriptors, matchedKeypoints);

    if (_verbose) SD_TRACE1("Matched keypoints count : %1", matchedKeypoints.size());
    SD_TRACE1("Matched keypoints count : %1", matchedKeypoints.size());

    // Sort matches
    std::sort(matchedKeypoints.begin(), matchedKeypoints.end());

    if (_verbose) SD_TRACE2("Matches : min/max distances : %1, %2", matchedKeypoints[0].distance, matchedKeypoints[matchedKeypoints.size()-1].distance);

    // Select "good" matches
    std::vector<cv::DMatch> goodMatches;
    for (int k=0; k<matchedKeypoints.size();k++)
    {
        cv::DMatch m = matchedKeypoints[k];
        if (m.distance < _goodDistance)
        {
            goodMatches.push_back(m);
        }
    }
    if (_verbose) SD_TRACE1("Good matched keypoints count : %1", goodMatches.size());
    SD_TRACE1("Good matched keypoints count : %1", goodMatches.size());

    if (_verbose) {
        // DISPLAY
        cv::Mat out, refImageCopy, imageCopy;
        _refImage.copyTo(refImageCopy);
        image.copyTo(imageCopy);
        std::vector<std::vector<cv::DMatch> > vectorOfMatchedKeypoints;
        vectorOfMatchedKeypoints.push_back(goodMatches);
//        cv::drawMatches(refImageCopy, _refKeyPoints, imageCopy, keypoints, vectorOfMatchedKeypoints, out);
//        ImageCommon::displayMat(out, true, "Matched keypoints");
    }

    return goodMatches.size() >= _goodMatchesMinLimit;
}

//******************************************************************************************

bool BasicPairsDetector::matchTwoObjects(const cv::Mat &object1, const cv::Mat &object2, const cv::Mat &mask1, const cv::Mat &mask2)
{
    return false;
}

//******************************************************************************************

cv::Mat BasicPairsDetector::computeDescriptors(const cv::Mat &image, const cv::Mat &mask, std::vector<cv::KeyPoint> & keypoints)
{
    cv::Mat descriptors;
    _extractor->detectAndCompute(image,
                                mask,
                                keypoints,
                                descriptors);


    if (descriptors.empty())
    {
        SD_TRACE("Descriptors are not found");
        return descriptors;
    }

    if (_verbose)
    {
        cv::Mat keypointsImg;
        cv::drawKeypoints(image.mul(mask), keypoints, keypointsImg);
        ImageCommon::displayMat(keypointsImg, false, "Image keypoints");
    }

    if (_verbose) SD_TRACE1("Nb of keypoints (image) : %1", keypoints.size());
    SD_TRACE1("Nb of keypoints (image) : %1", keypoints.size());

    // Compute invariant Hu Moments and put them into descriptors
    if (!mask.empty())
    {
        cv::Mat objectWithMask = image.mul(mask);
        prependHuMoments(objectWithMask, descriptors);
    }
    else
    {
        prependHuMoments(image, descriptors);
    }

    return descriptors;
}

//******************************************************************************************

/*!
 * \brief prependHuMoments method to preprend hu moments to each keypoint descriptors
 * \param object Image which used to compute hu moments
 * \param descriptors
 * \return
 */
bool prependHuMoments(const cv::Mat & object, cv::Mat &descriptors)
{
    if (descriptors.type() == CV_32F)
    {
        cv::Moments mts = cv::moments(object);
        double hu[7];
        cv::HuMoments(mts, hu);
        cv::Mat huMat(descriptors.rows, 7, descriptors.depth(), cv::Scalar(0));
        for (int r = 0; r < huMat.rows; ++r)
        {
            for (int c=0; c<huMat.cols; ++c)
            {
                huMat.at<float>(r, c) = hu[c % 7];
            }
        }
//        ImageCommon::printMat(huMat, "huMat");

        cv::Mat augDescriptors;
        cv::hconcat(huMat, descriptors, augDescriptors);

        descriptors = augDescriptors;
        return true;
    }


    SD_TRACE("Descriptor type should be CV_32F");
    return false;
}

//******************************************************************************************

}
