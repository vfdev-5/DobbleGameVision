#ifndef BASICPAIRSDETECTOR_H
#define BASICPAIRSDETECTOR_H


// Opencv
#include <opencv2/features2d.hpp>

// Project
#include "PairsDetector.h"

namespace DGV
{

//******************************************************************************************

class BasicPairsDetector : public PairsDetector
{
public:
    BasicPairsDetector(float goodDistance = 0.29, int goodMatchesMinLimit = 10, bool verbose=false);

    // Compare with a reference object:
    virtual bool setupRefObject(const cv::Mat & image, const cv::Mat &mask = cv::Mat());
    virtual bool matchWithRefObject(const cv::Mat & image, const cv::Mat &mask = cv::Mat());

    // Compare two objects:
    virtual bool matchTwoObjects(const cv::Mat & object1, const cv::Mat & object2, const cv::Mat &mask1 = cv::Mat(), const cv::Mat &mask2 = cv::Mat());

protected:

    cv::Mat computeDescriptors(const cv::Mat & image, const cv::Mat &mask, std::vector<cv::KeyPoint> &keypoints);

    cv::Ptr<cv::Feature2D> _extractor;
    cv::Ptr<cv::DescriptorMatcher> _matcher;
    float _goodDistance;
    int _goodMatchesMinLimit;


    cv::Mat _refImage;
    std::vector<cv::KeyPoint> _refKeyPoints;

};

bool prependHuMoments(const cv::Mat &object, cv::Mat & descriptors);

//******************************************************************************************

}

#endif // BASICPAIRSDETECTOR_H
