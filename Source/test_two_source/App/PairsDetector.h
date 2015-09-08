#ifndef PAIRSDETECTOR_H
#define PAIRSDETECTOR_H

// Opencv
#include <opencv2/core.hpp>


namespace DGV
{

//******************************************************************************************

class PairsDetector
{
public:
    PairsDetector(bool verbose=false);

    // Compare with a reference object:
    virtual bool setupRefObject(const cv::Mat & image, const cv::Mat &mask = cv::Mat()) = 0;
    virtual bool matchWithRefObject(const cv::Mat & image, const cv::Mat &mask = cv::Mat()) = 0;

    // Compare two objects:
    virtual bool matchTwoObjects(const cv::Mat & object1, const cv::Mat & object2, const cv::Mat &mask1 = cv::Mat(), const cv::Mat &mask2 = cv::Mat()) = 0;

    //

protected:

    bool _verbose;

};

//******************************************************************************************

}

#endif // PAIRSDETECTOR_H
