#ifndef ECC_H
#define ECC_H

#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/features2d.hpp"
using namespace cv;

class ECC
{
public:
    ECC();
    double findTransformECC(InputArray templateImage,
                                InputArray inputImage,
                                InputOutputArray warpMatrix,
                                int motionType,
                                TermCriteria criteria,
                                InputArray inputMask);
};

#endif // ECC_H
