#ifndef ROTATOR_H
#define ROTATOR_H

#include <memory>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "MSAC.h"

#define MAX_NUM_LINES 200

using namespace std;
using namespace cv;

class Rotator
{
private:
    shared_ptr<MSAC> msac;

public:
    Rotator();
    void process(const Mat& input, Mat &output);
};

#endif // ROTATOR_H
