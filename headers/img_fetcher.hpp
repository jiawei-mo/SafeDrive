#ifndef IMGFetcher_HPP
#define IMGFetcher_HPP

#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/features2d.hpp"
#include "parameters.hpp"

using namespace cv;
using namespace std;

class IMGFetcher
{
public:
    Mat camera_K;
    Mat camera_coeff;
    IMGFetcher();
    void setCam(const vector<float>& _K, const vector<float>& _D);
    bool get_local(Mat& output, string filename);
private:
    string key;
};

#endif // IMGFetcher_HPP
