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
    IMGFetcher(string _key);
    bool get(Mat &output, Size size, float lan, float lon, float head, float pitch);
    bool get_local(Mat& output, string filename);
private:
    string key;
};

#endif // IMGFetcher_HPP
