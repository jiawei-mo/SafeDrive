#ifndef GSVFETCHER_HPP
#define GSVFETCHER_HPP

#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/features2d.hpp"
using namespace cv;
using namespace std;

class GSVFetcher
{
public:
    GSVFetcher();
    GSVFetcher(string _key);
    void get(Mat &output, Size size, float lan, float lon, float head, float pitch);
private:
    string key;
};

#endif // GSVFETCHER_HPP
