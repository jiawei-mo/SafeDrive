#ifndef IMGFetcher_HPP
#define IMGFetcher_HPP

#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/features2d.hpp"
using namespace cv;
using namespace std;

class IMGFetcher
{
public:
    IMGFetcher();
    IMGFetcher(string _key);
    void get(Mat &output, Size size, float lan, float lon, float head, float pitch, string localSearch="");
private:
    string key;
};

#endif // IMGFetcher_HPP
