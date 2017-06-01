#include "headers/img_fetcher.hpp"

IMGFetcher::IMGFetcher()
{
    IMGFetcher::key = "AIzaSyBAVmBgHM26Lxosh60k9wtrUb1gNZBIorw";
}

void IMGFetcher::setCam(const vector<float>& _K, const vector<float>& _D)
{
    camera_K = (cv::Mat_<double>(3,3) << _K[0], 0, _K[2],
                                         0, _K[1], _K[3],
                                         0, 0, 1);

    camera_coeff = (cv::Mat_<double>(1,5) << _D[0], _D[1], _D[2], _D[3], _D[4]);

}

bool IMGFetcher::get_local(Mat& output, string filename)
{
    output = imread(filename);
    if(output.empty()) {
        return false;
    }

    Mat orig = output.clone();
    cv::undistort(orig, output, camera_K, camera_coeff);
    GaussianBlur(output, output, Size(BS,BS), BV, BV);
    return true;
}
