#include "headers/img_fetcher.hpp"

IMGFetcher::IMGFetcher()
{
    IMGFetcher::key = "AIzaSyBAVmBgHM26Lxosh60k9wtrUb1gNZBIorw";
    camera_K = (cv::Mat_<double>(3,3) << FX, 0, CX,
                                     0, FY, CY,
                                     0, 0, 1);

    camera_coeff = (cv::Mat_<double>(1,5) << CO1, CO2, CO3, CO4, CO5);
}

IMGFetcher::IMGFetcher(string _key)
{
    IMGFetcher::key = _key;
    camera_K = (cv::Mat_<double>(3,3) << FX, 0, CX,
                                     0, FY, CY,
                                     0, 0, 1);

    camera_coeff = (cv::Mat_<double>(1,5) << CO1, CO2, CO3, CO4, CO5);
}

bool IMGFetcher::get(Mat& output, Size size, float lan, float lon, float head, float pitch)
{
    double width = size.width;
    double height = size.height;
    ostringstream s;
    s<<"https://maps.googleapis.com/maps/api/streetview?size="<<width<<"x"<<height<<"&location="<<lan<<","<<lon
    <<"&heading="<<head<<"&pitch="<<pitch<<"&key="<<IMGFetcher::key;
    //        cout<<s.str()<<endl;
    VideoCapture cap(s.str());
    if(!cap.isOpened())
    {
        return false;
    }
    cap >> output;
    if(output.empty()) {
        return false;
    }
    return true;
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
