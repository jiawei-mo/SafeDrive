#include "gsv_fetcher.hpp"

GSVFetcher::GSVFetcher()
{
    GSVFetcher::key = "AIzaSyBAVmBgHM26Lxosh60k9wtrUb1gNZBIorw";
}

GSVFetcher::GSVFetcher(string _key)
{
    GSVFetcher::key = _key;
}

void GSVFetcher::get(Mat& output, Size size, float lan, float lon, float head, float pitch)
{
    double width = size.width;
    double height = size.height;
    ostringstream s;
    s<<"https://maps.googleapis.com/maps/api/streetview?size="<<width<<"x"<<height<<"&location="<<lan<<","<<lon
            <<"&heading="<<head<<"&pitch="<<pitch<<"&key="<<GSVFetcher::key;
    VideoCapture cap(s.str());
    if(!cap.isOpened())
    {
        return;
    }
    cap >> output;
    return;
}
