#include "img_fetcher.hpp"

IMGFetcher::IMGFetcher()
{
    IMGFetcher::key = "AIzaSyBAVmBgHM26Lxosh60k9wtrUb1gNZBIorw";
}

IMGFetcher::IMGFetcher(string _key)
{
    IMGFetcher::key = _key;
}

bool IMGFetcher::get(Mat& output, Size size, float lan, float lon, float head, float pitch, string localSearch)
{
    if(localSearch.size()<3) {              //search on GSV
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
    } else {                               //local search
        ostringstream s;
        s<<localSearch<<lan<<"_"<<lon
                <<"_"<<head<<"_"<<pitch<<".jpg";
//        cout<<s.str()<<endl;
        output = imread(s.str());
        if(output.empty()) {
            return false;
        }
        return true;
    }

}
