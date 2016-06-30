#include "gsvFetcher.hpp"

GSVFetcher::GSVFetcher()
{
	GSVFetcher::key = "AIzaSyBAVmBgHM26Lxosh60k9wtrUb1gNZBIorw";
}

GSVFetcher::GSVFetcher(string _key)
{
	GSVFetcher::key = _key;
}

Mat GSVFetcher::get(Size size, float lan, float lon, float head, float pitch)
{
	double width = size.width;
	double height = size.height;
	std::ostringstream s;
	s<<"https://maps.googleapis.com/maps/api/streetview?size="<<width<<"x"<<height<<"&location="<<lan<<","<<lon
			<<"&heading="<<head<<"&pitch="<<pitch<<"&key="<<GSVFetcher::key;
	cout<<s.str()<<endl;
	VideoCapture cap(s.str());
	Mat res = Mat::zeros(size, CV_8U);
	if(!cap.isOpened())
	{
		return res;
	}
	cap >> res;
	return res;
}
