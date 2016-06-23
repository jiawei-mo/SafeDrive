#include "line_detector.hpp"

Mat Line_detector::process(const Mat img)
{
	//ROI
	Mat roi = img(Rect(img.cols/4,img.rows/2,img.cols/2,img.rows/2));

	//Detection by color
	Mat hsvImg, edgeImg;
	cvtColor(roi, hsvImg, CV_BGR2HSV);
	inRange(hsvImg, cv::Scalar(0, 0, 200), cv::Scalar(150, 50, 255), hsvImg);

	//Canny
	GaussianBlur(hsvImg, hsvImg, Size(7,7), 3, 3);
	Canny( hsvImg, edgeImg, 50, 150, 3);

	roi.setTo(Scalar(0,255,0), edgeImg);
	Mat res = img.clone();
	roi.copyTo(res(Rect(img.cols/4,img.rows/2,img.cols/2,img.rows/2)));
	return res;
}

