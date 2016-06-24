#include "line_detector.hpp"

Mat Line_detector::process(const Mat img)
{
	Mat imgCopy = img.clone();
	//ROI
	Mat roi = imgCopy(Rect(img.cols/4,img.rows/2,img.cols/2,img.rows/2));

	//Detection by color
	Mat histImg, edgeImg;
	inRange(roi, cv::Scalar(190, 190, 190), cv::Scalar(255, 255, 255), histImg);

	//Canny
	GaussianBlur(histImg, histImg, Size(7,7), 3, 3);
	Canny( histImg, edgeImg, 100, 300, 3);

	roi.setTo(Scalar(0,255,0), edgeImg);
	roi.copyTo(imgCopy(Rect(img.cols/4,img.rows/2,img.cols/2,img.rows/2)));
	return imgCopy;
}

