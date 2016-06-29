#include "line_detector.hpp"

vector<Point2f> Line_detector::process(const Mat img)
{
//	namedWindow("lanes", WINDOW_NORMAL);
	Mat imgCopy = img.clone();
	//ROI
	Mat roi = imgCopy(Rect(img.cols/4,img.rows/2,img.cols/2,img.rows/2));

	//Detection by color
	Mat histImg;
	inRange(roi, Scalar(190, 190, 190), Scalar(255, 255, 255), histImg);

	//Canny
	Mat edgeImg;
	GaussianBlur(histImg, histImg, Size(7,7), 3, 3);
	Canny( histImg, edgeImg, 100, 300, 3);

	vector<Point2f> lanePoints;
	for(int i=0; i<edgeImg.rows; i++)
	{
		for(int j=0; j<edgeImg.cols; j++)
		{
			if(edgeImg.at<uchar>(i, j))
			{
				lanePoints.push_back(Point(img.cols/4 + j, img.rows/2 + i));
			}
		}
	}
//	Mat projectedImg = img.clone();
//	for(int i=0; i<(int)lanePoints.size(); i++)
//	{
//		projectedImg.at<Vec3b>(lanePoints[i]) = Vec3b(0, 255, 255);
//	}
//	imshow("lanes",projectedImg);
	return lanePoints;
}

