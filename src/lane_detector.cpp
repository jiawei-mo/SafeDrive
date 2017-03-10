#include "headers/lane_detector.hpp"

void LaneDetector::detect(const Mat& img, vector<Point2f>& markerPoints)
{
    //ROI
    Mat roi = img(Rect(0,img.rows/2,img.cols,img.rows/2));
//    Mat roi = img(Rect(0,0,img.cols,img.rows));

    //Detection by color
    Mat whiteHist, yellowHist;
    inRange(roi, Scalar(200, 200, 200), Scalar(255, 255, 255), whiteHist);
    inRange(roi, Scalar(0, 200, 200), Scalar(150, 255, 255), yellowHist);

    Mat yelloAndWhite = yellowHist + whiteHist;

    namedWindow("Lane detected", WINDOW_NORMAL);
    imshow("Lane detected", yelloAndWhite);
    waitKey(1);

    for(int i=0; i<yelloAndWhite.rows; i++)
    {
        for(int j=0; j<yelloAndWhite.cols; j++)
        {
            if(yelloAndWhite.at<uchar>(i, j))
            {
                markerPoints.push_back(Point(j, img.rows/2+i));
            }
        }
    }

//        for(int i=300; i<600; i++)
//        {
//            for(int j=600; j<700; j++)
//            {
//                markerPoints.push_back(Point(j, i));
//            }
//        }
}
