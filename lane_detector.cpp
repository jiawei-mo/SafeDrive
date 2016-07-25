#include "lane_detector.hpp"

LaneRes* LaneDetector::process(const Mat& img)
{
//	namedWindow("lanes", WINDOW_NORMAL);
    Mat imgCopy = img.clone();
    //ROI
    Mat roi = imgCopy(Rect(0,img.rows/2,img.cols,img.rows/2));

    //Detection by color
    Mat whiteHist, yellowHist;
    inRange(roi, Scalar(180, 180, 190), Scalar(255, 255, 255), whiteHist);
    inRange(roi, Scalar(0, 150, 170), Scalar(150, 255, 255), yellowHist);

//    imshow("lane", histImg);
//    waitKey(10000000);
    //Canny
    Mat whiteEdge, yellowEdge;
    GaussianBlur(whiteHist, whiteHist, Size(3,3), 2, 2);
    GaussianBlur(yellowHist, yellowHist, Size(3,3), 2, 2);
    Canny( whiteHist, whiteEdge, 50, 400, 3);
    Canny( yellowHist, yellowEdge, 50, 400, 3);

    vector<Point2f> whitePoints, yellowPoints;
    for(int i=0; i<roi.rows; i++)
    {
        for(int j=0; j<roi.cols; j++)
        {
            if(whiteEdge.at<uchar>(i, j))
            {
                whitePoints.push_back(Point(j, img.rows/2 + i));
            }
            if(yellowEdge.at<uchar>(i, j))
            {
                yellowPoints.push_back(Point(j, img.rows/2 + i));
            }
        }
    }

    Mat laneImg = img.clone();
    for(int i=0; i<(int)whitePoints.size(); i++)
    {
        laneImg.at<Vec3b>(whitePoints[i]) = Vec3b(255, 255, 255);
    }
    for(int i=0; i<(int)yellowPoints.size(); i++)
    {
        laneImg.at<Vec3b>(yellowPoints[i]) = Vec3b(0, 255, 255);
    }
    LaneRes *res = new LaneRes(laneImg, whitePoints, yellowPoints);
    return res;
}

