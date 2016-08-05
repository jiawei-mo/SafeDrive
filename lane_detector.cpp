#include "lane_detector.hpp"

void LaneDetector::detect(const Mat& img, vector<Point2f>& whitePoints, vector<Point2f>& yellowPoints, bool showImg)
{
    Mat imgCopy = img.clone();
    //ROI
    Mat roi = imgCopy(Rect(0,img.rows/2,img.cols,img.rows/2));

    //Detection by color
    Mat whiteHist, yellowHist;
    inRange(roi, Scalar(180, 180, 190), Scalar(255, 255, 255), whiteHist);
    inRange(roi, Scalar(0, 150, 170), Scalar(150, 255, 255), yellowHist);

    //Canny
    Mat whiteEdge, yellowEdge;
    GaussianBlur(whiteHist, whiteHist, Size(3,3), 2, 2);
    GaussianBlur(yellowHist, yellowHist, Size(3,3), 2, 2);
    Canny( whiteHist, whiteEdge, 50, 400, 3);
    Canny( yellowHist, yellowEdge, 50, 400, 3);

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

    if(showImg) {
        Mat laneImg = img.clone();
        for(int i=0; i<(int)whitePoints.size(); i++)
        {
            laneImg.at<Vec3b>(whitePoints[i]) = Vec3b(255, 255, 255);
        }
        for(int i=0; i<(int)yellowPoints.size(); i++)
        {
            laneImg.at<Vec3b>(yellowPoints[i]) = Vec3b(0, 255, 255);
        }
        imshow("Lane Result", laneImg);
    }
}

bool imgBoundValid(const Mat& img, Point2f pt) {
    bool a = pt.x >= 0;
    bool b = pt.x < img.cols;
    bool c = pt.y >=0;
    bool d = pt.y < img.rows;
    return a && b && c && d;
}

void LaneDetector::detectAndProject(const Mat& detImg, Mat& projImg, const Mat& homo, bool showImg)
{
    vector<Point2f> whitePoints;
    vector<Point2f> yellowPoints;
    detect(detImg, whitePoints, yellowPoints, showImg);
    vector<Point2f> whiteProjectedPoints, yellowProjectedPoints;
    if(whitePoints.size() > 0) {
        perspectiveTransform(whitePoints, whiteProjectedPoints, homo);
    }
    if(yellowPoints.size()) {
        perspectiveTransform(yellowPoints, yellowProjectedPoints, homo);
    }

    //draw lanes
    for(int i=0; i<(int)whiteProjectedPoints.size(); i++)
    {
        if((imgBoundValid(projImg, whiteProjectedPoints[i]))) {
            projImg.at<Vec3b>(whiteProjectedPoints[i]) = Vec3b(255, 255, 255);
        }
    }
    for(int i=0; i<(int)yellowProjectedPoints.size(); i++)
    {
        if((imgBoundValid(projImg, yellowProjectedPoints[i]))) {
            projImg.at<Vec3b>(yellowProjectedPoints[i]) = Vec3b(0, 255, 255);
        }
    }
}
