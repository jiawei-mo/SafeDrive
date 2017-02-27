#include "headers/lane_detector.hpp"

void LaneDetector::detect(const Mat& img, vector<Point2f>& markerPoints)
{
    //ROI
    Mat roi = img(Rect(0,img.rows/2,img.cols,img.rows/2));

    //Detection by color
//    Mat whiteHist, yellowHist;
//    inRange(roi, Scalar(180, 180, 190), Scalar(255, 255, 255), whiteHist);
//    inRange(roi, Scalar(0, 150, 170), Scalar(150, 255, 255), yellowHist);

//    imshow("lane test", whiteHist);
//    waitKey();

    //Canny
    Mat whiteEdge, yellowEdge;
//    GaussianBlur(whiteHist, whiteHist, Size(3,3), 2, 2);
//    GaussianBlur(yellowHist, yellowHist, Size(3,3), 2, 2);
    Canny( roi, whiteEdge, 50, 400, 3);
    Canny( roi, yellowEdge, 50, 400, 3);
    blur(whiteEdge, whiteEdge, Size(10, 10));
    blur(yellowEdge, yellowEdge, Size(10, 10));

    for(int i=0; i<roi.rows; i++)
    {
        for(int j=0; j<roi.cols; j++)
        {
            if(whiteEdge.at<uchar>(i, j))
            {
                markerPoints.push_back(Point(j, img.rows/2 + i));
            }
            if(yellowEdge.at<uchar>(i, j))
            {
                markerPoints.push_back(Point(j, img.rows/2 + i));
            }
        }
    }
}
