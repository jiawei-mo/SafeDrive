#include "headers/lane_detector.hpp"

void LaneDetector::detect(const Mat& img, Mat& mask)
{
    Mat whiteHist, yellowHist;
    Mat empty_top = Mat::zeros(img.rows/2, img.cols, CV_8U);

    Mat roi = img(Rect(0,img.rows/2,img.cols,img.rows/2)).clone();

    Mat roi_gray, lanes;
    cvtColor(roi, roi_gray, COLOR_BGR2GRAY);
    GaussianBlur(roi_gray, roi_gray, Size(3,3), 2, 2);
    Canny(roi_gray, lanes, 60, 80);
    namedWindow("DEBUG:Canny Lanes", WINDOW_NORMAL);
    imshow("DEBUG:Canny Lanes", lanes);

    Mat roi_hsv;
    cvtColor(roi, roi_hsv, COLOR_BGR2HSV);
    inRange(roi_hsv, Scalar(0, 0, 0.7*255), Scalar(180, 0.05*255, 255), whiteHist);
    inRange(roi_hsv, Scalar(25/2, 0.2*255, 0.5*255), Scalar(55/2, 0.7*255, 255), yellowHist);
    Mat yellowAndWhite = yellowHist + whiteHist;
    medianBlur(yellowAndWhite, yellowAndWhite, 5);
    blur(yellowAndWhite, yellowAndWhite, Size(5,5));
    namedWindow("DEBUG:Threshold Lanes", WINDOW_NORMAL);
    imshow("DEBUG:Threshold Lanes", yellowAndWhite);

    Mat YWline = yellowAndWhite.mul(lanes);
    namedWindow("DEBUG:Lanes", WINDOW_NORMAL);
    imshow("DEBUG:Lanes", YWline);

    vconcat(empty_top, YWline, mask);
}
