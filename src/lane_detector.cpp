#include "headers/lane_detector.hpp"

void LaneDetector::detect(const Mat& img, Mat& mask)
{
    Mat whiteHist, yellowHist;
    Mat empty_top = Mat::zeros(img.rows/2, img.cols, CV_8U);

    Mat roi = img(Rect(0,img.rows/2,img.cols,img.rows/2)).clone();
    Mat roi_gray, roi_hsv;
    cvtColor(roi, roi_gray, COLOR_BGR2GRAY);
    cvtColor(roi, roi_hsv, COLOR_BGR2HSV);
    inRange(roi_hsv, Scalar(0, 0, 0.7*255), Scalar(180, 0.05*255, 255), whiteHist);
    inRange(roi_hsv, Scalar(25/2, 0.35*255, 0.5*255), Scalar(55/2, 0.7*255, 255), yellowHist);
    Mat yellowAndWhite = yellowHist + whiteHist;
    medianBlur(yellowAndWhite, yellowAndWhite, 5);
    Canny(yellowAndWhite, yellowAndWhite, 40, 100);
    vconcat(empty_top, yellowAndWhite, mask);
}
