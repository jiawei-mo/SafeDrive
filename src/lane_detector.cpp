#include "headers/lane_detector.hpp"

void LaneDetector::detect(const Mat& img, Mat& mask)
{
    Mat whiteHist, yellowHist;
    Mat empty_top = Mat::zeros(img.rows/2, img.cols, CV_8U);

    Mat roi = img(Rect(0,img.rows/2,img.cols,img.rows/2)).clone();
    cvtColor(roi, roi, COLOR_BGR2HSV);
    inRange(roi, Scalar(0, 0, 0.7*255), Scalar(180, 0.05*255, 255), whiteHist);
    inRange(roi, Scalar(25/2, 0.5*255, 0.5*255), Scalar(55/2, 0.6*255, 255), yellowHist);
    Mat yelloAndWhite = yellowHist + whiteHist;
    blur(yelloAndWhite, yelloAndWhite, Size(5,5));

    vconcat(empty_top, yelloAndWhite, mask);
}
