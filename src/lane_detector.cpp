#include "headers/lane_detector.hpp"

void LaneDetector::detect(const Mat& img, Mat& mask)
{
    Mat whiteHist, yellowHist;
    Mat empty_top = Mat::zeros(img.rows/2, img.cols, CV_8U);

    Mat roi = img(Rect(0,img.rows/2,img.cols,img.rows/2));
    inRange(roi, Scalar(200, 200, 200), Scalar(255, 255, 255), whiteHist);
    inRange(roi, Scalar(0, 150, 150), Scalar(180, 255, 255), yellowHist);
    Mat yelloAndWhite = yellowHist + whiteHist;

    vconcat(empty_top, yelloAndWhite, mask);
}
