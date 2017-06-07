#include "headers/lane_detector.hpp"

double REMOVE_TOP_RATIO = 1.0/2.0;

void LaneDetector::detect(const Mat& img, Mat& mask)
{
    Mat imgROI = img(Rect(0,REMOVE_TOP_RATIO*img.rows,img.cols,img.rows-REMOVE_TOP_RATIO*img.rows)).clone();
    GaussianBlur(imgROI, imgROI, Size(3,3), 2, 2);

    // Canny algorithm
    Mat contours;
    Canny(imgROI,contours,75,150);

    Mat roi_hsv, whiteHist, yellowHist;
    cvtColor(imgROI, roi_hsv, COLOR_BGR2HSV);
    inRange(roi_hsv, Scalar(-1, -1, 0.7*255), Scalar(180, 0.5*255, 256), whiteHist);
    inRange(roi_hsv, Scalar(25/2, 0.2*255, 0.35*255), Scalar(55/2, 0.7*255, 256), yellowHist);
    Mat yellowAndWhite = yellowHist + whiteHist;
    blur(yellowAndWhite, yellowAndWhite, Size(3,3));
    contours = yellowAndWhite.mul(contours);

    int houghVote = 80;
    std::vector<Vec4i> lines;
    int line_counter = 0;
    while(line_counter < 30 && houghVote > 0)
    {
        HoughLinesP(contours,lines,1,CV_PI/180, houghVote, 30, 10);
        houghVote -= 5;

        line_counter = lines.size();
    }

    // Draw the lines
    std::vector<Vec4i>::const_iterator it= lines.begin();
    Mat hough(imgROI.size(),CV_8U,Scalar(0));
    while (it!=lines.end()) {
        {
            line( hough, Point((*it)[0], (*it)[1]),
                    Point((*it)[2], (*it)[3]), Scalar(255,255,255), 1);
        }
        it++;
    }

    Mat lanes;
    threshold(hough, lanes, 0, 255, THRESH_BINARY);

    Mat empty_top = Mat::zeros(REMOVE_TOP_RATIO*img.rows, img.cols, CV_8U);
    vconcat(empty_top, lanes, mask);
}
