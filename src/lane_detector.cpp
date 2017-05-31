#include "headers/lane_detector.hpp"
#define PI 3.1415926
void LaneDetector::detect(const Mat& img, Mat& mask)
{
    Mat imgROI = img(Rect(0,img.rows/2,img.cols,img.rows/2)).clone();
    GaussianBlur(imgROI, imgROI, Size(3,3), 2, 2);

    // Canny algorithm
    Mat contours;
    Canny(imgROI,contours,100,200);

    Mat roi_hsv, whiteHist, yellowHist;
    cvtColor(imgROI, roi_hsv, COLOR_BGR2HSV);
    inRange(roi_hsv, Scalar(-1, -1, 0.7*255), Scalar(181, 0.05*255, 256), whiteHist);
    inRange(roi_hsv, Scalar(25/2, 0.2*255, 0.35*255), Scalar(55/2, 0.7*255, 256), yellowHist);
    Mat yellowAndWhite = yellowHist + whiteHist;
    medianBlur(yellowAndWhite, yellowAndWhite, 15);
    blur(yellowAndWhite, yellowAndWhite, Size(5,5));

    contours = yellowAndWhite.mul(contours);

    int houghVote = 200;
    std::vector<Vec2f> lines;
    int line_counter = 0;
    while(line_counter < 30 && houghVote > 0){
        HoughLines(contours,lines,1,PI/180, houghVote);
        houghVote -= 5;

        std::vector<Vec2f>::const_iterator it= lines.begin();
        line_counter=0;
        while (it!=lines.end()) {
            float theta= (*it)[1]; // second element is angle theta
            if ( !(theta > 1.55 && theta < 1.85))
            {
                line_counter++;
            }
            ++it;
        }
    }

    // Draw the lines
    std::vector<Vec2f>::const_iterator it= lines.begin();
    Mat hough(imgROI.size(),CV_8U,Scalar(0));
    while (it!=lines.end()) {

        float rho= (*it)[0];   // first element is distance rho
        float theta= (*it)[1]; // second element is angle theta

        if ( !(theta > 1.55 && theta < 1.85))
        {
            // point of intersection of the line with first row
            Point pt1(rho/cos(theta),0);
            // point of intersection of the line with last row
            Point pt2((rho-imgROI.rows*sin(theta))/cos(theta),imgROI.rows);
            line( hough, pt1, pt2, Scalar(255,255,255), 1);
        }
        ++it;
    }

    Mat lanes;
    blur(contours, contours, Size(3,3));
    threshold(contours, contours, 0, 255, THRESH_BINARY);

if(DEBUG) {
    static bool oppo_direct=false;
    Mat tmp = hough/2 + contours/2;
    string windowname;
    if(oppo_direct) {
        namedWindow("Hough and contour-Right", WINDOW_NORMAL);
        windowname = "Hough and contour-Right";
    } else {
        namedWindow("Hough and contour-Left", WINDOW_NORMAL);
        windowname = "Hough and contour-Left";
    }
    oppo_direct = !oppo_direct;
    imshow(windowname, tmp);
}

    bitwise_and(hough, contours, lanes);
    threshold(lanes, lanes, 0, 255, THRESH_BINARY);

    Mat empty_top = Mat::zeros(img.rows/2, img.cols, CV_8U);
    vconcat(empty_top, lanes, mask);
}
