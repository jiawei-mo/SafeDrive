#include "headers/lane_detector.hpp"

double REMOVE_TOP_RATIO = 53.0/72.0;

void LaneDetector::detect(const Mat& img, vector<Mat>& white_yellow_mask)
{
    Mat imgROI = img(Rect(0,REMOVE_TOP_RATIO*img.rows,img.cols,img.rows-REMOVE_TOP_RATIO*img.rows)).clone();
    GaussianBlur(imgROI, imgROI, Size(3,3), 2, 2);

    // Canny algorithm
    Mat contours;
    Canny(imgROI,contours,75,150);

    Mat roi_hsv, whiteHist, yellowHist;
    cvtColor(imgROI, roi_hsv, COLOR_BGR2HSV);
    inRange(roi_hsv, Scalar(20/2, 0, 0.7*255), Scalar(280/2, 0.2*256, 256), whiteHist);
    inRange(roi_hsv, Scalar(25/2, 0.2*255, 0.35*255), Scalar(55/2, 0.7*255, 256), yellowHist);

    blur(yellowHist, yellowHist, Size(7,7));
    blur(whiteHist, whiteHist, Size(7,7));
    Mat yellowContours = yellowHist.mul(contours);
    Mat whiteContours = whiteHist.mul(contours);

    Mat yellowLanes, whiteLanes;
    threshold(yellowContours, yellowLanes, 0, 255, THRESH_BINARY);
    threshold(whiteContours, whiteLanes, 0, 255, THRESH_BINARY);

    Mat empty_top = Mat::zeros(REMOVE_TOP_RATIO*img.rows, img.cols, CV_8U);
    Mat yellow_mask, white_mask;
    vconcat(empty_top, whiteLanes, white_mask);
    white_yellow_mask.push_back(white_mask);
    vconcat(empty_top, yellowLanes, yellow_mask);
    white_yellow_mask.push_back(yellow_mask);
}

void LaneDetector::houghDetect(const Mat& img, Mat& mask) {
    int houghVote = 200;
    std::vector<Vec4i> lines;
    int line_counter=0;
    while(line_counter < 30 && houghVote > 0)
    {
        HoughLinesP(img,lines,5,3*CV_PI/180, houghVote, 20, 10);
        houghVote -= 5;

        std::vector<Vec4i>::const_iterator it= lines.begin();
        line_counter=0;
        while (it!=lines.end()) {
            float theta = atan2(fabs((*it)[1]-(*it)[3]) , fabs((*it)[0]-(*it)[2]));
            if ( (theta > 0.2 && theta < 2.9))
            {
                line_counter++;
            }
            ++it;
        }
    }
cout<<line_counter<<"  "<<houghVote<<endl;
    // Draw the lines
    std::vector<Vec4i>::const_iterator it= lines.begin();
    Mat hough(img.size(),CV_8U,Scalar(0));
    while (it!=lines.end()) {
        float theta = atan2(fabs((*it)[1]-(*it)[3]) , fabs((*it)[0]-(*it)[2]));
        if ( (theta > 0.2 && theta < 2.9))
        {
            line( hough, Point((*it)[0], (*it)[1]),
                    Point((*it)[2], (*it)[3]), Scalar(255,255,255), 1);
        }
        it++;
    }
    mask = hough;
}




//int houghVote = 200;
//std::vector<Vec2f> lines;
//int line_counter=0;
//while(line_counter < 30 && houghVote > 0)
//{
//    HoughLines(img,lines,10,CV_PI/180, houghVote);
//    houghVote -= 5;

//    std::vector<Vec2f>::const_iterator it= lines.begin();
//    line_counter=0;
//    while (it!=lines.end()) {
//        float theta= (*it)[1];
//        if ( !(theta > 1.45 && theta < 1.65))
//        {
//            line_counter++;
//        }
//        ++it;
//    }
//}
//cout<<line_counter<<"  "<<houghVote<<endl;
//// Draw the lines
//std::vector<Vec2f>::const_iterator it= lines.begin();
//Mat hough(img.size(),CV_8U,Scalar(0));
//while (it!=lines.end()) {
//    float rho= (*it)[0];   // first element is distance rho
//    float theta= (*it)[1]; // second element is angle theta
//    if ( !(theta > 1.45 && theta < 1.65))
//    {
//        // point of intersection of the line with first row
//        Point pt1(rho/cos(theta),0);
//        // point of intersection of the line with last row
//        Point pt2((rho-img.rows*sin(theta))/cos(theta),img.rows);
//        line( hough, pt1, pt2, Scalar(255,255,255), 1);
//    }
//    it++;
//}
//mask = hough;
//}
