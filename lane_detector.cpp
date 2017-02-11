#include "lane_detector.hpp"

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

bool imgBoundValid(const Mat& img, Point2f pt) {
    bool a = pt.x >= 0;
    bool b = pt.x < img.cols;
    bool c = pt.y >=0;
    bool d = pt.y < img.rows;
    return a && b && c && d;
}

void LaneDetector::detectAndShow(const Mat& detImg, Mat& projImg, const Mat& trans, const Mat &K)
{
    vector<Point2f> markerPoints;
    detect(detImg, markerPoints);

    Mat mask = Mat::zeros(projImg.size(), CV_32F);
    //draw lanes
    for(auto& db_p:markerPoints)
    {
        Mat X = K.inv()*(Mat_<double>(3,1) << db_p.x, db_p.y, 1);
        Mat X_homo = (Mat_<double>(4,1) << X.at<double>(0), X.at<double>(1), X.at<double>(2), 1);
        Mat X_p = K*trans*X_homo;
        Point2f projP(X_p.at<double>(0)/X_p.at<double>(2), X_p.at<double>(1)/X_p.at<double>(2));
        cout<<"orig: "<<db_p<<endl;
        cout<<"proj: "<<projP<<endl;
        if((imgBoundValid(projImg, projP))) {
            mask.at<float>((int)projP.y, (int)projP.x) = 1.0f;
        }
    }

    blur(mask, mask, Size(10, 10));
    threshold(mask, mask, 0, 1, cv::THRESH_BINARY);
    mask.convertTo(mask, CV_8U);

    detImg.copyTo(projImg, mask);
}
