#include "headers/three_d_handler.hpp"

bool imgBoundValid(const Mat& img, Point2f pt) {
    bool a = pt.x >= 0;
    bool b = pt.x < img.cols;
    bool c = pt.y >=0;
    bool d = pt.y < img.rows;
    return a && b && c && d;
}

ThreeDHandler::ThreeDHandler()
{}

ThreeDHandler::~ThreeDHandler()
{}

ThreeDHandler::ThreeDHandler(const shared_ptr<Matcher> _matcher)
{
    matcher = _matcher;
    calibrator = shared_ptr<PolarCalibration>(new PolarCalibration());
    calibrator->toggleShowCommonRegion(false);
    calibrator->toggleShowIterations(false);
    lane_detector = shared_ptr<LaneDetector>(new LaneDetector());
    ransac_thres_essential = RTE / 1.0f;
    ransac_thres_pnp = RTP / 1.0f;

    camera_K = (cv::Mat_<double>(3,3) << FX, 0, CX,
                                     0, FY, CY,
                                     0, 0, 1);

    camera_coeff = cv::Mat_<double>::zeros(1,5);
}
void ThreeDHandler::changeParam(const shared_ptr<Matcher> _matcher, float rte, float rtp)
{
    matcher = _matcher;
    ransac_thres_essential = rte;
    ransac_thres_pnp = rtp;
}

void ThreeDHandler::findCorrespondence(const Mat& left_img, const Mat& right_img, vector<pair<Point2f, Point2f> >& marker_corres)
{
    vector<Point2f> left_kp, right_kp;
    matcher->match(left_img, left_kp, right_img, right_kp);

    if(DEBUG) {
        matcher->showMatches(left_img, left_kp, right_img, right_kp, "DEBUG: original matches");
    }

    //find fundamental based on matches using RANSAC
    Mat inliner_mask;
    Mat F = findFundamentalMat(left_kp, right_kp, RANSAC, ransac_thres_essential, 0.999, inliner_mask);

    cout<<"Essential inliers: "<<sum(inliner_mask)[0]<<" / "<<left_kp.size()<<endl;

    vector<Point2f> left_kp_inliner, right_kp_inliner;
    for(unsigned int i=0; i<left_kp.size(); i++) {
        if(inliner_mask.at<uchar>(i,0)>0) {
            left_kp_inliner.push_back(left_kp[i]);
            right_kp_inliner.push_back(right_kp[i]);
        }
    }

    cv::Mat left_rectified, right_rectified;
    calibrator->compute(left_img, right_img, F, left_kp_inliner, right_kp_inliner);
    calibrator->getRectifiedImages(left_img, right_img, left_rectified, right_rectified);

    Mat polarRect;
    hconcat(left_rectified, right_rectified, polarRect);
    for(int j = 0; j < polarRect.rows; j += (polarRect.rows / 20) )
        line(polarRect, Point(0, j), Point(polarRect.cols, j), Scalar(0, 255, 0), 1, 8);
    namedWindow("Polar Rectification", WINDOW_NORMAL);
    imshow("Polar Rectification", polarRect);

    Mat left_mask, right_mask;
    lane_detector->detect(left_img, left_mask);
    lane_detector->detect(right_img, right_mask);

    namedWindow("DEBUG:Land Marker", WINDOW_NORMAL);
    imshow("DEBUG:Land Marker", left_mask);
    waitKey(1);

    vector<Point2d> left_marker_cartesian, right_marker_cartesian;
    for(int i=0; i<left_img.rows; i++) {
        for(int j=0; j<left_img.cols; j++) {
            if(left_mask.at<uchar>(i,j) > 0) {
                left_marker_cartesian.push_back(Point2d(j,i));
            }
            if(right_mask.at<uchar>(i,j) > 0) {
                right_marker_cartesian.push_back(Point2d(j,i));
            }
        }
    }

    vector<Point2d> left_marker_polar, right_marker_polar;
    calibrator->transformPointsToPolar(left_marker_cartesian, left_marker_polar, 1);
    calibrator->transformPointsToPolar(right_marker_cartesian, right_marker_polar, 2);

    Mat right_marker_polar_mat = Mat::zeros(right_rectified.size(), CV_8U);
    for(unsigned int c=0; c<right_marker_polar.size(); c++) {
        right_marker_polar_mat.at<uchar>(right_marker_polar[c].y, right_marker_polar[c].x) = 1;
    }

    vector<Point2d> left_corres_polar, right_corres_polar;
    int batch_size = 10;
    for(unsigned int c=0; c<left_marker_polar.size(); c++) {
        int theta(left_marker_polar[c].x), rho(left_marker_polar[c].y);
        if(theta<batch_size) continue;

        Mat left_batch = left_rectified(Rect(theta-batch_size, rho, batch_size*2, 1));
        double min_dist = -1;
        int min_theta = -1;
        for(int i_theta=batch_size; i_theta<right_rectified.cols-batch_size; i_theta++) {
            if(right_marker_polar_mat.at<uchar>(rho,i_theta) == 0) continue;
            Mat right_batch = right_rectified(Rect(i_theta-batch_size, rho, batch_size*2, 1));
            Mat diff = right_batch - left_batch;
            double dist = norm(diff);
            if(min_dist < 0 || min_dist > dist){
                min_dist = dist;
                min_theta = i_theta;
            }
        }
        if(min_theta<0 || min_dist > 100) continue; //TODO
        left_corres_polar.push_back(left_marker_polar[c]);
        right_corres_polar.push_back(Point2d(min_theta, rho));
    }

    vector<Point2d> left_corres_cartesian, right_corres_cartesian;
    calibrator->transformPointsFromPolar(left_corres_polar, left_corres_cartesian, 1);
    calibrator->transformPointsFromPolar(right_corres_polar, right_corres_cartesian, 2);

    Mat marker_match_img;
    hconcat(left_img, right_img, marker_match_img);
    for(unsigned int i=0; i<left_corres_cartesian.size(); i++) {
        marker_corres.push_back(pair<Point2d, Point2d>(left_corres_cartesian[i], right_corres_cartesian[i]));
        line(marker_match_img, left_corres_cartesian[i], Point(right_corres_cartesian[i].x+left_img.cols, right_corres_cartesian[i].y), Scalar(0, 255, 0));
    }
    namedWindow("Marker Match", WINDOW_NORMAL);
    imshow("Marker Match", marker_match_img);

    return;
}

void ThreeDHandler::project(const Mat& left_img, const Mat& right_img, Mat& cur_img, const vector<pair<Point2f, Point2f> >& marker_corres)
{
    vector<Point2f> kp_tmp1, kp_tmp2;
    matcher->match(cur_img, kp_tmp1, left_img, kp_tmp2);
    Mat cl_F = findFundamentalMat(kp_tmp1, kp_tmp2, RANSAC, ransac_thres_essential, 0.999, noArray());

    matcher->match(cur_img, kp_tmp1, right_img, kp_tmp2);
    Mat cr_F = findFundamentalMat(kp_tmp1, kp_tmp2, RANSAC, ransac_thres_essential, 0.999, noArray());

    Mat A;
    Mat row_u = Mat::ones(1,3,CV_64F);
    for(unsigned int i=0; i<marker_corres.size(); i++) {
        row_u.at<double>(0,0) = marker_corres[i].first.x;
        row_u.at<double>(0,1) = marker_corres[i].first.y;
        Mat A_l = row_u*cl_F;

        row_u.at<double>(0,0) = marker_corres[i].second.x;
        row_u.at<double>(0,1) = marker_corres[i].second.y;
        Mat A_r = row_u*cr_F;

        vconcat(A_l, A_r, A);

        cout<<A<<endl;
        Mat W, U, V;
        SVDecomp(A, W, U, V, SVD::FULL_UV);
        transpose(V,V);

        int u = V.at<double>(0,2) / V.at<double>(2,2);
        int v = V.at<double>(1,2) / V.at<double>(2,2);

        if(u>0 && u<cur_img.cols && v>0 && v<cur_img.rows) {
            cur_img.at<Vec3b>(v,u) = Vec3b(255,0,0);
        }
    }
}
