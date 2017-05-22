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

void ThreeDHandler::findDisparity(vector<KeyPoint> &feature_disp, Mat& marker_disp, Mat &Q, Mat& left_img, Mat& right_img)
{
    marker_disp = Mat::zeros(left_img.size(), CV_32F);
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
    vector<Point2d> left_kp_cartesian, right_kp_cartesian;
    for(unsigned int i=0; i<left_kp.size(); i++) {
        if(inliner_mask.at<uchar>(i,0)>0) {
            left_kp_inliner.push_back(left_kp[i]);
            right_kp_inliner.push_back(right_kp[i]);
            left_kp_cartesian.push_back(Point2d(left_kp[i].x, left_kp[i].y));
            right_kp_cartesian.push_back(Point2d(right_kp[i].x, right_kp[i].y));
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

    vector<Point2d> left_kp_polar, right_kp_polar, left_marker_polar, right_marker_polar;
    calibrator->transformPointsToPolar(left_kp_cartesian, left_kp_polar, 1);
    calibrator->transformPointsToPolar(right_kp_cartesian, right_kp_polar, 2);
    calibrator->transformPointsToPolar(left_marker_cartesian, left_marker_polar, 1);
    calibrator->transformPointsToPolar(right_marker_cartesian, right_marker_polar, 2);

    for(unsigned int c=0; c<left_kp_cartesian.size(); c++) {
        float d = left_kp_polar[c].x - right_kp_polar[c].x;
        feature_disp.push_back(KeyPoint(left_kp_cartesian[c], d));
    }

    Mat right_mask_polar = Mat::zeros(right_rectified.size(), CV_8U);
    for(unsigned int c=0; c<right_marker_polar.size(); c++) {
        right_mask_polar.at<uchar>(right_marker_polar[c].y, right_marker_polar[c].x) = 1;
    }

    int batch_size = 30;
    for(unsigned int c=0; c<left_marker_polar.size(); c++) {
        int x(left_marker_polar[c].x), y(left_marker_polar[c].y);
        if(x<batch_size) continue;

        Mat left_batch = left_rectified(Rect(x-batch_size, y, batch_size*2, 1));
        double min_dist = -1;
        int min_pos = -1;
        for(int ix=batch_size; ix<right_rectified.cols-batch_size; ix++) {
            if(right_mask_polar.at<uchar>(y,ix) == 0) continue;
            Mat right_batch = right_rectified(Rect(ix-batch_size, y, batch_size*2, 1));
            Mat diff = right_batch - left_batch;
            double dist = norm(diff);
            if(min_dist < 0 || min_dist > dist){
                min_dist = dist;
                min_pos = ix;
            }
        }
        float d = float(x-min_pos);
        int cx(left_marker_cartesian[c].x), cy(left_marker_cartesian[c].y);
        marker_disp.at<float>(cy, cx) = d;
    }
    Mat disp_img;
    normalize(marker_disp, disp_img, 0, 255, CV_MINMAX, CV_8U);

    namedWindow("DEBUG:Dense Disparity", WINDOW_NORMAL);
    imshow("DEBUG:Dense Disparity", disp_img);

    waitKey();
    return;


//    Mat img_rep;
//    hconcat(left_img, right_img, img_rep);
//    Mat_<double> point_3d_tmp(4,1),point_2d_tmp(3,1);
//    for(unsigned int i=0; i<left_kp.size(); i++) {
//        float disp = left_kp[i].x  - right_kp[i].x;
//        feature_disp.push_back(KeyPoint(left_kp[i], disp));

//        //reprojection
//        point_3d_tmp(0) = left_kp[i].x; point_3d_tmp(1) = left_kp[i].y; point_3d_tmp(2) = disp; point_3d_tmp(3) = 1;
//        point_3d_tmp = Q*point_3d_tmp;
//        point_3d_tmp /= point_3d_tmp(3);

//        circle(img_rep, left_kp[i], 5, Scalar(255,0,0));
//        point_2d_tmp = P1*point_3d_tmp;
//        point_2d_tmp /= point_2d_tmp(2);
//        drawMarker(img_rep, Point2f(point_2d_tmp(0), point_2d_tmp(1)), Scalar(0,0,255), MARKER_CROSS, 5);

//        circle(img_rep, Point2f(right_kp[i].x+left_img.cols, right_kp[i].y), 5, Scalar(255,0,0));
//        point_2d_tmp = P2*point_3d_tmp;
//        point_2d_tmp /= point_2d_tmp(2);
//        drawMarker(img_rep, Point2f(point_2d_tmp(0)+left_img.cols, point_2d_tmp(1)), Scalar(0,0,255), MARKER_CROSS, 5);
//    }

//if(DEBUG) {
//    namedWindow("DEBUG:Essential reprojection", WINDOW_NORMAL);
//    imshow("DEBUG:Essential reprojection", img_rep);
//    waitKey(1);
//}

}

void ThreeDHandler::project(const Mat& obj_img, Mat& cur_img, vector<KeyPoint> &feature_disp, const Mat& marker_disp, const Mat &Q)
{
    vector<Point2f> img_kp;
    matcher->match_given_kp(obj_img, feature_disp, cur_img, img_kp);

    if(feature_disp.size()<3) {
        cout<<"Not enough points for PnP, exiting..."<<endl;
        return;
    }

    //register camera frame
    vector<Point3f> obj_pts;
    vector<Point2f> _obj_kp, _img_kp;
    Mat rep_img = cur_img.clone();
    cv::Mat_<double> point_3d_tmp(4,1);
    for(unsigned int i=0; i<feature_disp.size(); i++)
    {
        float x = feature_disp[i].pt.x;
        float y = feature_disp[i].pt.y;
        float d = feature_disp[i].size;
        point_3d_tmp(0) = x; point_3d_tmp(1) = y; point_3d_tmp(2) = d; point_3d_tmp(3) = 1;
        point_3d_tmp = Q*point_3d_tmp;
        point_3d_tmp /= point_3d_tmp(3);

        obj_pts.push_back(Point3f(point_3d_tmp(0), point_3d_tmp(1), point_3d_tmp(2)));
        _obj_kp.push_back(feature_disp[i].pt);
        _img_kp.push_back(img_kp[i]);
    }


    cv::Mat rvec, t, inliers;
    cv::solvePnPRansac( obj_pts, _img_kp, camera_K, camera_coeff, rvec, t, false, 100, ransac_thres_pnp, 0.999, inliers, cv::SOLVEPNP_ITERATIVE );

    if(inliers.rows<3) {
        cout<<"Not enough inlier for PnP, exiting..."<<endl;
        return;
    }

    cout<<"PnP inliers: "<<inliers.rows<<" / "<<_img_kp.size()<<endl;

if(DEBUG) {
    vector<Point3f> obj_pts_inlier;
    vector<Point2f> obj_kp_inlier, img_kp_inlier;
    for(int i=0; i<inliers.rows; i++)
    {
        obj_pts_inlier.push_back(obj_pts[inliers.at<int>(0,i)]);
        obj_kp_inlier.push_back(_obj_kp[inliers.at<int>(0,i)]);
        img_kp_inlier.push_back(_img_kp[inliers.at<int>(0,i)]);
        circle(rep_img, img_kp[inliers.at<int>(0,i)], 5, Scalar(255,0,0));
    }
    matcher->showMatches(obj_img, obj_kp_inlier, cur_img, img_kp_inlier, "DEBUG:Project matches");

//    cout<<"Proj rotation: "<<rvec<<endl;
//    cout<<"Proj translation: "<<t<<endl;
    vector<Point2f> points_2d;
    projectPoints(obj_pts_inlier, rvec, t, camera_K, camera_coeff, points_2d);
    for(unsigned int i=0; i<points_2d.size(); i++) {
        drawMarker(rep_img, points_2d[i], Scalar(0,0,255), MARKER_CROSS, 5);
    }
    namedWindow("DEBUG:PnP reprojection", WINDOW_NORMAL);
    imshow("DEBUG:PnP reprojection", rep_img);
    waitKey(1);
}

    Mat R, P;
    Rodrigues(rvec, R);
    hconcat(R, t, P);

    Mat lane_mask;
    lane_detector->detect(obj_img, lane_mask);
    //project road marker
    Mat canvas = Mat::zeros(cur_img.size(), CV_8UC3);
    cv::Mat_<double> p_homo(4,1);
    for(int x=0; x<obj_img.cols; x++) {
        for(int y=0; y<obj_img.rows; y++) {
            if(lane_mask.at<uchar>(y,x) == 0) continue;
            float d = marker_disp.at<float>(y,x);
            if(d == 0)
            {
                continue;
            }

            p_homo(0) = x; p_homo(1) = y; p_homo(2) = d; p_homo(3) = 1;
            p_homo = Q*p_homo;
            p_homo /= p_homo(3);
            Mat proj_p_homo = camera_K*P*p_homo;
            Point2f proj_p(proj_p_homo.at<double>(0,0)/proj_p_homo.at<double>(2,0), proj_p_homo.at<double>(1,0)/proj_p_homo.at<double>(2,0));
            if(imgBoundValid(cur_img, proj_p))
            {
                canvas.at<Vec3b>(proj_p.y, proj_p.x) += (obj_img.at<Vec3b>(y, x) / 2);
            }
        }
    }
    GaussianBlur(canvas, canvas, Size(BS, BS), BV);

    cur_img += canvas;
}
