#include "headers/three_d_handler.hpp"

bool imgBoundValid(const Mat& img, Point2f pt) {
    bool a = pt.x >= 3;
    bool b = pt.x < img.cols-2;
    bool c = pt.y >=3;
    bool d = pt.y < img.rows-2;
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

    K = (cv::Mat_<double>(3,3) << FX, 0, CX,
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

void ThreeDHandler::find3DPoints(const Mat& left_img, const Mat& right_img, vector<Point2f> &features, vector<Point3f> &feature_pts, vector<Point3f> &marker_pts, vector<Vec3b>& marker_color)
{
    vector<Point2f> left_kp, right_kp;
    matcher->match(left_img, left_kp, right_img, right_kp);

//    if(DEBUG) {
//        matcher->showMatches(left_img, left_kp, right_img, right_kp, "DEBUG: original matches");
//    }

    //find fundamental based on matches using RANSAC
    Mat inliner_mask;
    Mat K_T;
    transpose(K, K_T);
    Mat E, R, t;
    E  = findEssentialMat(left_kp, right_kp, K, RANSAC, 0.999, ransac_thres_essential, inliner_mask);
    recoverPose(E, left_kp, right_kp, K, R, t, inliner_mask);

    cout<<"Stereo matching inliners: "<<sum(inliner_mask)[0]<<" / "<<left_kp.size()<<endl;

    vector<Point2f> left_kp_inliner, right_kp_inliner;
    for(unsigned int i=0; i<left_kp.size(); i++) {
        if(inliner_mask.at<uchar>(i,0)>0) {
            features.push_back(left_kp[i]);
            left_kp_inliner.push_back(left_kp[i]);
            right_kp_inliner.push_back(right_kp[i]);
        }
    }

    Mat Pl, Pr;
    Pl = K*(Mat_<double>(3,4) <<1,0,0,0,0,1,0,0,0,0,1,0);
    hconcat(R, t, Pr);
    Pr = K*Pr;

    Mat F = findFundamentalMat(left_kp_inliner, right_kp_inliner, RANSAC, ransac_thres_essential, 0.999);
    cv::Mat left_rectified, right_rectified;
    calibrator->compute(left_img, right_img, F, left_kp_inliner, right_kp_inliner);
    calibrator->getRectifiedImages(left_img, right_img, left_rectified, right_rectified);

    Mat polarRect;
    hconcat(left_rectified, right_rectified, polarRect);
    for(int j = 0; j < polarRect.rows; j += (polarRect.rows / 50) ) {
        line(polarRect, Point(0, j), Point(polarRect.cols, j), Scalar(0, 255, 0), 1, 8);
    }

if(DEBUG) {
    namedWindow("Polar Rectification", WINDOW_NORMAL);
    imshow("Polar Rectification", polarRect);
}

//    Ptr<StereoSGBM> sbm = StereoSGBM::create( minDisparity, numberOfDisparities, SADWindowSize, SP1, SP2, disp12MaxDiff, preFilterCap, uniquenessRatio, speckleWindowSize, speckleRange);
//    Mat imgDisparity, disp_img;
//    sbm->compute( left_rectified, right_rectified, imgDisparity );
//    normalize(imgDisparity, disp_img, 0, 255, CV_MINMAX, CV_8U);
//    namedWindow("Polar Disparity", WINDOW_NORMAL);
//    imshow("Polar Disparity", disp_img);

    Mat left_mask, right_mask;
    lane_detector->detect(left_img, left_mask);
    lane_detector->detect(right_img, right_mask);

//if(DEBUG) {
//    namedWindow("DEBUG:Land Marker Left", WINDOW_NORMAL);
//    imshow("DEBUG:Land Marker Left", left_mask);
//    namedWindow("DEBUG:Land Marker Right", WINDOW_NORMAL);
//    imshow("DEBUG:Land Marker Right", right_mask);
//}

    vector<Point2d> left_marker_detected_cartesian, right_marker_detected_cartesian;
    for(int i=0; i<left_img.rows; i++) {
        for(int j=0; j<left_img.cols; j++) {
            if(left_mask.at<uchar>(i,j) > 0) {
                left_marker_detected_cartesian.push_back(Point2d(j,i));
            }
            if(right_mask.at<uchar>(i,j) > 0) {
                right_marker_detected_cartesian.push_back(Point2d(j,i));
            }
        }
    }

    vector<Point2d> left_marker_detected_polar, right_marker_detected_polar;
    calibrator->transformPointsToPolar(left_marker_detected_cartesian, left_marker_detected_polar, 1);
    calibrator->transformPointsToPolar(right_marker_detected_cartesian, right_marker_detected_polar, 2);

    Mat right_marker_detected_polar_mat = Mat::zeros(right_rectified.size(), CV_8U);
    Mat left_marker_detected_polar_mat = Mat::zeros(right_rectified.size(), CV_8U);
    for(unsigned int c=0; c<right_marker_detected_polar.size(); c++) {
        right_marker_detected_polar_mat.at<uchar>(right_marker_detected_polar[c].y, right_marker_detected_polar[c].x) = 1;
    }
    for(unsigned int c=0; c<left_marker_detected_polar.size(); c++) {
        left_marker_detected_polar_mat.at<uchar>(left_marker_detected_polar[c].y, left_marker_detected_polar[c].x) = 1;
    }

    vector<Point2d> left_marker_polar, right_marker_polar;
    int batch_size = 50;
    Mat left_batch, right_batch, diff;
    for(unsigned int c=0; c<left_marker_detected_polar.size(); c++) {
        int rho(left_marker_detected_polar[c].x), theta(left_marker_detected_polar[c].y);
        if(rho<batch_size || rho>=left_rectified.cols-batch_size-1) continue;

        left_batch = left_rectified(Rect(rho-batch_size, theta, batch_size*2, 1));
        double min_dist = -1;
        double second_dist = -1;
        int min_rho = -1;
        for(int i_rho=batch_size; i_rho<right_rectified.cols-batch_size-1; i_rho++) {
            if(right_marker_detected_polar_mat.at<uchar>(theta,i_rho) == 0) continue;
            right_batch = right_rectified(Rect(i_rho-batch_size, theta, batch_size*2, 1));
            diff = right_batch - left_batch;
            double dist = norm(diff);
            if(min_dist < 0 || min_dist > dist){
                second_dist = min_dist;
                min_dist = dist;
                min_rho = i_rho;
            }
        }
        if(min_rho<0 || min_dist/second_dist > 1) continue; //TODO

        //bi-directional
        right_batch = right_rectified(Rect(min_rho-batch_size, theta, batch_size*2, 1));
        min_dist = -1;
        second_dist = -1;
        int min_rho_rev = -1;
        for(int i_rho=batch_size; i_rho<left_rectified.cols-batch_size-1; i_rho++) {
            if(left_marker_detected_polar_mat.at<uchar>(theta,i_rho) == 0) continue;
            left_batch = left_rectified(Rect(i_rho-batch_size, theta, batch_size*2, 1));
            diff = right_batch - left_batch;
            double dist = norm(diff);
            if(min_dist < 0 || min_dist > dist){
                second_dist = min_dist;
                min_dist = dist;
                min_rho_rev = i_rho;
            }
        }

        if(abs(min_rho_rev-rho)<5)
        {
            left_marker_polar.push_back(left_marker_detected_polar[c]);
            right_marker_polar.push_back(Point2d(min_rho, theta));
        }
    }

    vector<Point2d> left_marker_cartesian, right_marker_cartesian;
    calibrator->transformPointsFromPolar(left_marker_polar, left_marker_cartesian, 1);
    calibrator->transformPointsFromPolar(right_marker_polar, right_marker_cartesian, 2);

    Mat marker_match_img;
    vconcat(left_img, right_img, marker_match_img);
    for(unsigned int i=0; i<left_marker_cartesian.size(); i++) {
        line(marker_match_img, left_marker_cartesian[i], Point(right_marker_cartesian[i].x, right_marker_cartesian[i].y+left_img.rows), Scalar(0, 255, 0));
    }

if(DEBUG) {
    namedWindow("Marker Match", WINDOW_NORMAL);
    imshow("Marker Match", marker_match_img);
}

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    Mat img_rep;
    Mat_<double> point_3d_tmp(4,1),point_2d_tmp(3,1);
    hconcat(left_img, right_img, img_rep);
    //reconstruct 3d feature points
    for(unsigned int i=0; i<left_kp_inliner.size(); i++) {
        Mat ul_skew = (Mat_<double>(3,3) << 0, -1, left_kp_inliner[i].y, 1, 0, -left_kp_inliner[i].x, -left_kp_inliner[i].y, left_kp_inliner[i].x, 0);
        Mat ur_skew = (Mat_<double>(3,3) << 0, -1, right_kp_inliner[i].y, 1, 0, -right_kp_inliner[i].x, -right_kp_inliner[i].y, right_kp_inliner[i].x, 0);
        Mat uPl = ul_skew*Pl;
        Mat uPr = ur_skew*Pr;
        Mat A, W, U, V;
        vconcat(uPl, uPr, A);
        SVDecomp(A, W, U, V, SVD::FULL_UV);
        transpose(V,V);

        double x = V.at<double>(0,3) / V.at<double>(3,3);
        double y = V.at<double>(1,3) / V.at<double>(3,3);
        double z = V.at<double>(2,3) / V.at<double>(3,3);
        if(z<0) continue;
        feature_pts.push_back(Point3f(x,y,z));

        pcl::PointXYZ p;
        p.x = x;
        p.y = y;
        p.z = z;
        point_cloud->push_back(p);

        point_3d_tmp(0)=x; point_3d_tmp(1)=y; point_3d_tmp(2)=z; point_3d_tmp(3) = 1;
        circle(img_rep, left_kp_inliner[i], 5, Scalar(255,0,0));
        point_2d_tmp = Pl*point_3d_tmp;
        point_2d_tmp /= point_2d_tmp(2);
        drawMarker(img_rep, Point2f(point_2d_tmp(0), point_2d_tmp(1)), Scalar(0,0,255), MARKER_CROSS, 5);

        circle(img_rep, Point2f(right_kp_inliner[i].x+left_img.cols, right_kp_inliner[i].y), 5, Scalar(255,0,0));
        point_2d_tmp = Pr*point_3d_tmp;
        point_2d_tmp /= point_2d_tmp(2);
        drawMarker(img_rep, Point2f(point_2d_tmp(0)+left_img.cols, point_2d_tmp(1)), Scalar(0,0,255), MARKER_CROSS, 5);
    }

    //reconstruct 3d marker points
    for(unsigned int i=0; i<left_marker_cartesian.size(); i++) {
        Mat ul_skew = (Mat_<double>(3,3) << 0, -1, left_marker_cartesian[i].y, 1, 0, -left_marker_cartesian[i].x, -left_marker_cartesian[i].y, left_marker_cartesian[i].x, 0);
        Mat ur_skew = (Mat_<double>(3,3) << 0, -1, right_marker_cartesian[i].y, 1, 0, -right_marker_cartesian[i].x, -right_marker_cartesian[i].y, right_marker_cartesian[i].x, 0);
        Mat uPl = ul_skew*Pl;
        Mat uPr = ur_skew*Pr;
        Mat A, W, U, V;
        vconcat(uPl, uPr, A);
        SVDecomp(A, W, U, V, SVD::FULL_UV);
        transpose(V,V);

        double x = V.at<double>(0,3) / V.at<double>(3,3);
        double y = V.at<double>(1,3) / V.at<double>(3,3);
        double z = V.at<double>(2,3) / V.at<double>(3,3);
        if(z<0) continue;
        marker_color.push_back(left_img.at<Vec3b>(left_marker_cartesian[i].y, left_marker_cartesian[i].x));
        marker_pts.push_back(Point3f(x,y,z));

        pcl::PointXYZ p;
        p.x = x;
        p.y = y;
        p.z = z;
        point_cloud->push_back(p);

        circle(img_rep, left_marker_cartesian[i], 5, Scalar(0,255,0));
        circle(img_rep, Point2f(right_marker_cartesian[i].x+left_img.cols, right_marker_cartesian[i].y), 5, Scalar(0,255,0));
    }

    assert(point_cloud->size()>0);
    pcl::io::savePCDFile("test_pcd.pcd", *point_cloud);

    for(unsigned int i=0; i<marker_pts.size(); i++) {
        point_3d_tmp(0)=marker_pts[i].x; point_3d_tmp(1)=marker_pts[i].y; point_3d_tmp(2)=marker_pts[i].z; point_3d_tmp(3) = 1;
        point_2d_tmp = Pl*point_3d_tmp;
        point_2d_tmp /= point_2d_tmp(2);
        drawMarker(img_rep, Point2f(point_2d_tmp(0), point_2d_tmp(1)), Scalar(0,0,255), MARKER_CROSS, 5);

        point_2d_tmp = Pr*point_3d_tmp;
        point_2d_tmp /= point_2d_tmp(2);
        drawMarker(img_rep, Point2f(point_2d_tmp(0)+left_img.cols, point_2d_tmp(1)), Scalar(0,0,255), MARKER_CROSS, 5);
    }

if(DEBUG) {
    namedWindow("DEBUG:Essential reprojection", WINDOW_NORMAL);
    imshow("DEBUG:Essential reprojection", img_rep);
}

    return;
}

bool ThreeDHandler::project(const Mat& obj_img, Mat &cur_img, const vector<Point2f>& features, const vector<Point3f>& feature_pts, const vector<Point3f>& marker_pts, const vector<Vec3b>& marker_color)
{
    vector<int> inliners_features;
    vector<Point2f> img_kp;
    matcher->match_given_kp(obj_img, features, cur_img, img_kp, inliners_features);

    if(inliners_features.size()<3) {
        cout<<"Not enough points for PnP, exiting..."<<endl;
        return false;
    }

    //register camera frame
    vector<Point3f> obj_pts;
    vector<Point2f> _obj_kp, _img_kp;
    Mat rep_img = cur_img.clone();
    for(unsigned int i=0; i<inliners_features.size(); i++)
    {
        obj_pts.push_back(feature_pts[inliners_features[i]]);
        _obj_kp.push_back(features[inliners_features[i]]);
        _img_kp.push_back(img_kp[i]);
    }


    cv::Mat rvec, t, inliners;
    cv::solvePnPRansac( obj_pts, _img_kp, K, camera_coeff, rvec, t, false, 500, ransac_thres_pnp, 0.999, inliners, cv::SOLVEPNP_ITERATIVE );

    if(inliners.rows<3) {
        cout<<"Not enough inlier "<<inliners.rows<<"/"<<inliners_features.size()<<" for PnP, exiting..."<<endl;
        return false;
    }

    cout<<"PnP inliners: "<<inliners.rows<<" / "<<_img_kp.size()<<endl;

if(DEBUG) {
    vector<Point3f> obj_pts_inlier;
    vector<Point2f> obj_kp_inlier, img_kp_inlier;
    for(int i=0; i<inliners.rows; i++)
    {
        obj_pts_inlier.push_back(obj_pts[inliners.at<int>(0,i)]);
        obj_kp_inlier.push_back(_obj_kp[inliners.at<int>(0,i)]);
        img_kp_inlier.push_back(_img_kp[inliners.at<int>(0,i)]);
        circle(rep_img, img_kp[inliners.at<int>(0,i)], 5, Scalar(255,0,0));
    }
//    matcher->showMatches(obj_img, obj_kp_inlier, cur_img, img_kp_inlier, "DEBUG:Project matches");

//    cout<<"Proj rotation: "<<rvec<<endl;
//    cout<<"Proj translation: "<<t<<endl;
    vector<Point2f> points_2d;
    projectPoints(obj_pts_inlier, rvec, t, K, camera_coeff, points_2d);
    for(unsigned int i=0; i<points_2d.size(); i++) {
        drawMarker(rep_img, points_2d[i], Scalar(0,0,255), MARKER_CROSS, 5);
    }
    namedWindow("DEBUG:PnP reprojection", WINDOW_NORMAL);
    imshow("DEBUG:PnP reprojection", rep_img);
}

    Mat R, P;
    Rodrigues(rvec, R);
    hconcat(R, t, P);

    //project road marker
    Mat canvas = Mat::zeros(cur_img.size(), CV_8UC3);
    cv::Mat_<double> p_homo(4,1);
    for(unsigned int i=0; i<marker_pts.size(); i++) {
        p_homo(0) = marker_pts[i].x; p_homo(1) = marker_pts[i].y; p_homo(2) = marker_pts[i].z; p_homo(3) = 1;
        Mat proj_p_homo = K*P*p_homo;
        Point2f proj_p(proj_p_homo.at<double>(0,0)/proj_p_homo.at<double>(2,0), proj_p_homo.at<double>(1,0)/proj_p_homo.at<double>(2,0));
        if(imgBoundValid(cur_img, proj_p))
        {
            rectangle(canvas, Point2f(proj_p.x-1, proj_p.y-1), Point2f(proj_p.x+1, proj_p.y+1), CvScalar(marker_color[i]));
        }
    }

    cur_img += canvas;
    return true;
}
