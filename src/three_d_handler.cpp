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

    camera_coeff = cv::Mat_<double>::zeros(1,5);
}

void ThreeDHandler::setCamK(const vector<float>& _K)
{
    K = (cv::Mat_<double>(3,3) << _K[0], 0, _K[2],
                                  0, _K[1], _K[3],
                                  0, 0, 1);
}

void ThreeDHandler::changeParam(const shared_ptr<Matcher> _matcher, float rte, float rtp, int mlrd, int mto)
{
    matcher = _matcher;
    ransac_thres_essential = rte;
    ransac_thres_pnp = rtp;
    max_lane_reproj_dist = mlrd;
    max_theta_offset = mto;
}

bool ThreeDHandler::getPose(const Mat& left_img, const Mat& right_img, Mat& R, Mat& t, vector<Point2f>& left_kp_inliner, vector<Point2f>& right_kp_inliner, const string& window_name)
{
    vector<Point2f> left_kp, right_kp;
    matcher->match(left_img, left_kp, right_img, right_kp);

    if(DEBUG) {
        matcher->showMatches(left_img, left_kp, right_img, right_kp, window_name);
    }

    //find fundamental based on matches using RANSAC
    Mat inliner_mask, E;
    E  = findEssentialMat(left_kp, right_kp, K, RANSAC, 0.999, ransac_thres_essential, inliner_mask);
    recoverPose(E, left_kp, right_kp, K, R, t, inliner_mask);

    cout<<"Stereo matching inliers: "<<sum(inliner_mask)[0]<<" / "<<left_kp.size()<<endl;

    if(sum(inliner_mask)[0]<8)
    {
        cout<<"Not enough points for Stereo, exiting..."<<endl;
        return false;
    }

    for(unsigned int i=0; i<left_kp.size(); i++) {
        if(inliner_mask.at<uchar>(i,0)>0) {
            left_kp_inliner.push_back(left_kp[i]);
            right_kp_inliner.push_back(right_kp[i]);
        }
    }
    return true;
}

void ThreeDHandler::matchRoadMarkers(const Mat& left_rectified, const Mat& right_rectified,
                                     const vector<vector<Point2d> >& left_marker_detected_cartesian, const vector<vector<Point2d> >& right_marker_detected_cartesian,
                                     vector<vector<Point2d> >& left_marker_cartesian, vector<vector<Point2d> >& right_marker_cartesian)
{
    for(int t=0; t<2; t++)
    {
        vector<Point2d> left_marker_detected_polar, right_marker_detected_polar;
        calibrator->transformPointsToPolar(left_marker_detected_cartesian[t], left_marker_detected_polar, 1);
        calibrator->transformPointsToPolar(right_marker_detected_cartesian[t], right_marker_detected_polar, 2);

        Mat right_marker_detected_polar_mat = Mat::zeros(right_rectified.size(), CV_8U);
        Mat left_marker_detected_polar_mat = Mat::zeros(right_rectified.size(), CV_8U);
        for(unsigned int c=0; c<left_marker_detected_polar.size(); c++) {
            left_marker_detected_polar_mat.at<uchar>(left_marker_detected_polar[c].y, left_marker_detected_polar[c].x) = 1;
        }
        for(unsigned int c=0; c<right_marker_detected_polar.size(); c++) {
            right_marker_detected_polar_mat.at<uchar>(right_marker_detected_polar[c].y, right_marker_detected_polar[c].x) = 1;
        }

        vector<Point2d> left_marker_polar, right_marker_polar;
        int batch_size_h = 5;
        int batch_size_v = 5;
        Mat left_batch, right_batch, diff;
        for(unsigned int c=0; c<left_marker_detected_polar.size(); c++) {
            int rho(left_marker_detected_polar[c].x), theta(left_marker_detected_polar[c].y);
            if(theta<(max_theta_offset+batch_size_v) || (theta+max_theta_offset+batch_size_v)>=left_rectified.rows ||
                    rho<batch_size_h || rho>=left_rectified.cols-batch_size_h-1) continue;
            left_batch = left_rectified(Rect(rho-batch_size_h, theta-batch_size_v, batch_size_h*2+1, 2*batch_size_v+1));
            double min_dist = -1;
            int min_rho = -1;
            int min_theta_offset = 0;
            for(int theta_offset=-max_theta_offset; theta_offset<=max_theta_offset; theta_offset++) {
                for(int i_rho=batch_size_h; i_rho<right_rectified.cols-batch_size_h-1; i_rho++) {
                    if(right_marker_detected_polar_mat.at<uchar>(theta+theta_offset,i_rho) == 0) continue;
                    right_batch = right_rectified(Rect(i_rho-batch_size_h, theta+theta_offset-batch_size_v, batch_size_h*2+1, 2*batch_size_v+1));
                    diff = right_batch - left_batch;
                    double dist = norm(diff);
                    if(min_dist < 0 || min_dist > dist){
                        min_dist = dist;
                        min_rho = i_rho;
                        min_theta_offset = theta_offset;
                    }
                }
            }
            if(min_rho<0) continue;

            //bi-directional
            //        right_batch = right_rectified(Rect(min_rho-batch_size_h, theta+min_theta_offset, batch_size_h*2, 1));
            //        min_dist = -1;
            //        int min_rho_rev = -1;
            //        for(int i_rho=batch_size_h; i_rho<left_rectified.cols-batch_size_h-1; i_rho++) {
            //            if(left_marker_detected_polar_mat.at<uchar>(theta,i_rho) == 0) continue;
            //            left_batch = left_rectified(Rect(i_rho-batch_size_h, theta, batch_size_h*2, 1));
            //            diff = right_batch - left_batch;
            //            double dist = norm(diff);
            //            if(min_dist < 0 || min_dist > dist){
            //                min_dist = dist;
            //                min_rho_rev = i_rho;
            //            }
            //        }

            //        if(abs(min_rho_rev-rho)<15)
            {
                left_marker_polar.push_back(left_marker_detected_polar[c]);
                right_marker_polar.push_back(Point2d(min_rho, theta+min_theta_offset));
            }
        }
        vector<Point2d> _left_marker_cartesian, _right_marker_cartesian;
        calibrator->transformPointsFromPolar(left_marker_polar, _left_marker_cartesian, 1);
        calibrator->transformPointsFromPolar(right_marker_polar, _right_marker_cartesian, 2);
        left_marker_cartesian.push_back(_left_marker_cartesian);
        right_marker_cartesian.push_back(_right_marker_cartesian);
    }
}

bool ThreeDHandler::find3DPoints(const Mat& left_img, const Mat& right_img, vector<Point2f> &features, Mat& additional_desc, vector<Point3f>& feature_pts, vector<vector<Point3f> >& marker_pts)
{
    // find pose between left img and right img
    Mat R, t;
    vector<Point2f> left_kp_inliner, right_kp_inliner;
    if(!getPose(left_img, right_img, R, t, left_kp_inliner, right_kp_inliner, "Original match")) return false;

    features = left_kp_inliner;
    matcher->get_desc(right_img, right_kp_inliner, additional_desc);

    //get perspective projection matrix
    Mat Pl, Pr;
    Pl = K*(Mat_<double>(3,4) <<1,0,0,0,0,1,0,0,0,0,1,0);
    hconcat(R, t, Pr);
    Pr = K*Pr;

    Mat img_rep;
    hconcat(left_img, right_img, img_rep);
    Mat_<double> point_3d_tmp(4,1),point_2d_tmp(3,1);

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

    //detect road markers
    vector<Mat> left_mask, right_mask;
    lane_detector->detect(left_img, left_mask);
    lane_detector->detect(right_img, right_mask);

    //rectify imgs
    Mat F = findFundamentalMat(left_kp_inliner, right_kp_inliner, RANSAC, ransac_thres_essential, 0.999);
    cv::Mat left_rectified, right_rectified;
    calibrator->compute(left_img, right_img, F, left_kp_inliner, right_kp_inliner);
    calibrator->getRectifiedImages(left_img, right_img, left_rectified, right_rectified);

    //prepare road markers
    vector<Point2d> left_yellow_marker_detected, left_white_marker_detected, right_yellow_marker_detected, right_white_marker_detected;
    for(int i=0; i<left_img.rows; i++) {
        for(int j=0; j<left_img.cols; j++) {
            if(left_mask[0].at<uchar>(i,j) > 0) {
                left_yellow_marker_detected.push_back(Point2d(j,i));
            }
            if(left_mask[1].at<uchar>(i,j) > 0) {
                left_white_marker_detected.push_back(Point2d(j,i));
            }
            if(right_mask[0].at<uchar>(i,j) > 0) {
                right_yellow_marker_detected.push_back(Point2d(j,i));
            }
            if(right_mask[1].at<uchar>(i,j) > 0) {
                right_white_marker_detected.push_back(Point2d(j,i));
            }
        }
    }
    vector<vector<Point2d>> left_marker_detected, right_marker_detected;
    left_marker_detected.push_back(left_yellow_marker_detected);
    left_marker_detected.push_back(left_white_marker_detected);
    right_marker_detected.push_back(right_yellow_marker_detected);
    right_marker_detected.push_back(right_white_marker_detected);

    //match road markers between left img and right img based on rectified imgs
    vector<vector<Point2d>> left_marker_matched, right_marker_matched;
    matchRoadMarkers(left_rectified, right_rectified,
                     left_marker_detected, right_marker_detected,
                     left_marker_matched, right_marker_matched);

    //reconstruct 3d marker points
    vector<Point3f> _marker_pts;
    for(int t=0; t<2; t++)
    {
        _marker_pts.clear();
        for(unsigned int i=0; i<left_marker_matched[t].size(); i++) {
            Mat ul_skew = (Mat_<double>(3,3) << 0, -1, left_marker_matched[t][i].y, 1, 0, -left_marker_matched[t][i].x, -left_marker_matched[t][i].y, left_marker_matched[t][i].x, 0);
            Mat ur_skew = (Mat_<double>(3,3) << 0, -1, right_marker_matched[t][i].y, 1, 0, -right_marker_matched[t][i].x, -right_marker_matched[t][i].y, right_marker_matched[t][i].x, 0);
            Mat uPl = ul_skew*Pl;
            Mat uPr = ur_skew*Pr;
            Mat A, W, U, V;
            vconcat(uPl, uPr, A);
            SVDecomp(A, W, U, V, SVD::FULL_UV);
            transpose(V,V);

            double x = V.at<double>(0,3) / V.at<double>(3,3);
            double y = V.at<double>(1,3) / V.at<double>(3,3);
            double z = V.at<double>(2,3) / V.at<double>(3,3);
            //        if(z<0)     //mark z<0 as purple
            //        {
            //            circle(img_rep, left_marker_matched[t][i], 3, Scalar(255,0,255));
            //            circle(img_rep, Point2f(right_marker_matched[t][i].x+left_img.cols, right_marker_matched[t][i].y), 3, Scalar(255,0,255));
            //            continue;
            //        }

            //remove outlier by reporjection dist
            point_3d_tmp(0)=x; point_3d_tmp(1)=y; point_3d_tmp(2)=z; point_3d_tmp(3) = 1;
            point_2d_tmp = Pl*point_3d_tmp;
            point_2d_tmp /= point_2d_tmp(2);
            double dist = (left_marker_matched[t][i].x-point_2d_tmp(0))*(left_marker_matched[t][i].x-point_2d_tmp(0));
            dist += (left_marker_matched[t][i].y-point_2d_tmp(1))*(left_marker_matched[t][i].y-point_2d_tmp(1));

            point_2d_tmp = Pr*point_3d_tmp;
            point_2d_tmp /= point_2d_tmp(2);
            dist += (right_marker_matched[t][i].x-point_2d_tmp(0))*(right_marker_matched[t][i].x-point_2d_tmp(0));
            dist += (right_marker_matched[t][i].y-point_2d_tmp(1))*(right_marker_matched[t][i].y-point_2d_tmp(1));
            if(dist > max_lane_reproj_dist)      //mark large dist as yellow
            {

                circle(img_rep, left_marker_matched[t][i], 3, Scalar(0,255,255));
                circle(img_rep, Point2f(right_marker_matched[t][i].x+left_img.cols, right_marker_matched[t][i].y), 3, Scalar(0,255,255));
                continue;
            }

            _marker_pts.push_back(Point3f(x,y,z));

            circle(img_rep, left_marker_matched[t][i], 5, Scalar(0,255,0));
            circle(img_rep, Point2f(right_marker_matched[t][i].x+left_img.cols, right_marker_matched[t][i].y), 5, Scalar(0,255,0));
        }
        marker_pts.push_back(_marker_pts);
    }


if(DEBUG) {
    Mat left_lane = left_img.clone();
    cvtColor(left_lane, left_lane, CV_BGR2GRAY);
    cvtColor(left_lane, left_lane, CV_GRAY2BGR);
    left_lane.setTo(Scalar(0,255,255), left_mask[0]);
    left_lane.setTo(Scalar(255,255,255), left_mask[1]);
    Mat right_lane = right_img.clone();
    cvtColor(right_lane, right_lane, CV_BGR2GRAY);
    cvtColor(right_lane, right_lane, CV_GRAY2BGR);
    right_lane.setTo(Scalar(0,255,255), right_mask[0]);
    right_lane.setTo(Scalar(255,255,255), right_mask[1]);
    Mat lane_concat;
    hconcat(left_lane, right_lane, lane_concat);
    namedWindow("DEBUG:Road Marker", WINDOW_NORMAL);
    imshow("DEBUG:Road Marker", lane_concat);


    Mat left_rectified_lane, right_rectified_lane;
    calibrator->getRectifiedImages(left_lane, right_lane, left_rectified_lane, right_rectified_lane);
    Mat lane_rectified_concat;
    hconcat(left_rectified_lane, right_rectified_lane, lane_rectified_concat);
    for(int j = 0; j < lane_rectified_concat.rows; j += (lane_rectified_concat.rows / 50) ) {
        line(lane_rectified_concat, Point(0, j), Point(lane_rectified_concat.cols, j), Scalar(0, 255, 0), 1, 8);
    }
    namedWindow("DEBUG:Rectified Road Marker", WINDOW_NORMAL);
    imshow("DEBUG:Rectified Road Marker", lane_rectified_concat);

    Mat marker_match_img;
    vconcat(left_img, right_img, marker_match_img);
    for(int t=0; t<2; t++)
    {
        for(unsigned int i=0; i<left_marker_matched[t].size(); i++) {
            line(marker_match_img, left_marker_matched[t][i], Point(right_marker_matched[t][i].x, right_marker_matched[t][i].y+left_img.rows), Scalar(0, 255, 0));
        }
    }
    namedWindow("Marker Match", WINDOW_NORMAL);
    imshow("Marker Match", marker_match_img);

    for(int t=0; t<2; t++)
    {
        for(unsigned int i=0; i<marker_pts[t].size(); i++) {
            point_3d_tmp(0)=marker_pts[t][i].x; point_3d_tmp(1)=marker_pts[t][i].y; point_3d_tmp(2)=marker_pts[t][i].z; point_3d_tmp(3) = 1;
            point_2d_tmp = Pl*point_3d_tmp;
            point_2d_tmp /= point_2d_tmp(2);
            drawMarker(img_rep, Point2f(point_2d_tmp(0), point_2d_tmp(1)), Scalar(0,0,255), MARKER_CROSS, 5);

            point_2d_tmp = Pr*point_3d_tmp;
            point_2d_tmp /= point_2d_tmp(2);
            drawMarker(img_rep, Point2f(point_2d_tmp(0)+left_img.cols, point_2d_tmp(1)), Scalar(0,0,255), MARKER_CROSS, 5);
        }
    }
    namedWindow("DEBUG:Essential reprojection", WINDOW_NORMAL);
    imshow("DEBUG:Essential reprojection", img_rep);
}

    return true;
}

bool ThreeDHandler::project(const Mat& obj_img, const Mat &cur_img,
                            const vector<Point2f>& features, const Mat& additional_desc,
                            const vector<Point3f>& feature_pts, const vector<vector<Point3f> >& marker_pts,
                            Mat& output)
{
    vector<int> inliers_features, inliers_features_addition;
    vector<Point2f> img_kp, img_kp_addition;
    matcher->match_given_kp(obj_img, features, cur_img, img_kp, inliers_features);
    matcher->match_given_desc(additional_desc, cur_img, img_kp_addition, inliers_features_addition);
    img_kp.insert(img_kp.end(), img_kp_addition.begin(), img_kp_addition.end());
    inliers_features.insert(inliers_features.end(), inliers_features_addition.begin(), inliers_features_addition.end());
    if(inliers_features.size()<3)
    {
        cout<<"Not enough points for PnP, exiting..."<<endl;
        return false;
    }

    //register camera frame
    vector<Point3f> obj_pts;
    vector<Point2f> _obj_kp, _img_kp;
    Mat rep_img = cur_img.clone();
    for(unsigned int i=0; i<inliers_features.size(); i++)
    {
        obj_pts.push_back(feature_pts[inliers_features[i]]);
        _obj_kp.push_back(features[inliers_features[i]]);
        _img_kp.push_back(img_kp[i]);
    }

    cv::Mat rvec, t, inliers;
    double inlier_ratio = 0.0;
    float thres = 1.0;
    while(inlier_ratio < 0.7 && thres < ransac_thres_pnp) {
        cv::solvePnPRansac( obj_pts, _img_kp, K, camera_coeff, rvec, t, false, 1000, thres, 0.99, inliers, cv::SOLVEPNP_ITERATIVE );
        inlier_ratio = float(inliers.rows) / float(inliers_features.size());
        thres *= 1.2;
    }
    cout<<"PnP Thres: "<<thres<<endl;

    if(inliers.rows < 3) {
        matcher->showMatches(obj_img, _obj_kp, cur_img, _img_kp, "Fail:Project matches");
        cout<<"Not enough inlier ("<<inliers.rows<<"/"<<inliers_features.size()<<") for PnP, exiting..."<<endl;
        return false;
    }

    cout<<"PnP inliers: "<<inliers.rows<<" / "<<_img_kp.size()<<endl;

    cout<<"Rotation:"<<endl<<rvec<<endl;
    cout<<"Translation:"<<endl<<t<<endl;

    Mat R, P;
    Rodrigues(rvec, R);
    hconcat(R, t, P);

    //project road marker
//    Mat canvas = Mat::zeros(cur_img.size(), CV_8U);
    output = cur_img.clone();
    cv::Mat_<double> p_homo(4,1);
    for(int t=0; t<2; t++)
    {
        for(unsigned int i=0; i<marker_pts[t].size(); i++) {
            p_homo(0) = marker_pts[t][i].x; p_homo(1) = marker_pts[t][i].y; p_homo(2) = marker_pts[t][i].z; p_homo(3) = 1;
            Mat proj_p_homo = K*P*p_homo;
            Point2f proj_p(proj_p_homo.at<double>(0,0)/proj_p_homo.at<double>(2,0), proj_p_homo.at<double>(1,0)/proj_p_homo.at<double>(2,0));
            if(imgBoundValid(cur_img, proj_p))
            {
                //            canvas.at<uchar>(proj_p.y, proj_p.x) = 255;
                //            circle(output, proj_p, 2, Scalar(0,0,255));
                if(t==0)
                {
                    output.at<Vec3b>(proj_p.y, proj_p.x) = Vec3b(255,255,255);
                }
                else
                {
                    output.at<Vec3b>(proj_p.y, proj_p.x) = Vec3b(0,255,255);
                }
            }
        }
    }
//    lane_detector->houghDetect(canvas, canvas);
//    imshow("sdasdsada", cur_img_dup);
//    imshow("tetete", canvas);
//    cur_img.setTo(Scalar(0,0,255), canvas);

if(DEBUG)
{
    matcher->showMatches(obj_img, _obj_kp, cur_img, _img_kp, "DEBUG:Current matches");

    vector<Point3f> obj_pts_inlier;
    vector<Point2f> obj_kp_inlier, img_kp_inlier;

    for(int i=0; i<inliers.rows; i++)
    {
        obj_pts_inlier.push_back(obj_pts[inliers.at<int>(0,i)]);
        obj_kp_inlier.push_back(_obj_kp[inliers.at<int>(0,i)]);
        img_kp_inlier.push_back(_img_kp[inliers.at<int>(0,i)]);
        circle(rep_img, img_kp[inliers.at<int>(0,i)], 5, Scalar(255,0,0));
    }

    matcher->showMatches(obj_img, obj_kp_inlier, cur_img, img_kp_inlier, "DEBUG:PnP inlier matches");

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

    return true;
}
