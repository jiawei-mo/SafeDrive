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

bool ThreeDHandler::getPose(const Mat& left_img, const Mat& right_img, Mat& R, Mat& t, vector<Point2f>& left_kp_inliner, vector<Point2f>& right_kp_inliner)
{
    vector<Point2f> left_kp, right_kp;
    matcher->match(left_img, left_kp, right_img, right_kp);

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

        vector<Point2d> _left_marker_detected_polar, _right_marker_detected_polar;
        calibrator->transformPointsToPolar(left_marker_detected_cartesian[t], _left_marker_detected_polar, 1);
        calibrator->transformPointsToPolar(right_marker_detected_cartesian[t], _right_marker_detected_polar, 2);

        Mat right_marker_detected_polar_mat = -Mat::ones(right_rectified.size(), CV_16S);
        Mat left_marker_detected_polar_mat = -Mat::ones(right_rectified.size(), CV_16S);

        vector<Point2d> left_marker_detected_polar, right_marker_detected_polar;
        int counter = 0;
        for(unsigned int c=0; c<_left_marker_detected_polar.size(); c++) {
            if(_left_marker_detected_polar[c].y<ORB_BORDER || _left_marker_detected_polar[c].y+ORB_BORDER>=left_rectified.rows
            || _left_marker_detected_polar[c].x<ORB_BORDER || _left_marker_detected_polar[c].x+ORB_BORDER>=left_rectified.cols) continue;
            left_marker_detected_polar.push_back(_left_marker_detected_polar[c]);
            left_marker_detected_polar_mat.at<short>(_left_marker_detected_polar[c].y, _left_marker_detected_polar[c].x) = counter++;
        }
        counter = 0;
        for(unsigned int c=0; c<_right_marker_detected_polar.size(); c++) {
            if(_right_marker_detected_polar[c].y<32 || _right_marker_detected_polar[c].y+32>=right_rectified.rows
            || _right_marker_detected_polar[c].x<32 || _right_marker_detected_polar[c].x+32>=right_rectified.cols) continue;
            right_marker_detected_polar.push_back(_right_marker_detected_polar[c]);
            right_marker_detected_polar_mat.at<short>(_right_marker_detected_polar[c].y, _right_marker_detected_polar[c].x) = counter++;
        }

        Mat left_desc_mat, right_desc_mat;
        matcher->get_desc(left_rectified, left_marker_detected_polar, left_desc_mat);
        matcher->get_desc(right_rectified, right_marker_detected_polar, right_desc_mat);

        vector<Point2d> left_pts_collector, right_pts_collector;
        for(int theta=0; theta<left_rectified.rows; theta++) {
            vector<Point2d> left_pts, right_pts;
            Mat left_desc = Mat(0, left_desc_mat.cols, left_desc_mat.type());
            Mat right_desc = Mat(0, right_desc_mat.cols, right_desc_mat.type());

            for(int rho_left=0; rho_left<left_rectified.cols; rho_left++) {
                short pts_idx = left_marker_detected_polar_mat.at<short>(theta,rho_left);
                if(pts_idx < 0) continue;
                left_pts.push_back(left_marker_detected_cartesian[t][pts_idx]);
                vconcat(left_desc, left_desc_mat.row(pts_idx), left_desc);
            }
            if(left_pts.empty()) continue;

            for(int theta_offset=-max_theta_offset; theta_offset<=max_theta_offset; theta_offset++) {
                int theta_right = theta+theta_offset;
                if(theta_right<0 || theta_right>=right_marker_detected_polar_mat.rows) continue;
                for(int rho_right=0; rho_right<right_rectified.cols; rho_right++) {
                    short pts_idx = right_marker_detected_polar_mat.at<short>(theta_right,rho_right);
                    if(pts_idx < 0) continue;
                    right_pts.push_back(right_marker_detected_cartesian[t][pts_idx]);
                    vconcat(right_desc, right_desc_mat.row(pts_idx), right_desc);
                }
            }
            if(right_pts.empty()) continue;

            vector<pair<int, int> > matches;
            matcher->match_desc(left_desc, right_desc, matches, 1.0);

            for(size_t i=0; i<matches.size(); i++)
            {
                left_pts_collector.push_back(left_pts[matches[i].first]);
                right_pts_collector.push_back(right_pts[matches[i].second]);
            }
        }

        left_marker_cartesian.push_back(left_pts_collector);
        right_marker_cartesian.push_back(right_pts_collector);
    }

}

bool ThreeDHandler::find3DPoints(const Mat& left_img, const Mat& right_img, vector<Point2f> &features, Mat& additional_desc, vector<Point3f>& feature_pts, vector<vector<Point3f> >& marker_pts)
{
    // find pose between left img and right img
    Mat R, t;
    vector<Point2f> left_kp_inliner, right_kp_inliner;
    if(!getPose(left_img, right_img, R, t, left_kp_inliner, right_kp_inliner)) return false;

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

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    //reconstruct 3d feature points
    int feature_counter = 0;
    double feature_reproj_err = 0.0;
    for(unsigned int i=0; i<left_kp_inliner.size(); i++) {
        float ul = left_kp_inliner[i].x;
        float vl = left_kp_inliner[i].y;
        float ur = right_kp_inliner[i].x;
        float vr = right_kp_inliner[i].y;
        Mat ul_skew = (Mat_<double>(3,3) << 0, -1, vl, 1, 0, -ul, -vl, ul, 0);
        Mat ur_skew = (Mat_<double>(3,3) << 0, -1, vr, 1, 0, -ur, -vr, ur, 0);
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

        if(fabs(x) < 100.0 && fabs(y) < 100.0 && fabs(z) < 100.0)
        {
            pcl::PointXYZRGB p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.b = left_img.at<Vec3b>(vl, ul)[0];
            p.g = left_img.at<Vec3b>(vl, ul)[1];
            p.r = left_img.at<Vec3b>(vl, ul)[2];
            point_cloud->push_back(p);
        }

        point_3d_tmp(0)=x; point_3d_tmp(1)=y; point_3d_tmp(2)=z; point_3d_tmp(3) = 1;
        circle(img_rep, left_kp_inliner[i], 15, Scalar(255,0,0), 4);
        point_2d_tmp = Pl*point_3d_tmp;
        point_2d_tmp /= point_2d_tmp(2);
        double dist = sqrt((ul-point_2d_tmp(0))*(ul-point_2d_tmp(0))+(vl-point_2d_tmp(1))*(vl-point_2d_tmp(1)));
        drawMarker(img_rep, Point2f(point_2d_tmp(0), point_2d_tmp(1)), Scalar(0,0,255), MARKER_STAR, 10, 3);

        circle(img_rep, Point2f(ur+left_img.cols, vr), 15, Scalar(255,0,0), 4);
        point_2d_tmp = Pr*point_3d_tmp;
        point_2d_tmp /= point_2d_tmp(2);
        dist += sqrt((ur-point_2d_tmp(0))*(ur-point_2d_tmp(0))+(vr-point_2d_tmp(1))*(vr-point_2d_tmp(1)));
        drawMarker(img_rep, Point2f(point_2d_tmp(0)+left_img.cols, point_2d_tmp(1)), Scalar(0,0,255), MARKER_STAR, 10, 3);

        feature_reproj_err += dist;
        feature_counter++;
    }

    feature_reproj_err /= (2*feature_counter);

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
    int marker_counter = 0;
    double marker_reproj_err = 0.0;
    vector<Point3f> _marker_pts;
    for(int t=0; t<2; t++)
    {
        _marker_pts.clear();
        for(unsigned int i=0; i<left_marker_matched[t].size(); i++) {
            float ul = left_marker_matched[t][i].x;
            float vl = left_marker_matched[t][i].y;
            float ur = right_marker_matched[t][i].x;
            float vr = right_marker_matched[t][i].y;
            Mat ul_skew = (Mat_<double>(3,3) << 0, -1, vl, 1, 0, -ul, -vl, ul, 0);
            Mat ur_skew = (Mat_<double>(3,3) << 0, -1, vr, 1, 0, -ur, -vr, ur, 0);
            Mat uPl = ul_skew*Pl;
            Mat uPr = ur_skew*Pr;
            Mat A, W, U, V;
            vconcat(uPl, uPr, A);
            SVDecomp(A, W, U, V, SVD::FULL_UV);
            transpose(V,V);

            double x = V.at<double>(0,3) / V.at<double>(3,3);
            double y = V.at<double>(1,3) / V.at<double>(3,3);
            double z = V.at<double>(2,3) / V.at<double>(3,3);
                    if(z<0)     //mark z<0 as purple
                    {
                        circle(img_rep, left_marker_matched[t][i], 3, Scalar(255,0,255));
                        circle(img_rep, Point2f(ur+left_img.cols, vr), 3, Scalar(255,0,255));
                        continue;
                    }

            //remove outlier by reporjection dist
            point_3d_tmp(0)=x; point_3d_tmp(1)=y; point_3d_tmp(2)=z; point_3d_tmp(3) = 1;
            point_2d_tmp = Pl*point_3d_tmp;
            point_2d_tmp /= point_2d_tmp(2);
            double dist = sqrt((ul-point_2d_tmp(0))*(ul-point_2d_tmp(0))
                               +(vl-point_2d_tmp(1))*(vl-point_2d_tmp(1)));

            point_2d_tmp = Pr*point_3d_tmp;
            point_2d_tmp /= point_2d_tmp(2);
            dist += sqrt((ur-point_2d_tmp(0))*(ur-point_2d_tmp(0))
                    +(vr-point_2d_tmp(1))*(vr-point_2d_tmp(1)));
            if(dist > max_lane_reproj_dist)      //mark large dist as yellow
            {

                circle(img_rep, left_marker_matched[t][i], 3, Scalar(0,255,255));
                circle(img_rep, Point2f(ur+left_img.cols, vr), 3, Scalar(0,255,255));
                continue;
            }

            _marker_pts.push_back(Point3f(x,y,z));

            if(fabs(x) < 100.0 && fabs(y) < 100.0 && fabs(z) < 100.0)
            {
                pcl::PointXYZRGB p;
                p.x = x;
                p.y = y;
                p.z = z;
                p.b = left_img.at<Vec3b>(vl, ul)[0];
                p.g = left_img.at<Vec3b>(vl, ul)[1];
                p.r = left_img.at<Vec3b>(vl, ul)[2];
                point_cloud->push_back(p);
            }

            circle(img_rep, left_marker_matched[t][i], 5, Scalar(0,255,0));
            circle(img_rep, Point2f(ur+left_img.cols, vr), 5, Scalar(0,255,0));

            marker_reproj_err += dist;
            marker_counter++;
        }
        marker_pts.push_back(_marker_pts);
    }

    assert(point_cloud->size()>0);
    pcl::io::savePCDFile("test_pcd.pcd", *point_cloud);

    marker_reproj_err /= (2*marker_counter);


if(DEBUG) {
    matcher->showMatches(left_img, left_kp_inliner, right_img, right_kp_inliner, "Feature Match");

    cout<<"Fundamental matrix: "<<endl<<F<<endl;
    cout<<"Rotation: "<<endl<<R<<endl;
    cout<<"Translation: "<<endl<<t<<endl;
    cout<<"Feature reproj ave err: "<<feature_reproj_err<<endl;
    cout<<"Marker reproj ave err: "<<marker_reproj_err<<endl;
    Mat left_lane = left_img.clone();
    cvtColor(left_lane, left_lane, CV_BGR2GRAY);
    cvtColor(left_lane, left_lane, CV_GRAY2BGR);
    left_lane.setTo(Scalar(255,255,255), left_mask[0]);
    left_lane.setTo(Scalar(0,255,255), left_mask[1]);
    Mat right_lane = right_img.clone();
    cvtColor(right_lane, right_lane, CV_BGR2GRAY);
    cvtColor(right_lane, right_lane, CV_GRAY2BGR);
    right_lane.setTo(Scalar(255,255,255), right_mask[0]);
    right_lane.setTo(Scalar(0,255,255), right_mask[1]);
    Mat lane_concat;
    hconcat(left_lane, right_lane, lane_concat);
    namedWindow("Road Marker", WINDOW_NORMAL);
    imshow("Road Marker", lane_concat);


    Mat left_rectified_lane, right_rectified_lane;
    calibrator->getRectifiedImages(left_lane, right_lane, left_rectified_lane, right_rectified_lane);
    cout<<left_rectified_lane.size()<<endl;
    Mat lane_rectified_concat;
    hconcat(left_rectified_lane, right_rectified_lane, lane_rectified_concat);
    for(int j = 0; j < lane_rectified_concat.rows; j += (lane_rectified_concat.rows / 50) ) {
        line(lane_rectified_concat, Point(0, j), Point(lane_rectified_concat.cols, j), Scalar(0, 255, 0), 2);
    }
    namedWindow("Rectified Road Marker", WINDOW_NORMAL);
    imshow("Rectified Road Marker", lane_rectified_concat);

    Mat marker_match_img;
    hconcat(left_img, right_img, marker_match_img);
    for(int t=0; t<2; t++)
    {
        for(unsigned int i=0; i<left_marker_matched[t].size(); i+=20) {
            line(marker_match_img, left_marker_matched[t][i], Point(right_marker_matched[t][i].x+left_img.cols, right_marker_matched[t][i].y), Scalar(0, 255, 0));
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
            drawMarker(img_rep, Point2f(point_2d_tmp(0), point_2d_tmp(1)), Scalar(255,0,255), MARKER_STAR, 5);

            point_2d_tmp = Pr*point_3d_tmp;
            point_2d_tmp /= point_2d_tmp(2);
            drawMarker(img_rep, Point2f(point_2d_tmp(0)+left_img.cols, point_2d_tmp(1)), Scalar(255,0,255), MARKER_STAR, 5);
        }
    }
    namedWindow("Essential reprojection", WINDOW_NORMAL);
    imshow("Essential reprojection", img_rep);
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
    Mat obj_pts_mat = Mat::zeros(3,inliers_features.size(),CV_64F);
    vector<Point3f> obj_pts;
    vector<Point2f> _obj_kp, _img_kp;
    Mat rep_img = cur_img.clone();
    for(unsigned int i=0; i<inliers_features.size(); i++)
    {
        obj_pts.push_back(feature_pts[inliers_features[i]]);
        _obj_kp.push_back(features[inliers_features[i]]);
        _img_kp.push_back(img_kp[i]);

        obj_pts_mat.at<double>(0, i) = feature_pts[inliers_features[i]].x;
        obj_pts_mat.at<double>(1, i) = feature_pts[inliers_features[i]].y;
        obj_pts_mat.at<double>(2, i) = feature_pts[inliers_features[i]].z;
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

    Mat covar, mean;
    calcCovarMatrix(obj_pts_mat, covar, mean, CV_COVAR_NORMAL | CV_COVAR_COLS);
    cout<<"Mean: "<<endl<<mean<<endl;
    cout<<"Covariance: "<<endl<<covar<<endl;

    if(inliers.rows < 3) {
        matcher->showMatches(obj_img, _obj_kp, cur_img, _img_kp, "Fail:Project matches");
        cout<<"Not enough inlier ("<<inliers.rows<<"/"<<inliers_features.size()<<") for PnP, exiting..."<<endl;
        return false;
    }

    cout<<"PnP inliers: "<<inliers.rows<<" / "<<_img_kp.size()<<endl;

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
//                            circle(output, proj_p, 2, Scalar(0,0,255));
                if(t==0)
                {
                    circle(output, proj_p, 2, Scalar(255,255,255), -1);
                }
                else
                {
                    circle(output, proj_p, 2, Scalar(0,255,255), -1);
                }
            }
        }
    }
//    lane_detector->houghDetect(canvas, canvas);
//    imshow("sdasdsada", cur_img_dup);
//    imshow("tetete", canvas);
//    cur_img.setTo(Scalar(0,0,255), canvas);

    imwrite("test.png", output);
if(DEBUG)
{

    cout<<"Rotation:"<<endl<<R<<endl;
    cout<<"Translation:"<<endl<<t<<endl;
//    matcher->showMatches(obj_img, _obj_kp, cur_img, _img_kp, "Current matches");

    vector<Point3f> obj_pts_inlier;
    vector<Point2f> obj_kp_inlier, img_kp_inlier;

    for(int i=0; i<inliers.rows; i++)
    {
        obj_pts_inlier.push_back(obj_pts[inliers.at<int>(0,i)]);
        obj_kp_inlier.push_back(_obj_kp[inliers.at<int>(0,i)]);
        img_kp_inlier.push_back(_img_kp[inliers.at<int>(0,i)]);
        circle(rep_img, img_kp[inliers.at<int>(0,i)], 15, Scalar(255,0,0), 4);
    }

    matcher->showMatches(obj_img, obj_kp_inlier, cur_img, img_kp_inlier, "PnP inlier matches");

//    cout<<"Proj rotation: "<<rvec<<endl;
//    cout<<"Proj translation: "<<t<<endl;
    vector<Point2f> points_2d;
    projectPoints(obj_pts_inlier, rvec, t, K, camera_coeff, points_2d);
    double proj_err = 0.0;
    for(unsigned int i=0; i<points_2d.size(); i++) {
        drawMarker(rep_img, points_2d[i], Scalar(0,0,255), MARKER_STAR, 10, 3);
        double dist = sqrt((img_kp_inlier[i].x-points_2d[i].x)*(img_kp_inlier[i].x-points_2d[i].x)+(img_kp_inlier[i].y-points_2d[i].y)*(img_kp_inlier[i].y-points_2d[i].y));
        proj_err += dist;
    }
    proj_err /= points_2d.size();
    cout<<"PnP projection err "<<proj_err<<endl;
    namedWindow("PnP projection", WINDOW_NORMAL);
    imshow("PnP projection", rep_img);
}

    return true;
}
