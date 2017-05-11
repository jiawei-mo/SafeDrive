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
    lane_detector = shared_ptr<LaneDetector>(new LaneDetector());
    ransac_thres_essential = RTE / 1.0f;
    ransac_thres_pnp = RTP / 1.0f;

    camera_K = (cv::Mat_<double>(3,3) << 1130.2, 0, 677.1,
                                     0, 1129.9, 507.5,
                                     0, 0, 1);

    camera_coeff = (cv::Mat_<double>(1,5) << 0.0995, -0.2875, 0, 0, 0);
}
void ThreeDHandler::changeParam(const shared_ptr<Matcher> _matcher, float rte, float rtp)
{
    matcher = _matcher;
    ransac_thres_essential = rte;
    ransac_thres_pnp = rtp;
}

void ThreeDHandler::findDisparity(vector<KeyPoint> &feature_disp, Mat& marker_disp, Mat &Q, Mat& left_img, Mat& right_img)
{
    vector<Point2f> left_kp, right_kp;
    matcher->match(left_img, left_kp, right_img, right_kp);

    //find essential_mat based on matches using RANSAC
    Mat inliner_mask, essential_mat, R, t;
    essential_mat  = findEssentialMat(left_kp, right_kp, camera_K, RANSAC, 0.999, ransac_thres_essential, inliner_mask);
    recoverPose(essential_mat, left_kp, right_kp, camera_K, R, t, inliner_mask);

    if(t.at<double>(0,0) > 0)
    {
        R = R.t();
        t = -t;

        Mat tmp_mat;
        tmp_mat = left_img;
        left_img = right_img;
        right_img = tmp_mat;

        vector<Point2f> tmp_vec;
        tmp_vec = left_kp;
        left_kp = right_kp;
        right_kp = tmp_vec;

        essential_mat = findEssentialMat(left_kp, right_kp, camera_K, RANSAC, 0.999, ransac_thres_essential, inliner_mask);
        recoverPose(essential_mat, left_kp, right_kp, camera_K, R, t, inliner_mask);
    }

    Mat rot_vec;
    Rodrigues(R, rot_vec);

#ifdef QT_DEBUG
    cout<<"R= "<<rot_vec<<endl;
    cout<<"t= "<<t<<endl;
#endif

    Size frame_size = left_img.size();
    Mat R1, R2, P1, P2;
    stereoRectify(camera_K, camera_coeff, camera_K, camera_coeff,frame_size, R, t, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 0, frame_size, 0, 0);


    Mat left_rect_map_x, left_rect_map_y, right_rect_map_x, right_rect_map_y;
    initUndistortRectifyMap(camera_K, camera_coeff, R1, P1, frame_size,CV_32F, left_rect_map_x, left_rect_map_y);
    initUndistortRectifyMap(camera_K, camera_coeff, R2, P2, frame_size, CV_32F, right_rect_map_x, right_rect_map_y);
    remap(left_img, left_img, left_rect_map_x, left_rect_map_y, INTER_LINEAR);
    remap(right_img, right_img, right_rect_map_x, right_rect_map_y, INTER_LINEAR);

    matcher->rectified_match(left_img, left_kp, right_img, right_kp);

#ifdef QT_DEBUG
    matcher->showMatches(left_img, left_kp, right_img, right_kp, "DEBUG: undistorted matches");
#endif

    for(unsigned int i=0; i<left_kp.size(); i++) {
        float disp = left_kp[i].x  - right_kp[i].x;
        feature_disp.push_back(KeyPoint(left_kp[i], disp));
    }

    Mat left_mask, right_mask, left_lane_img, right_lane_img;
    lane_detector->detect(left_img, left_mask);
    left_img.copyTo(left_lane_img, left_mask);
    lane_detector->detect(right_img, right_mask);
    right_img.copyTo(right_lane_img, right_mask);

    //force marker pixels to have disparity
    marker_disp = Mat::zeros(frame_size, CV_32F);
    int batch_size = 30;
    for(int x=batch_size; x<left_img.cols-batch_size; x++){
        for(int y=0; y<left_img.rows; y++){
            if(left_mask.at<uchar>(y,x) == 0) continue;

            Mat left_batch = left_lane_img(Rect(x-batch_size, y, batch_size*2, 1));
            double min_dist = -1;
            int min_pos = x-1;
            for(int right_x=x; right_x>batch_size; right_x--){
                if(right_mask.at<uchar>(y,right_x) == 0) continue;
                Mat right_batch = right_lane_img(Rect(right_x-batch_size, y, batch_size*2, 1));
                Mat diff = right_batch - left_batch;
                double dist = norm(diff);
                if(min_dist < 0 || min_dist > dist){
                    min_dist = dist;
                    min_pos = right_x;
                }
            }
            float d = float(x-min_pos);
            marker_disp.at<float>(y, x) = d;
        }
    }

#ifdef QT_DEBUG
    Ptr<StereoSGBM> sbm = StereoSGBM::create( MOD, ND, SWS, S1, S2, DMD, PFC, UR, SW, SR);
    Mat imgDisparity;

    sbm->compute( left_img, right_img, imgDisparity );
    Mat disp_img;
    normalize(imgDisparity, disp_img, 0, 255, CV_MINMAX, CV_8U);

//    namedWindow("DEBUG:Dense Disparity", WINDOW_NORMAL);
//    imshow("DEBUG:Dense Disparity", disp_img);

    cv::Mat XYZ(disp_img.size(),CV_32FC3);
    reprojectImageTo3D(disp_img, XYZ, Q, false, CV_32F);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i=0; i<XYZ.cols; i++)
    {
        for(int j=0; j<XYZ.rows; j++)
        {
            Vec3f &pos_vec = XYZ.at<Vec3f>(j, i);
            if((pos_vec[2] > 0.0 && pos_vec[2] < 200.0) || (pos_vec[2] > -200.0 && pos_vec[2] < -0.0))
            {
                pcl::PointXYZRGB p;
                p.x = pos_vec[0];
                p.y = pos_vec[1];
                p.z = pos_vec[2];
                Vec3b &color_vec = left_img.at<Vec3b>(j, i);
                p.r = color_vec[2];
                p.g = color_vec[1];
                p.b = color_vec[0];
                point_cloud->push_back(p);
            }
        }
    }

    assert(point_cloud->size()>0);
    pcl::io::savePCDFile("test_pcd.pcd", *point_cloud);

#endif
}

void ThreeDHandler::project(const Mat& obj_img, Mat& cur_img, vector<KeyPoint> &feature_disp, const Mat& marker_disp, const Mat &Q)
{
    vector<Point2f> img_kp;
    matcher->match_given_kp(obj_img, feature_disp, cur_img, img_kp);

    //register camera frame
    Mat Qf;
    Q.convertTo(Qf, CV_32F);
    vector<Point3f> obj_pts;
    vector<Point2f> _obj_kp;
    vector<Point2f> _img_kp;
    cv::Mat_<float> vec_tmp(4,1);
    for(unsigned int i=0; i<feature_disp.size(); i++)
    {
        float x = feature_disp[i].pt.x;
        float y = feature_disp[i].pt.y;
        float d = feature_disp[i].size;
        vec_tmp(0) = x; vec_tmp(1) = y; vec_tmp(2) = d; vec_tmp(3) = 1;
        vec_tmp = Qf*vec_tmp;
        vec_tmp /= vec_tmp(3);

        obj_pts.push_back(Point3f(vec_tmp(0), vec_tmp(1), vec_tmp(2)));
        _obj_kp.push_back(feature_disp[i].pt);
        _img_kp.push_back(img_kp[i]);
    }

    cv::Mat rvec, t, inliers;
    cv::solvePnPRansac( obj_pts, _img_kp, camera_K, cv::Mat(), rvec, t, false, 1000, ransac_thres_pnp, 0.99, inliers, cv::SOLVEPNP_ITERATIVE );
#ifdef QT_DEBUG
    cout<<"PnP inliers: "<<inliers.rows<<" / "<<_img_kp.size()<<endl;
    matcher->showMatches(obj_img, _obj_kp, cur_img, _img_kp, "DEBUG:Project matches", inliers);

    cout<<"Proj rotation: "<<rvec<<endl;
    cout<<"Proj translation: "<<t<<endl;
#endif

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
