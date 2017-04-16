#include "headers/three_d_handler.hpp"

bool imgBoundValid(const Mat& img, Point2i pt) {
    bool a = pt.x >= 0;
    bool b = pt.x < img.cols;
    bool c = pt.y >=0;
    bool d = pt.y < img.rows;
    return a && b && c && d;
}

ThreeDHandler::ThreeDHandler()
{
    matcher = shared_ptr<Matcher>(new Matcher());
    lane_detector = shared_ptr<LaneDetector>(new LaneDetector());
    ransac_thres_feature = RTF / 1.0f;
    SADWindowSize = SWS;
    numberOfDisparities = ND * 16;
    preFilterCap = PFC;
    minDisparity = MOD;
    uniquenessRatio = UR;
    speckleWindowSize = SW;
    speckleRange = SR;
    disp12MaxDiff = DMD;
    SP1 = S1;
    SP2 = S2;

    camera_K = (cv::Mat_<double>(3,3) << 1130.2, 0, 677.1,
                                     0, 1129.9, 507.5,
                                     0, 0, 1);

    camera_coeff = (cv::Mat_<double>(1,5) << 0.0995, -0.2875, 0, 0, 0);

//    camera_K = (cv::Mat_<double>(3,3) << 1623.4, 0, 1081.9,
//                                     0, 1623.7, 709.9,
//                                     0, 0, 1);

//    camera_coeff = (cv::Mat_<double>(1,5) << -0.2004, 0.1620, 0, 0, 0);
}

ThreeDHandler::~ThreeDHandler()
{
        showPoints.join();
}

void ThreeDHandler::changeParam(float rtf, int sws, int nd, int pfc, int mod, int ur, int sw, int sr, int dmd, int s1, int s2)
{
    ransac_thres_feature = rtf;
    SADWindowSize = sws;
    numberOfDisparities = nd;
    preFilterCap = pfc;
    minDisparity = mod;
    uniquenessRatio = ur;
    speckleWindowSize = sw;
    speckleRange = sr;
    disp12MaxDiff = dmd;
    SP1 = s1;
    SP2 = s2;
}

void showPC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud)
{
    static pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(point_cloud);
    while( !viewer.wasStopped() )
    {
        sleep(1);
    }
}

void ThreeDHandler::find3DPoints(const Mat& _left_img, const Mat& _right_img)
{
    left_img = _left_img.clone();
    right_img = _right_img.clone();
    vector<Point2f> _left_kp, _right_kp;
    matcher->match(left_img, _left_kp, right_img, _right_kp, false);

    //find essential_mat based on matches using RANSAC
    Mat inliner_mask, essential_mat, R, t;
    essential_mat  = findEssentialMat(_left_kp, _right_kp, camera_K, RANSAC, 0.99, ransac_thres_feature, inliner_mask);
    recoverPose(essential_mat, _left_kp, _right_kp, camera_K, R, t, inliner_mask);

    if(t.at<double>(0,0) > 0)
    {
        Mat tmp_mat;
        tmp_mat = left_img;
        left_img = right_img;
        right_img = tmp_mat;

        vector<Point2f> tmp_vec;
        tmp_vec = _left_kp;
        _left_kp = _right_kp;
        _right_kp = tmp_vec;

        essential_mat = findEssentialMat(_left_kp, _right_kp, camera_K, RANSAC, 0.99, ransac_thres_feature, inliner_mask);
        recoverPose(essential_mat, _left_kp, _right_kp, camera_K, R, t, inliner_mask);
    }

    for(size_t i=0; i<_left_kp.size(); i++)
    {
        if(inliner_mask.at<uchar>(1,i))
        {
            left_kp.push_back(_left_kp[i]);
            right_kp.push_back(_right_kp[i]);
        }
    }

    Mat P1 = (cv::Mat_<double>(3,4) << 1, 0, 0, 0,
                                       0, 1, 0, 0,
                                       0, 0, 1, 0);
    P1 = camera_K*P1;

    Mat P2;
    hconcat(R, t, P2);
    P2 = camera_K*P2;

    Mat three_d_pts;
    triangulatePoints(P1, P2, left_kp, right_kp, three_d_pts);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    for(size_t i=0; i<left_kp.size(); i++)
    {
        Point3f p_3d(three_d_pts.at<float>(0, i)/three_d_pts.at<float>(3,i),
                     three_d_pts.at<float>(1, i)/three_d_pts.at<float>(3,i),
                     three_d_pts.at<float>(2, i)/three_d_pts.at<float>(3,i));
        three_d_pts_map[left_kp[i]] = p_3d;

        pcl::PointXYZRGB p;
        p.x = p_3d.x;
        p.y = p_3d.y;
        p.z = p_3d.z;
        Vec3b &color_vec = left_img.at<Vec3b>(left_kp[i].y, left_kp[i].x);
        p.r = color_vec[2];
        p.g = color_vec[1];
        p.b = color_vec[0];
        point_cloud->push_back(p);
    }

    Mat rot_vec;
    Rodrigues(R, rot_vec);

    cout<<"R= "<<rot_vec<<endl;
    cout<<"t= "<<t<<endl;

    Size frame_size = left_img.size();
    Mat R1, R2, Q;
    stereoRectify(camera_K, camera_coeff, camera_K, camera_coeff,frame_size, R, t, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 0, frame_size, 0, 0);

    Mat left_rect_map_x, left_rect_map_y, right_rect_map_x, right_rect_map_y,left_rect,right_rect;
    initUndistortRectifyMap(camera_K, camera_coeff, R1, P1, frame_size,CV_32F, left_rect_map_x, left_rect_map_y);
    initUndistortRectifyMap(camera_K, camera_coeff, R2, P2, frame_size, CV_32F, right_rect_map_x, right_rect_map_y);
    remap(left_img, left_rect, left_rect_map_x, left_rect_map_y, INTER_LINEAR);
    remap(right_img, right_rect, right_rect_map_x, right_rect_map_y, INTER_LINEAR);

    Mat left_mask, right_mask, left_lane_img, right_lane_img;
    lane_detector->detect(left_rect, left_mask);
    left_rect.copyTo(left_lane_img, left_mask);
    lane_detector->detect(right_rect, right_mask);
    right_rect.copyTo(right_lane_img, right_mask);

    int batch_size = 355;
//    Mat lane_disp = Mat::zeros(frame_size, CV_6);
    Mat_<double> lane_p(4,1);
    for(int x=batch_size; x<left_rect.cols-batch_size; x++){
        for(int y=0; y<left_rect.rows; y++){
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
            if(min_dist>500) continue;
            int d = x-min_pos;
            lane_p(0) = x; lane_p(1) = y; lane_p(2) = d; lane_p(3) = 1;
            lane_p = Q*lane_p;
            lane_p /= lane_p(3);
            if(lane_p(2)>10) continue;
            Point3f p_3d(lane_p(0), lane_p(1), lane_p(2));
            lane_pts.push_back(p_3d);

            pcl::PointXYZRGB p;
            p.x = p_3d.x;
            p.y = p_3d.y;
            p.z = p_3d.z;
            Vec3b &color_vec = left_rect.at<Vec3b>(y, x);
            p.r = color_vec[2];
            p.g = color_vec[1];
            p.b = color_vec[0];
            point_cloud->push_back(p);
        }
    }

    showPoints = boost::thread(showPC, point_cloud);
    return;
}

void ThreeDHandler::project(Mat& target_img)
{
    vector<Point2f> _target_kp, _left_kp;
    matcher->match(target_img, _target_kp, left_img, _left_kp, true);

    //register camera frame
    vector<Point3f> left_3d;
    vector<Point2f> target_kp;
    for(unsigned int i=0; i<_left_kp.size(); i++)
    {
        if(three_d_pts_map.find(_left_kp[i]) == three_d_pts_map.end()) continue;
        left_3d.push_back(three_d_pts_map[_left_kp[i]]);
        target_kp.push_back(_target_kp[i]);
    }

    cout<<left_3d.size()<<endl;
    cv::Mat rvec, t, inliers;
    cv::solvePnPRansac(left_3d, target_kp, camera_K, cv::Mat(), rvec, t, false, 100, 8.0, 0.99, inliers, cv::SOLVEPNP_ITERATIVE );

    cout<<"Proj rotation: "<<rvec<<endl;
    cout<<"Proj translation: "<<t<<endl;

    Mat R, P;
    Rodrigues(rvec, R);
    hconcat(R, t, P);

    Mat_<double> proj_p(4,1);
    Mat_<double> px(3,1);
    for(size_t i=0; i<lane_pts.size(); i++)
    {
        proj_p(0) = lane_pts[i].x; proj_p(1) = lane_pts[i].y; proj_p(2) = lane_pts[i].z; proj_p(3) = 1;
        px = camera_K*P*proj_p;
        Point2i proj_p(px(0)/px(2), px(1)/px(2));
        if(imgBoundValid(target_img, proj_p))
        {
            target_img.at<Vec3b>(proj_p.y, proj_p.x) = Vec3b(255,255,255);
        }
    }
}
