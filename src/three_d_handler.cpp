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

    showPoints = boost::thread(showPC, point_cloud);

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

    return;
}

void ThreeDHandler::project(const Mat& obj_img)
{
    vector<Point2f> p_obj;
    matcher->match(obj_img, p_obj, left_img, left_kp, true);

    //register camera frame
    vector<Point3f> pts_obj;
    for(unsigned int i=0; i<left_kp.size(); i++)
    {
        cout<<three_d_pts_map[left_kp[i]]<<endl;
        pts_obj.push_back(three_d_pts_map[left_kp[i]]);
    }

    cv::Mat rvec, t, inliers;
    cv::solvePnPRansac( pts_obj, left_kp, camera_K, cv::Mat(), rvec, t, false, 100, 8.0, 0.99, inliers, cv::SOLVEPNP_ITERATIVE );

    cout<<"Proj rotation: "<<rvec<<endl;
    cout<<"Proj translation: "<<t<<endl;

    Mat R, P;
    Rodrigues(rvec, R);
    hconcat(R, t, P);

//    Mat lane_mask;
//    lane_detector->detect(obj_img, lane_mask);
//    //project road marker
//    cv::Mat_<double> p_homo(4,1);
//    for(int x=0; x<obj_img.cols; x++) {
//        for(int y=0; y<obj_img.rows; y++) {
//            if(lane_mask.at<uchar>(y,x) == 0) continue;
//            int d = disp_img.at<uchar>(y,x);
//            if(d == 0)
//            {
//                continue;
//            }

//            p_homo(0) = x; p_homo(1) = y; p_homo(2) = d; p_homo(3) = 1;
//            p_homo = Q*p_homo;
//            p_homo /= p_homo(3);
//            Mat proj_p_homo = camera_K*P*p_homo;
//            Point2i proj_p(proj_p_homo.at<double>(0,0)/proj_p_homo.at<double>(2,0), proj_p_homo.at<double>(1,0)/proj_p_homo.at<double>(2,0));
//            if(imgBoundValid(cur_img, proj_p))
//            {
//                cur_img.at<Vec3b>(proj_p.y, proj_p.x) /= 2;
//                cur_img.at<Vec3b>(proj_p.y, proj_p.x) += (obj_img.at<Vec3b>(y, x) / 2);
//            }
//        }
//    }


}
