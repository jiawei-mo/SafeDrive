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

void ThreeDHandler::findDisparity(Mat &disp_img, Mat &Q, Mat& left_img, vector<Point2f>& left_kp, Mat& right_img, vector<Point2f>& right_kp)
{
    //find essential_mat based on matches using RANSAC
    Mat inliner_mask, essential_mat, R, t;
    essential_mat  = findEssentialMat(left_kp, right_kp, camera_K, RANSAC, 0.99, ransac_thres_feature, inliner_mask);
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

        essential_mat = findEssentialMat(left_kp, right_kp, camera_K, RANSAC, 0.99, ransac_thres_feature, inliner_mask);
        recoverPose(essential_mat, left_kp, right_kp, camera_K, R, t, inliner_mask);
    }

    Mat rot_vec;
    Rodrigues(R, rot_vec);

    cout<<"R= "<<rot_vec<<endl;
    cout<<"t= "<<t<<endl;

    Size frame_size = left_img.size();
    Mat R1, R2, P1, P2;
    stereoRectify(camera_K, camera_coeff, camera_K, camera_coeff,frame_size, R, t, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 0, frame_size, 0, 0);
//    cout<<R1<<endl<<R2<<endl<<P1<<endl<<P2<<endl<<Q<<endl;

    Mat left_undist_rect_map_x, left_undist_rect_map_y, right_undist_rect_map_x, right_undist_rect_map_y,left_undist_rect,right_undist_rect;
    initUndistortRectifyMap(camera_K, camera_coeff, R1, P1, frame_size,CV_32F, left_undist_rect_map_x, left_undist_rect_map_y);
    initUndistortRectifyMap(camera_K, camera_coeff, R2, P2, frame_size, CV_32F, right_undist_rect_map_x, right_undist_rect_map_y);
    remap(left_img, left_undist_rect, left_undist_rect_map_x, left_undist_rect_map_y, INTER_LINEAR);
    remap(right_img, right_undist_rect, right_undist_rect_map_x, right_undist_rect_map_y, INTER_LINEAR);

    Ptr<StereoSGBM> sbm = StereoSGBM::create( minDisparity, numberOfDisparities, SADWindowSize, SP1, SP2, disp12MaxDiff, preFilterCap, uniquenessRatio, speckleWindowSize, speckleRange);
    Mat imgDisparity;

    sbm->compute( left_undist_rect, right_undist_rect, imgDisparity );


    Mat whiteHist, yellowHist;
    Mat empty_top = Mat::zeros(frame_size.height/2, frame_size.width, CV_8U);
    Mat left_roi = left_undist_rect(Rect(0,frame_size.height/2,frame_size.width,frame_size.height/2));
    inRange(left_roi, Scalar(200, 200, 200), Scalar(255, 255, 255), whiteHist);
    inRange(left_roi, Scalar(0, 150, 150), Scalar(180, 255, 255), yellowHist);
    Mat left_yelloAndWhite = yellowHist + whiteHist;
    Mat right_roi = right_undist_rect(Rect(0,frame_size.height/2,frame_size.width,frame_size.height/2));
    inRange(right_roi, Scalar(200, 200, 200), Scalar(255, 255, 255), whiteHist);
    inRange(right_roi, Scalar(0, 150, 150), Scalar(180, 255, 255), yellowHist);
    Mat right_yelloAndWhite = yellowHist + whiteHist;

    Mat left_lane_img, right_lane_img;
    vconcat(empty_top, left_yelloAndWhite, left_lane_img);
    left_undist_rect.copyTo(left_lane_img, left_lane_img);
    vconcat(empty_top, right_yelloAndWhite, right_lane_img);
    right_undist_rect.copyTo(right_lane_img, right_lane_img);
    namedWindow("left", WINDOW_NORMAL);
    imshow("left", left_lane_img);
    namedWindow("right", WINDOW_NORMAL);
    imshow("right", right_lane_img);

    //    force marker pixels to have disparity
//    Mat lane_disp;
//    sbm->compute( left_lane_img, right_lane_img, lane_disp );

    Mat lane_disp = Mat::zeros(frame_size, CV_16S);
    int batch_size = 355;
    lane_detector->detect(left_undist_rect, marker_pixels);
    for(auto& mp: marker_pixels){
        if(mp.x-batch_size<0 || mp.x+batch_size>=frame_size.width){
            continue;
        }

        Mat mp_batch = left_lane_img(Rect(mp.x-batch_size, mp.y, batch_size*2, 1));
        double min_dist = -1;
        int min_pos = mp.x-1;
        for(int cor_x=mp.x; cor_x>batch_size; cor_x--){
            Mat cor_batch = right_lane_img(Rect(cor_x-batch_size, mp.y, batch_size*2, 1));
            Mat diff = cor_batch - mp_batch;
            double dist = norm(diff);
            if(min_dist < 0 || min_dist > dist){
                min_dist = dist;
                min_pos = cor_x;
            }
        }
        if( imgDisparity.at<short>(mp.y, mp.x)>0) continue;
        lane_disp.at<short>(mp.y, mp.x) = (mp.x-min_pos)*16;
    }
    imgDisparity += lane_disp;

    normalize(imgDisparity, disp_img, 0, 255, CV_MINMAX, CV_8U);

#ifdef QT_DEBUG
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
                Vec3b &color_vec = left_undist_rect.at<Vec3b>(j, i);
                p.r = color_vec[2];
                p.g = color_vec[1];
                p.b = color_vec[0];
                point_cloud->push_back(p);
            }
        }
    }

    showPoints = boost::thread(showPC, point_cloud);

    Mat epi;
    hconcat(left_undist_rect, right_undist_rect, epi);
    for(int j = 0; j < epi.rows; j += (epi.rows / 15) )
        line(epi, Point(0, j), Point(epi.cols, j), Scalar(0, 255, 0), 1, 8);
    namedWindow("Epipolar line", WINDOW_NORMAL);
    imshow("Epipolar line", epi);
    namedWindow("Disparity", WINDOW_NORMAL);
    imshow("Disparity", disp_img);
#endif
}

void ThreeDHandler::project(Mat& cur_img, const vector<Point2f>& p_cur, const Mat& disp_img, const Mat& obj_img, const vector<Point2f>& p_obj, const Mat &Q, const vector<Point2f>& p_marker)
{

    //register camera frame
    Mat Qf;
    Q.convertTo(Qf, CV_32F);
    vector<Point2f> pts_img;
    vector<Point3f> pts_obj;
    cv::Mat_<float> vec_tmp(4,1);
    for(unsigned int i=0; i<p_obj.size(); i++)
    {
        float x = p_obj[i].x;
        float y = p_obj[i].y;
        int d = disp_img.at<uchar>(y,x);
        if(d == 0)
        {
            continue;
        }

        vec_tmp(0) = x; vec_tmp(1) = y; vec_tmp(2) = d; vec_tmp(3) = 1;
        vec_tmp = Qf*vec_tmp;
        vec_tmp /= vec_tmp(3);
        pts_obj.push_back(Point3f(vec_tmp(0), vec_tmp(1), vec_tmp(2)));
        pts_img.push_back(p_cur[i]);
    }

    cv::Mat rvec, t, inliers;
    cv::solvePnPRansac( pts_obj, pts_img, camera_K, cv::Mat(), rvec, t, false, 100, 8.0, 0.99, inliers, cv::SOLVEPNP_ITERATIVE );

    cout<<"Proj rotation: "<<rvec<<endl;
    cout<<"Proj translation: "<<t<<endl;

    Mat R, P;
    Rodrigues(rvec, R);
    hconcat(R, t, P);

    //project road marker
    cv::Mat_<double> p_homo(4,1);
    for(unsigned int i=0; i<p_marker.size(); i++)
    {
        float x = p_marker[i].x;
        float y = p_marker[i].y;
        int d = disp_img.at<uchar>(y,x);
        if(d == 0)
        {
            continue;
        }

        p_homo(0) = x; p_homo(1) = y; p_homo(2) = d; p_homo(3) = 1;
        p_homo = Q*p_homo;
        p_homo /= p_homo(3);
        Mat proj_p_homo = camera_K*P*p_homo;
        Point2i proj_p(proj_p_homo.at<double>(0,0)/proj_p_homo.at<double>(2,0), proj_p_homo.at<double>(1,0)/proj_p_homo.at<double>(2,0));
        if(imgBoundValid(cur_img, proj_p))
        {
            cur_img.at<Vec3b>(proj_p.y, proj_p.x) /= 2;
            cur_img.at<Vec3b>(proj_p.y, proj_p.x) += (obj_img.at<Vec3b>(y, x) / 2);
        }
    }


}
