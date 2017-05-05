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
    static pcl::visualization::CloudViewer viewer("DEBUG:Cloud Viewer");
    viewer.showCloud(point_cloud);
    while( !viewer.wasStopped() )
    {
        sleep(1);
    }
}

void ThreeDHandler::findDisparity(Mat &disp_img, Mat &Q, Mat& left_img, Mat& right_img)
{
    vector<Point2f> left_kp, right_kp;
    matcher->match(left_img, left_kp, right_img, right_kp, 40);

    //find essential_mat based on matches using RANSAC
    Mat inliner_mask, essential_mat, R, t;
    essential_mat  = findEssentialMat(left_kp, right_kp, camera_K, RANSAC, 0.999, ransac_thres_feature, inliner_mask);
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

        essential_mat = findEssentialMat(left_kp, right_kp, camera_K, RANSAC, 0.999, ransac_thres_feature, inliner_mask);
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

    Mat left_rect_map_x, left_rect_map_y, right_rect_map_x, right_rect_map_y,left_rect,right_rect;
    initUndistortRectifyMap(camera_K, camera_coeff, R1, P1, frame_size,CV_32F, left_rect_map_x, left_rect_map_y);
    initUndistortRectifyMap(camera_K, camera_coeff, R2, P2, frame_size, CV_32F, right_rect_map_x, right_rect_map_y);
    remap(left_img, left_rect, left_rect_map_x, left_rect_map_y, INTER_LINEAR);
    remap(right_img, right_rect, right_rect_map_x, right_rect_map_y, INTER_LINEAR);

    Ptr<StereoSGBM> sbm = StereoSGBM::create( minDisparity, numberOfDisparities, SADWindowSize, SP1, SP2, disp12MaxDiff, preFilterCap, uniquenessRatio, speckleWindowSize, speckleRange);
    Mat imgDisparity;

    sbm->compute( left_rect, right_rect, imgDisparity );

    Mat left_mask, right_mask, left_lane_img, right_lane_img;
    lane_detector->detect(left_rect, left_mask);
    left_rect.copyTo(left_lane_img, left_mask);
    lane_detector->detect(right_rect, right_mask);
    right_rect.copyTo(right_lane_img, right_mask);

    //force marker pixels to have disparity
//    Mat lane_disp = Mat::zeros(frame_size, CV_16S);
    int batch_size = 30;
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
//            if( imgDisparity.at<short>(y, x)>0) continue;
            int d = (x-min_pos)*16;
            imgDisparity.at<short>(y, x) = d;
        }
    }

//    medianBlur(lane_disp, lane_disp, 5);
//    imgDisparity += lane_disp;

    normalize(imgDisparity, disp_img, 0, 255, CV_MINMAX, CV_8U);

    left_img = left_rect;
    right_img = right_rect;

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
                Vec3b &color_vec = left_rect.at<Vec3b>(j, i);
                p.r = color_vec[2];
                p.g = color_vec[1];
                p.b = color_vec[0];
                point_cloud->push_back(p);
            }
        }
    }

    showPoints = boost::thread(showPC, point_cloud);

    Mat epi;
    hconcat(left_rect, right_rect, epi);
    for(int j = 0; j < epi.rows; j += (epi.rows / 15) )
        line(epi, Point(0, j), Point(epi.cols, j), Scalar(0, 255, 0), 1, 8);
    namedWindow("DEBUG:Epipolar line", WINDOW_NORMAL);
    imshow("DEBUG:Epipolar line", epi);
    namedWindow("DEBUG:Disparity", WINDOW_NORMAL);
    imshow("DEBUG:Disparity", disp_img);
#endif
}

void ThreeDHandler::project(const Mat& obj_img, Mat& cur_img, const Mat& disp_img, const Mat &Q)
{
    vector<Point2f> p_obj, p_cur;
    matcher->match(obj_img, p_obj, cur_img, p_cur, 40);

    //register camera frame
    Mat Qf;
    Q.convertTo(Qf, CV_32F);
    vector<Point2f> p_img_d;
    vector<Point2f> p_obj_d;
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
        p_img_d.push_back(p_cur[i]);
        p_obj_d.push_back(p_obj[i]);
    }

    cv::Mat rvec, t, inliers;
    cv::solvePnPRansac( pts_obj, p_img_d, camera_K, cv::Mat(), rvec, t, false, 100, 8.0, 0.99, inliers, cv::SOLVEPNP_ITERATIVE );

    vector<KeyPoint> left_kp, right_kp;
    cout<<inliers.type()<<" "<<inliers.size()<<endl;
    for(size_t i=0; i<inliers.rows; i++)
    {
        left_kp.push_back(KeyPoint(p_obj_d[inliers.at<int>(0,i)], 0));
        right_kp.push_back(KeyPoint(p_img_d[inliers.at<int>(0,i)], 0));
    }

    Mat corres_img;
    corres_img = Mat::zeros(obj_img.rows, 2*obj_img.cols, obj_img.type());
    Mat left_features = obj_img.clone();
    Mat right_features = cur_img.clone();
    drawKeypoints(left_features, left_kp, left_features);
    drawKeypoints(right_features, right_kp, right_features);
    left_features.copyTo(corres_img(Rect(0, 0, obj_img.cols, obj_img.rows)));
    right_features.copyTo(corres_img(Rect(obj_img.cols, 0, obj_img.cols, obj_img.rows)));
//    show matches between two images side by side
            for(int i=0; i<(int)right_kp.size(); i++)
    {
        line(corres_img, left_kp[i].pt, Point2f(right_kp[i].pt.x+obj_img.cols, right_kp[i].pt.y), CV_RGB(255, 0, 0));
    }

    string windowName = "DEBUG: feature matching";
    namedWindow(windowName, WINDOW_NORMAL);
    imshow(windowName, corres_img);
    waitKey(1);

    cout<<"Proj rotation: "<<rvec<<endl;
    cout<<"Proj translation: "<<t<<endl;

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
                canvas.at<Vec3b>(proj_p.y, proj_p.x) += (obj_img.at<Vec3b>(y, x) / 2);
            }
        }
    }
    GaussianBlur(canvas, canvas, Size(3,3), 3);

    cur_img += canvas;
}
