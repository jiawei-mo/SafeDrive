#include "reconstructor.hpp"

Reconstructor::Reconstructor()
{
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
}

void Reconstructor::changeParam(float rtf, int sws, int nd, int pfc, int mod, int ur, int sw, int sr, int dmd, int s1, int s2)
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

void Reconstructor::reconstruct(const Mat& left_img, const Mat& left_kp, const Mat& right_img, const Mat& right_kp, const Mat& cam_K, const Mat& cam_coeff)
{
    //find essential_mat based on matches using RANSAC
    Mat inliner_mask;
    Mat essential_mat = findEssentialMat(right_kp, left_kp, cam_K, RANSAC, 0.99, ransac_thres_feature, inliner_mask);
    Mat R,t;
    recoverPose(essential_mat, right_kp, left_kp, cam_K, R, t, inliner_mask);

    Mat rot_vec;
    Rodrigues(R, rot_vec);

    cout<<"R= "<<rot_vec<<endl;
    cout<<"t= "<<t<<endl;

    Size frame_size = left_img.size();
    Mat R1, R2, P1, P2, Q;
    stereoRectify(cam_K, cam_coeff, cam_K, cam_coeff,frame_size, R, t, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 0, frame_size, 0, 0);
//    cout<<R1<<endl<<R2<<endl<<P1<<endl<<P2<<endl<<Q<<endl;


    Mat left_undist_rect_map_x, left_undist_rect_map_y, right_undist_rect_map_x, right_undist_rect_map_y,left_undist_rect,right_undist_rect;
    initUndistortRectifyMap(cam_K, cam_coeff, R1, P1, frame_size,CV_16SC2, left_undist_rect_map_x, left_undist_rect_map_y);
    initUndistortRectifyMap(cam_K, cam_coeff, R2, P2, frame_size, CV_16SC2, right_undist_rect_map_x, right_undist_rect_map_y);
    remap(right_img, left_undist_rect, left_undist_rect_map_x, left_undist_rect_map_y, INTER_LINEAR);
    remap(left_img, right_undist_rect, right_undist_rect_map_x, right_undist_rect_map_y, INTER_LINEAR);

    Ptr<StereoSGBM> sbm = StereoSGBM::create( minDisparity, numberOfDisparities, SADWindowSize, SP1, SP2, disp12MaxDiff, preFilterCap, uniquenessRatio, speckleWindowSize, speckleRange);
    Mat imgDisparity, imgDisparity8U;

    sbm->compute( left_undist_rect, right_undist_rect, imgDisparity );

    normalize(imgDisparity, imgDisparity8U, 0, 255, CV_MINMAX, CV_8U);

    Mat epi;
    hconcat(left_undist_rect, right_undist_rect, epi);
    for(int j = 0; j < epi.rows; j += 36 )
        line(epi, Point(0, j), Point(epi.cols, j), Scalar(0, 255, 0), 1, 8);
    namedWindow("Epipolar line", WINDOW_NORMAL);
    imshow("Epipolar line", epi);
    namedWindow("Disparity", WINDOW_NORMAL);
    imshow("Disparity", imgDisparity8U);

    cv::Mat XYZ(imgDisparity.size(),CV_32FC3);
    reprojectImageTo3D(imgDisparity8U, XYZ, Q, false, CV_32F);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i=0; i<XYZ.cols; i++)
    {
        for(int j=0; j<XYZ.rows; j++)
        {
            Vec3f &pos_vec = XYZ.at<Vec3f>(j, i);
            if((pos_vec[2] > 1.0 && pos_vec[2] < 20.0) || (pos_vec[2] > -20.0 && pos_vec[2] < -1.0))
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

    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(point_cloud);
    while( !viewer.wasStopped() );
}
