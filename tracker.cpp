#include "tracker.hpp"

int Tracker::checkArea(Point2f p, int n_c, int n_r, int num_grid) {
    int step_c = n_c / num_grid + 1;
    int step_r = n_r / num_grid + 1;
    int c = p.x / step_c;
    int r = p.y / step_r;
    return r * num_grid + c;
}

Tracker::Tracker()
{
    blur_size = BS*2 + 1;
    blur_var = BV / 10.0f;
    max_num_features = MNF;
    quality_level = QL / 100.0f;
    min_distance = MD;
    num_grid_feature = NGF;
    match_thres_feature = MTF / 100.0f;
    ransac_thres_feature = RTF / 1.0f;
    board_size = BDS*2 + 1;
    pp_grid = PG;
    num_grid_pixel = NGP;
    match_thres_pixel = MTP / 100.0f;
    ransac_thres_pixel = RTP / 1.0f;
    detector = ORB::create();
    matcher = DescriptorMatcher::create("BruteForce-Hamming");
}

void Tracker::changeParam(int bs, float bv, int mnf, float ql, int md,  int ngf, float mtf, float rtf, int bds, int pg, int ngp, float mtp, float rtp)
{
    blur_size = bs;
    blur_var = bv;
    max_num_features = mnf;
    quality_level = ql;
    min_distance = md;
    num_grid_feature = ngf;
    match_thres_feature = mtf;
    ransac_thres_feature = rtf;
    board_size = bds;
    pp_grid = pg;
    num_grid_pixel = ngp;
    match_thres_pixel = mtp;
    ransac_thres_pixel = rtp;
}

void Tracker::setTarget(const Mat& frame)
{
    targetFrame = frame.clone();
    Mat targetFrameBlured;
    GaussianBlur(targetFrame, targetFrameBlured, Size(blur_size,blur_size), blur_var, blur_var);
    Mat grayImg;
    vector<Point2f> corners;
    cvtColor(targetFrameBlured, grayImg, CV_BGR2GRAY);
    goodFeaturesToTrack(grayImg, corners, max_num_features, quality_level, min_distance);

    targetKp.clear();
    for( size_t i = 0; i < corners.size(); i++ ) {
        targetKp.push_back(KeyPoint(corners[i], 1.f));
    }
    detector->compute(targetFrameBlured, targetKp, targetDesc);
}

int Tracker::featureMatch(const Mat& frame, Mat& trans, const Mat&camera_K, bool showImg, string windowName, int num_grid, float match_thres, float ransac_thres)
{
    if(frame.empty()) {
        return -1;
    }
    if(num_grid<0) {
        num_grid = num_grid_feature;
    }
    if(match_thres<0) {
        match_thres = match_thres_feature;
    }
    if(ransac_thres<0) {
        ransac_thres = ransac_thres_feature;
    }

    //detect feature points and extract descriptors on dbFrame
    Mat dbBlured;
    GaussianBlur(frame, dbBlured, Size(blur_size,blur_size), blur_var, blur_var);
    Mat dbGray;
    vector<Point2f> corners;
    cvtColor(dbBlured, dbGray, CV_BGR2GRAY);
    goodFeaturesToTrack(dbGray, corners, max_num_features, quality_level, min_distance);

    vector<KeyPoint> dbKp;
    for( size_t i = 0; i < corners.size(); i++ ) {
        dbKp.push_back(KeyPoint(corners[i], 1.f));
    }

    // match result image
    Mat matchedImg;
    if(showImg)
    {
        matchedImg = Mat::zeros(frame.rows, 2*frame.cols, frame.type());
        Mat dbFrameFeatures = dbBlured.clone();
        Mat targetFrameFeatures = targetFrame.clone();
        drawKeypoints(dbFrameFeatures, dbKp, dbFrameFeatures);
        drawKeypoints(targetFrameFeatures, targetKp, targetFrameFeatures);
        dbFrameFeatures.copyTo(matchedImg(Rect(0, 0, frame.cols, frame.rows)));
        targetFrameFeatures.copyTo(matchedImg(Rect(frame.cols, 0, frame.cols, frame.rows)));
    }

    //************************************************grid matches**************************************************************
    vector<Point2f> targetMatchedKp, dbMatchedKp;
    int grid_num = num_grid*num_grid;

    vector<vector<KeyPoint> > gridTargetKp(grid_num, vector<KeyPoint>());
    vector<Mat> gridTargetDesc(grid_num, Mat());

    vector<vector<KeyPoint> > griddbKp(grid_num, vector<KeyPoint>());
    vector<Mat> griddbDesc(grid_num, Mat());

    int n_c = targetFrame.cols;
    int n_r = targetFrame.rows;
    for(int i=0; i<(int)targetKp.size(); i++) {
        int area = checkArea(targetKp[i].pt, n_c, n_r, num_grid);
        gridTargetKp[area].push_back(targetKp[i]);
    }
    for(int i=0; i<(int)dbKp.size(); i++) {
        int area = checkArea(dbKp[i].pt, n_c, n_r, num_grid);
        griddbKp[area].push_back(dbKp[i]);
    }

    Mat targetFrameBlured;
    GaussianBlur(targetFrame, targetFrameBlured, Size(blur_size,blur_size), blur_var, blur_var);

    for(int i=0; i<grid_num; i++) {
        detector->compute(targetFrameBlured, gridTargetKp[i], gridTargetDesc[i]);
        detector->compute(dbBlured, griddbKp[i], griddbDesc[i]);
    }

    for(int i=0; i<grid_num; i++) {
        vector<vector<DMatch> > matches;
        if(gridTargetDesc[i].empty() || griddbDesc[i].empty()) {
            continue;
        }
        matcher->knnMatch(griddbDesc[i], gridTargetDesc[i], matches, 2);
        vector<DMatch> goodMatches;
        for(int i=0; i<(int)matches.size(); i++) {
            if(matches[i][0].distance < match_thres*matches[i][1].distance)
            {
                matches[i][0].distance = matches[i][0].distance / matches[i][1].distance;
                goodMatches.push_back(matches[i][0]);
            }
        }
        vector<DMatch> matchHeap;
        if((int)goodMatches.size() > pp_grid) {
            matchHeap = vector<DMatch>(goodMatches.begin(), goodMatches.begin()+pp_grid);
            make_heap(matchHeap.begin(), matchHeap.end(), matchComp());

            for(int j=pp_grid; j<(int)goodMatches.size(); j++) {
                if(matchHeap.front().distance > goodMatches[j].distance) {
                    pop_heap(matchHeap.begin(), matchHeap.end(), matchComp());
                    matchHeap.pop_back();
                    matchHeap.push_back(goodMatches[j]);
                    push_heap(matchHeap.begin(), matchHeap.end(), matchComp());
                }
            }
        } else {
            matchHeap = goodMatches;
        }
        for(int j=0; j<(int)matchHeap.size(); j++) {
//            cout<<griddbKp[i][matchHeap[j].queryIdx].pt<<endl;
            dbMatchedKp.push_back(griddbKp[i][matchHeap[j].queryIdx].pt);
            targetMatchedKp.push_back(gridTargetKp[i][matchHeap[j].trainIdx].pt);
        }
    }
    if(dbMatchedKp.size() == 0) {
        return -1;
    }
    //************************************************grid matches**************************************************************


    //find essential_mat based on matches using RANSAC
    Mat inliner_mask;
    Mat essential_mat = findEssentialMat(dbMatchedKp, targetMatchedKp, camera_K, RANSAC, 0.99, ransac_thres_feature, inliner_mask);
    Mat R,t;
    recoverPose(essential_mat, dbMatchedKp, targetMatchedKp, camera_K, R, t, inliner_mask);

    Mat rot_vec;
    Rodrigues(R, rot_vec);

    cout<<"R= "<<rot_vec<<endl;
    cout<<"t= "<<t<<endl;

    hconcat(R,t,trans);

    Mat coeff = (Mat_<double>(1,5) << -0.2004, 0.1620, 0, 0, 0);
    Size frame_size = targetFrame.size();
    Mat R1, R2, P1, P2, Q;
    stereoRectify(camera_K, coeff, camera_K, coeff,frame_size, R, t, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 0, frame_size, 0, 0);
//    cout<<R1<<endl<<R2<<endl<<P1<<endl<<P2<<endl<<Q<<endl;
//    cout<<Q<<endl;

    Mat targetGray;
    cvtColor(targetFrameBlured, targetGray, CV_BGR2GRAY);

    Mat left_undist_rect_map_x, left_undist_rect_map_y, right_undist_rect_map_x, right_undist_rect_map_y,left_undist_rect,right_undist_rect,db_color_undist_rect;
    initUndistortRectifyMap(camera_K, coeff, R1, P1, frame_size,CV_16SC2, left_undist_rect_map_x, left_undist_rect_map_y);
    initUndistortRectifyMap(camera_K, coeff, R2, P2, frame_size, CV_16SC2, right_undist_rect_map_x, right_undist_rect_map_y);
    remap(frame, left_undist_rect, left_undist_rect_map_x, left_undist_rect_map_y, INTER_LINEAR);
    remap(targetFrame, right_undist_rect, right_undist_rect_map_x, right_undist_rect_map_y, INTER_LINEAR);
    remap(frame, db_color_undist_rect, left_undist_rect_map_x, left_undist_rect_map_y, INTER_LINEAR);

    int SADWindowSize = 3;
    int numberOfDisparities = 144;
    int preFilterCap = 63;
    int minDisparity = 0;
    int uniquenessRatio = 10;
    int speckleWindowSize = 100;
    int speckleRange = 32;
    int disp12MaxDiff = 1;
    int SP1 = 216;
    int SP2 = 864;
    Ptr<StereoSGBM> sbm = StereoSGBM::create( minDisparity, numberOfDisparities, SADWindowSize, SP1, SP2, disp12MaxDiff, preFilterCap, uniquenessRatio, speckleWindowSize, speckleRange);
    Mat imgDisparity, imgDisparity8U;

//    Mat testDisp, test8;
//    Mat left_test = imread("/home/kimiwings/SafeDrive/test/Sport0_OG0_R.JPG");
//    Mat right_test = imread("/home/kimiwings/SafeDrive/test/Sport1_OG0_R.JPG");
//    sbm->compute( left_test, right_test, testDisp );
    sbm->compute( left_undist_rect, right_undist_rect, imgDisparity );

//    normalize(testDisp, test8, 0, 255, CV_MINMAX, CV_8U);
normalize(imgDisparity, imgDisparity8U, 0, 255, CV_MINMAX, CV_8U);

//cout<<imgDisparity32F<<endl;
    Mat epi;
    hconcat(left_undist_rect, right_undist_rect, epi);
    for(int j = 0; j < epi.rows; j += 36 )
        line(epi, Point(0, j), Point(epi.cols, j), Scalar(0, 255, 0), 1, 8);
    namedWindow("epi", WINDOW_NORMAL);
    imshow("epi", epi);
    namedWindow("test", WINDOW_NORMAL);
    imshow("test", imgDisparity8U);
//    namedWindow("testss", WINDOW_NORMAL);
//    imshow("testss", test8);

    cv::Mat XYZ(imgDisparity.size(),CV_32FC3);
    reprojectImageTo3D(imgDisparity8U, XYZ, Q, false, CV_32F);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i=0; i<XYZ.cols; i++)
    {
        for(int j=0; j<XYZ.rows; j++)
        {
            Vec3f &pos_vec = XYZ.at<Vec3f>(j, i);
//            cout<<pos_vec<<endl;
            if((pos_vec[2] > 1.0 && pos_vec[2] < 20.0) || (pos_vec[2] > -20.0 && pos_vec[2] < -1.0))
            {
//                cout<<pos_vec<<endl;
                pcl::PointXYZRGB p;
                p.x = pos_vec[0];
                p.y = pos_vec[1];
                p.z = pos_vec[2];
                Vec3i &color_vec = db_color_undist_rect.at<Vec3i>(j, i);
                p.r = color_vec[2];
                p.g = color_vec[1];
                p.b = color_vec[0];
                point_cloud->push_back(p);
            }
        }
    }


//    pcl::io::savePCDFile("/home/kimiwings/data/result.pcd", *point_cloud);
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    viewer.showCloud(point_cloud);
    while( !viewer.wasStopped() );

    vector<Point2f> projectedKp;
//    perspectiveTransform(dbMatchedKp, projectedKp, homo);
    for(auto& db_p:dbMatchedKp)
    {
        Mat X = camera_K.inv()*(Mat_<double>(3,1) << db_p.x, db_p.y, 1);
        Mat X_homo = (Mat_<double>(4,1) << X.at<double>(0), X.at<double>(1), X.at<double>(2), 1);
        Mat X_p = camera_K*trans*X_homo;
        projectedKp.push_back(Point2f(X_p.at<double>(0)/X_p.at<double>(2), X_p.at<double>(1)/X_p.at<double>(2)));
    }

    //show inliner pairs between two images side by side
    int inliner_counter = 0;
    float inliner_dist = 0.0;
    for(int i=0; i<(int)targetMatchedKp.size(); i++)
    {
        if(inliner_mask.at<uchar>(i))
        {
            inliner_counter++;
            inliner_dist += (projectedKp[i].x - targetMatchedKp[i].x)*(projectedKp[i].x - targetMatchedKp[i].x);
            inliner_dist += (projectedKp[i].y - targetMatchedKp[i].y)*(projectedKp[i].y - targetMatchedKp[i].y);
            if(showImg) {
                line(matchedImg, dbMatchedKp[i], Point2f(targetMatchedKp[i].x+frame.cols, targetMatchedKp[i].y), CV_RGB(255, 0, 0));
                circle(matchedImg, Point2f(projectedKp[i].x+frame.cols, projectedKp[i].y), 2, CV_RGB(0, 255, 0));
            }
        }
    }


    if(showImg) {
        namedWindow(windowName, WINDOW_NORMAL);
        imshow(windowName, matchedImg);
    }

//    cout<<"norm: "<<norm(homography)<<" inliner_dist: "<<inliner_dist<<endl;
    return inliner_counter;
}

void Tracker::showDifference(const Mat& image1, const Mat& image2, string title)
{
    Mat img1, img2;
    image1.convertTo(img1, CV_32FC3);
    image2.convertTo(img2, CV_32FC3);
    if(img1.channels() != 1)
        cvtColor(img1, img1, CV_RGB2GRAY);
    if(img2.channels() != 1)
        cvtColor(img2, img2, CV_RGB2GRAY);

    Mat imgDiff;
    img1.copyTo(imgDiff);
    imgDiff -= img2;
//    cout<<"diff norm: "<<norm(imgDiff)<<endl;
    imgDiff /= 2.f;
    imgDiff += 128.f;
    Mat imgSh;
    imgDiff.convertTo(imgSh, CV_8UC3);
    if(title.size()>1) {
        namedWindow(title, WINDOW_NORMAL);
        imshow(title, imgSh);
    }
}

void Tracker::showDifferenceEdge(const Mat& image1, const Mat& image2, string title)
{
    Mat img1Tmp, img2Tmp, img1Edge, img2Edge;
    image1.convertTo(img1Tmp, CV_8UC3);
    image2.convertTo(img2Tmp, CV_8UC3);
    Canny( img1Tmp, img1Edge, 50, 150, 3);
    Canny( img2Tmp, img2Edge, 50, 150, 3);

    Mat res(image1.size(), CV_8UC3, Scalar(0, 0, 0));
    Mat redImg(image1.size(), CV_8UC3, Scalar(255, 0, 0));
    Mat blueImg(image1.size(), CV_8UC3, Scalar(0, 0, 255));
    redImg.copyTo(res, img1Edge);
    blueImg.copyTo(res, img2Edge);

    if(title.size()>1) {
        namedWindow(title, WINDOW_NORMAL);
        imshow(title, res);
    }

    return;
}
