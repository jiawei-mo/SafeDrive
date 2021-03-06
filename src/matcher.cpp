#include "headers/matcher.hpp"

Matcher::Matcher()
{
    max_num_features = MNF;
    quality_level = QL / 100.0f;
    min_distance = MD;
    match_thres_feature = MTF / 100.0f;
    match_thres_given_kp = MTG / 100.0f;
    detector = ORB::create();
    matcher = DescriptorMatcher::create("BruteForce-Hamming");
}

void Matcher::changeParam(int mnf, float ql, int md, float mtf, float mtg)
{
    max_num_features = mnf;
    quality_level = ql;
    min_distance = md;
    match_thres_feature = mtf;
    match_thres_given_kp = mtg;
}

void Matcher::match_desc(const Mat& left_desc, const Mat& right_desc, vector<pair<int, int> >& matches, float thres, bool bi_direct)
{
    vector<vector<DMatch> > _lr_matches, _rl_matches;
    matcher->knnMatch(left_desc, right_desc, _lr_matches, 2);
    matcher->knnMatch(right_desc, left_desc, _rl_matches, 2);

    vector<vector<DMatch> > lr_matches, rl_matches;
    for(unsigned int i=0; i<_lr_matches.size(); i++) {
        if(float(_lr_matches[i][0].distance) < thres*float(_lr_matches[i][1].distance))
        {
            lr_matches.push_back(_lr_matches[i]);
        }
    }
    for(unsigned int i=0; i<_rl_matches.size(); i++) {
        if(float(_rl_matches[i][0].distance) < thres*float(_rl_matches[i][1].distance))
        {
            rl_matches.push_back(_rl_matches[i]);
        }
    }

    unordered_map<int, int> match_hash;
    for(auto& it:rl_matches) {
        match_hash[it[0].trainIdx] = it[0].queryIdx;
    }

    for(unsigned int i=0; i<lr_matches.size(); i++) {
        if(bi_direct && match_hash.find(lr_matches[i][0].queryIdx) == match_hash.end()) continue;
        matches.push_back({lr_matches[i][0].queryIdx, lr_matches[i][0].trainIdx});
    }
}

void Matcher::match(const Mat& left_img, vector<Point2f>& left_matched_kp, const Mat& right_img, vector<Point2f>& right_matched_kp)
{
    left_matched_kp.clear();
    right_matched_kp.clear();
    if(left_img.empty() || right_img.empty()) {
        return;
    }

    //detect feature points and extract descriptors
    Mat left_gray, right_gray;
    vector<Point2f> left_corners, right_corners;
    cvtColor(left_img, left_gray, CV_BGR2GRAY);
    cvtColor(right_img, right_gray, CV_BGR2GRAY);
    goodFeaturesToTrack(left_gray, left_corners, max_num_features, quality_level, min_distance);
    goodFeaturesToTrack(right_gray, right_corners, max_num_features, quality_level, min_distance);

    vector<KeyPoint> left_kp, right_kp;
    for( size_t i = 0; i < left_corners.size(); i++ ) {
        left_kp.push_back(KeyPoint(left_corners[i], 1.f));
    }
    for( size_t i = 0; i < right_corners.size(); i++ ) {
        right_kp.push_back(KeyPoint(right_corners[i], 1.f));
    }

    Mat left_desc, right_desc;
    detector->compute(left_img,left_kp, left_desc);
    detector->compute(right_img, right_kp, right_desc);
    assert(left_kp.size()==left_desc.rows && right_kp.size()==right_desc.rows);

    vector<pair<int, int> > matches;
    match_desc(left_desc, right_desc, matches, match_thres_feature);

    for(size_t i=0; i<matches.size(); i++)
    {
        left_matched_kp.push_back(left_kp[matches[i].first].pt);
        right_matched_kp.push_back(right_kp[matches[i].second].pt);
    }

    return;
}

void Matcher::match_given_kp(const Mat& template_img, const vector<Point2f>& template_kp, const Mat& match_img, vector<Point2f>& matched_kp, vector<int>& inlier)
{
    matched_kp.clear();
    if(template_img.empty() || match_img.empty()) {
        return;
    }

    //detect feature points and extract descriptors
    Mat match_gray;
    vector<Point2f> match_corners;
    cvtColor(match_img, match_gray, CV_BGR2GRAY);
    goodFeaturesToTrack(match_gray, match_corners, max_num_features, quality_level, min_distance);

    vector<KeyPoint> _template_kp, _matched_kp;
    for( size_t i = 0; i < template_kp.size(); i++ ) {
        _template_kp.push_back(KeyPoint(template_kp[i], 1.f));
    }
    for( size_t i = 0; i < match_corners.size(); i++ ) {
        _matched_kp.push_back(KeyPoint(match_corners[i], 1.f));
    }

    Mat template_desc, match_desc;
    detector->compute(template_img,_template_kp, template_desc);
    detector->compute(match_img, _matched_kp, match_desc);

    vector<vector<DMatch> > matches;
    matcher->knnMatch(template_desc, match_desc, matches, 2);

    for(unsigned int i=0; i<matches.size(); i++) {
        if(float(matches[i][0].distance) < match_thres_given_kp*float(matches[i][1].distance))
        {
            matched_kp.push_back(_matched_kp[matches[i][0].trainIdx].pt);
            inlier.push_back(matches[i][0].queryIdx);
        }
    }

    return;
}

void Matcher::match_given_desc(const Mat& template_desc, const Mat& match_img, vector<Point2f>& matched_kp, vector<int>& inlier)
{
    matched_kp.clear();
    if(template_desc.empty() || match_img.empty()) {
        return;
    }

    //detect feature points and extract descriptors
    Mat match_gray;
    vector<Point2f> match_corners;
    cvtColor(match_img, match_gray, CV_BGR2GRAY);
    goodFeaturesToTrack(match_gray, match_corners, max_num_features, quality_level, min_distance);

    vector<KeyPoint> _matched_kp;
    for( size_t i = 0; i < match_corners.size(); i++ ) {
        _matched_kp.push_back(KeyPoint(match_corners[i], 1.f));
    }

    Mat match_desc;
    detector->compute(match_img, _matched_kp, match_desc);

    vector<vector<DMatch> > matches;
    matcher->knnMatch(template_desc, match_desc, matches, 2);

    for(unsigned int i=0; i<matches.size(); i++) {
        if(float(matches[i][0].distance) < match_thres_given_kp*float(matches[i][1].distance))
        {
            matched_kp.push_back(_matched_kp[matches[i][0].trainIdx].pt);
            inlier.push_back(matches[i][0].queryIdx);
        }
    }

    return;
}

void Matcher::rectified_match(const Mat& left_img, vector<Point2f> &left_matched_kp, const Mat &right_img, vector<Point2f>& right_matched_kp)
{
    left_matched_kp.clear();
    right_matched_kp.clear();
    vector<Point2f> left_kp, right_kp;
    match(left_img, left_kp, right_img, right_kp);

    for(unsigned int i=0; i<left_kp.size(); i++) {
        float y_dist = left_kp[i].y - right_kp[i].y;
        y_dist *= y_dist;
        if(y_dist <= 16) {
            left_matched_kp.push_back(left_kp[i]);
            right_matched_kp.push_back(right_kp[i]);
        }
    }
}

size_t Matcher::matchCounter(const Mat& left_img, const Mat& right_img)
{
    vector<Point2f> dummy_kp_left, dummy_kp_right;
    match(left_img, dummy_kp_left, right_img, dummy_kp_right);
    return dummy_kp_right.size();
}

void Matcher::showMatches(const Mat& left_img, const vector<Point2f>& left_p, const Mat& right_img, const vector<Point2f>& right_p, const string& windowName)
{
    vector<KeyPoint> left_kp, right_kp;
    for(unsigned int i=0; i<left_p.size(); i++)
    {
        left_kp.push_back(KeyPoint(left_p[i], 0.0f));
        right_kp.push_back(KeyPoint(right_p[i], 0.0f));
    }

    Mat left_features = left_img.clone();
    Mat right_features = right_img.clone();
    drawKeypoints(left_features, left_kp, left_features);
    drawKeypoints(right_features, right_kp, right_features);
    Mat corres_img;
    hconcat(left_features, right_features, corres_img);
    //    show matches between two images side by side
    for(unsigned int i=0; i<right_kp.size(); i++)
    {
        line(corres_img, left_kp[i].pt, Point2f(right_kp[i].pt.x+left_img.cols, right_kp[i].pt.y), CV_RGB(255, 0, 0), 2);
    }

    namedWindow(windowName, WINDOW_NORMAL);
    imshow(windowName, corres_img);
    waitKey(1);
}

void Matcher::denseMatchAndGeneratePCL(Mat& left_img, const Mat& right_img, const Mat& Q)
{
    Ptr<StereoSGBM> sbm = StereoSGBM::create( 1, 320, 2, 448, 800, 1, 75, 1, 600, 12);
    Mat imgDisparity, disp_img;
    sbm->compute( left_img, right_img, imgDisparity );
    normalize(imgDisparity, disp_img, 0, 255, CV_MINMAX, CV_8U);

if(DEBUG) {
//    namedWindow("DEBUG:Dense Disparity", WINDOW_NORMAL);
//    imshow("DEBUG:Dense Disparity", disp_img);
}

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

}

void Matcher::showDifference(const Mat& image1, const Mat& image2, string title)
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
    imgDiff /= 2.f;
    imgDiff += 128.f;
    Mat imgSh;
    imgDiff.convertTo(imgSh, CV_8UC3);
    if(title.size()>1) {
        namedWindow(title, WINDOW_NORMAL);
        imshow(title, imgSh);
    }
}

void Matcher::showDifferenceEdge(const Mat& image1, const Mat& image2, string title)
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
