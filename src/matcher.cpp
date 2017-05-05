#include "headers/matcher.hpp"

int Matcher::checkArea(Point2f p, int n_c, int n_r, int num_grid) {
    int step_c = n_c / num_grid + 1;
    int step_r = n_r / num_grid + 1;
    int c = p.x / step_c;
    int r = p.y / step_r;
    return r * num_grid + c;
}

Matcher::Matcher()
{
    max_num_features = MNF;
    quality_level = QL / 100.0f;
    min_distance = MD;
    num_grid_feature = NGF;
    match_thres_feature = MTF / 100.0f;
    detector = ORB::create();
    matcher = DescriptorMatcher::create("BruteForce-Hamming");
}

void Matcher::changeParam(int mnf, float ql, int md,  int ngf, float mtf)
{
    max_num_features = mnf;
    quality_level = ql;
    min_distance = md;
    num_grid_feature = ngf;
    match_thres_feature = mtf;
}

void Matcher::match(const Mat& left_img, vector<Point2f>& left_matched_kp, const Mat& right_img, vector<Point2f>& right_matched_kp, int max_corres)
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

    //************************************************grid matches**************************************************************
    int grid_num = num_grid_feature*num_grid_feature;

    vector<vector<KeyPoint> > grid_left_kp(grid_num, vector<KeyPoint>());
    vector<Mat> grid_left_desc(grid_num, Mat());

    vector<vector<KeyPoint> > grid_right_kp(grid_num, vector<KeyPoint>());
    vector<Mat> grid_right_desc(grid_num, Mat());

    int n_c = right_img.cols;
    int n_r = right_img.rows;
    for(int i=0; i<(int)right_kp.size(); i++) {
        int area = checkArea(right_kp[i].pt, n_c, n_r, num_grid_feature);
        grid_right_kp[area].push_back(right_kp[i]);
    }
    for(int i=0; i<(int)left_kp.size(); i++) {
        int area = checkArea(left_kp[i].pt, n_c, n_r, num_grid_feature);
        grid_left_kp[area].push_back(left_kp[i]);
    }

    for(int i=0; i<grid_num; i++) {
        detector->compute(right_img, grid_right_kp[i], grid_right_desc[i]);
        detector->compute(left_img, grid_left_kp[i], grid_left_desc[i]);
    }

    for(int i=0; i<grid_num; i++) {
        vector<vector<DMatch> > matches;
        if(grid_right_desc[i].empty() || grid_left_desc[i].empty()) {
            continue;
        }
        matcher->knnMatch(grid_left_desc[i], grid_right_desc[i], matches, 2);
        vector<DMatch> goodMatches;
        for(int i=0; i<(int)matches.size(); i++) {
            if(matches[i][0].distance < match_thres_feature*matches[i][1].distance)
            {
                matches[i][0].distance = matches[i][0].distance / matches[i][1].distance;
                goodMatches.push_back(matches[i][0]);
            }
        }

        if(max_corres>0 && (int)goodMatches.size() > max_corres) {
            vector<DMatch> matchHeap;
            matchHeap = vector<DMatch>(goodMatches.begin(), goodMatches.begin()+40);
            make_heap(matchHeap.begin(), matchHeap.end(), matchComp());

            for(int j=40; j<(int)goodMatches.size(); j++) {
                if(matchHeap.front().distance > goodMatches[j].distance) {
                    pop_heap(matchHeap.begin(), matchHeap.end(), matchComp());
                    matchHeap.pop_back();
                    matchHeap.push_back(goodMatches[j]);
                    push_heap(matchHeap.begin(), matchHeap.end(), matchComp());
                }
            }
            goodMatches = matchHeap;
        }

        for(int j=0; j<(int)goodMatches.size(); j++) {
            left_matched_kp.push_back(grid_left_kp[i][goodMatches[j].queryIdx].pt);
            right_matched_kp.push_back(grid_right_kp[i][goodMatches[j].trainIdx].pt);
        }
    }
    //************************************************grid matches**************************************************************

    return;
}

size_t Matcher::matchCounter(const Mat& left_frame, const Mat& right_img)
{
    vector<Point2f> dummy_kp_left, dummy_kp_right;
    match(left_frame, dummy_kp_left, right_img, dummy_kp_right);
    return dummy_kp_right.size();
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
