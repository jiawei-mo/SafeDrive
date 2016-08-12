#include "tracker.hpp"

Tracker::Tracker()
{
    blur_size = BS*2 + 1;
    blur_var = BV / 10.0f;
    max_num_features = MNF;
    quality_level = QL / 100.0f;
    min_distance = MD;
    nn_match_thres = NMT / 100.0f;
    ransac_thres = RT / 1.0f;
    board_size = BDS*2 + 1;
    targetKp.clear();
    detector = ORB::create();
    matcher = DescriptorMatcher::create("BruteForce-Hamming");
}

void Tracker::changeParam(int mnf, float ql, int md, int bs, float bv, float nmt, float rt, int bds)
{
    max_num_features = mnf;
    quality_level = ql;
    min_distance = md;
    blur_size = bs;
    blur_var = bv;
    nn_match_thres = nmt;
    ransac_thres = rt;
    board_size = bds;
    targetKp.clear();
}

void Tracker::setTarget(const Mat& frame)
{
    targetFrame = frame.clone();
    GaussianBlur(targetFrame, targetFrame, Size(blur_size,blur_size), blur_var, blur_var);
    Mat grayImg;
    vector<Point2f> corners;
    cvtColor(targetFrame, grayImg, CV_BGR2GRAY);
    mask = Mat::zeros(frame.size(), CV_8UC1);
    mask(Rect(0,0,mask.cols/3,mask.rows/2)).setTo(Scalar::all(255));
    mask(Rect(mask.cols*2/3,0,mask.cols/3,mask.rows/2)).setTo(Scalar::all(255));
    goodFeaturesToTrack(grayImg, corners, max_num_features, quality_level, min_distance);

    for( size_t i = 0; i < corners.size(); i++ ) {
        targetKp.push_back(KeyPoint(corners[i], 1.f));
    }
    detector->compute(targetFrame, targetKp, targetDesc);
}

int Tracker::featureMatch(const Mat& frame, Mat& homography, bool showImg, string windowName)
{
    //detect feature points on curFrame
    Mat curFrame = frame.clone();
    GaussianBlur(curFrame, curFrame, Size(blur_size,blur_size), blur_var, blur_var);
    Mat grayImg;
    vector<Point2f> corners;
    cvtColor(curFrame, grayImg, CV_BGR2GRAY);
    goodFeaturesToTrack(grayImg, corners, max_num_features, quality_level, min_distance);

    vector<KeyPoint> curKp;
    for( size_t i = 0; i < corners.size(); i++ ) {
        curKp.push_back(KeyPoint(corners[i], 1.f));
    }

    Mat curDesc;
    detector->compute(curFrame, curKp, curDesc);

    Mat matchedImg = Mat::zeros(frame.rows, 2*frame.cols, frame.type());
    Mat curFrameFeatures, targetFrameFeatures;
    drawKeypoints(curFrame, curKp, curFrameFeatures);
    drawKeypoints(targetFrame, targetKp, targetFrameFeatures);
    curFrameFeatures.copyTo(matchedImg(Rect(0, 0, frame.cols, frame.rows)));
    targetFrameFeatures.copyTo(matchedImg(Rect(frame.cols, 0, frame.cols, frame.rows)));

    //matches
    vector< vector<DMatch> > ctMatches, tcMatches;
    vector<Point2f> curMatchedKp, targetMatchedKp;
    matcher->knnMatch(curDesc, targetDesc, ctMatches, 2);
    matcher->knnMatch(targetDesc, curDesc, tcMatches, 2);
    unordered_map<int, int> matchMap;

    for(int i=0; i<(int)tcMatches.size(); i++)
    {
        if(tcMatches[i][0].distance < nn_match_thres*tcMatches[i][1].distance)
        {
            matchMap[tcMatches[i][0].trainIdx] = tcMatches[i][0].queryIdx;
        }
    }

    for(int i=0; i<(int)ctMatches.size(); i++)
    {
        if(ctMatches[i][0].distance < nn_match_thres*ctMatches[i][1].distance)
        {
            if(matchMap[ctMatches[i][0].queryIdx] == ctMatches[i][0].trainIdx)
            {
                curMatchedKp.push_back(curKp[ctMatches[i][0].queryIdx].pt);
                targetMatchedKp.push_back(targetKp[ctMatches[i][0].trainIdx].pt);
            }
        }
    }



    //find homography based on matches
    Mat inliner_mask;
    if(targetMatchedKp.size() >= NN_MATCH_NUMBER)
    {
        homography = findHomography(curMatchedKp, targetMatchedKp, RANSAC, ransac_thres, inliner_mask);
    }

    //track fail
    if(targetMatchedKp.size() < NN_MATCH_NUMBER || homography.empty() || norm(homography)>=HOMO_FAIL_NORM)
    {
        for(int i=0; i<(int)targetMatchedKp.size(); i++)
        {
            targetMatchedKp[i].x += frame.cols;
            line(matchedImg, curMatchedKp[i], targetMatchedKp[i], CV_RGB(255, 0, 0));
        }
        namedWindow("Match Result", WINDOW_NORMAL);
        imshow("Match Result", matchedImg);
        cout<<"Match fail, norm exceeded!"<<endl;
        return -1;
    }

    vector<Point2f> projectedKp;
    perspectiveTransform(curMatchedKp, projectedKp, homography);

    //show inliner pairs between two images side by side
    int count = 0;
    float dist = 0.0;
    inline_target.clear();
    inline_matched.clear();
    for(int i=0; i<(int)targetMatchedKp.size(); i++)
    {
        if(inliner_mask.at<uchar>(i))
        {
            count++;
            dist += (projectedKp[i].x - targetMatchedKp[i].x)*(projectedKp[i].x - targetMatchedKp[i].x);
            dist += (projectedKp[i].y - targetMatchedKp[i].y)*(projectedKp[i].y - targetMatchedKp[i].y);
            line(matchedImg, curMatchedKp[i], Point2f(targetMatchedKp[i].x+frame.cols, targetMatchedKp[i].y), CV_RGB(255, 0, 0));
            circle(matchedImg, Point2f(projectedKp[i].x+frame.cols, projectedKp[i].y), 2, CV_RGB(0, 255, 0));
            inline_target.push_back(targetMatchedKp[i]);
            inline_matched.push_back(projectedKp[i]);
        }
    }
    if(showImg) {
        namedWindow(windowName, WINDOW_NORMAL);
        imshow(windowName, matchedImg);
    }

    dist = dist / count;
    if(dist > PROJ_ERR_THRES) {
        cout<<"Match fail, proj error exceeded!"<<endl;
        return -1;
    }
    return count;
}

Mat Tracker::pixelMatch(const Mat& recMatchedFrame)
{
    Mat targetDots = Mat::zeros(targetFrame.size(), CV_32F);;
    Mat recMatchedDots = Mat::zeros(recMatchedFrame.size(), CV_32F);
    if(inline_target.size() == 0) {
        return Mat::ones(3, 3, CV_32F);
    }
    for(unsigned int i=0; i<inline_target.size(); i++) {
        targetDots.at<float>(inline_target[i].y, inline_target[i].x) = 1.0;
        recMatchedDots.at<float>(inline_matched[i].y, inline_matched[i].x) = 1.0;
    }

    GaussianBlur(targetDots, targetDots, Size(board_size, board_size), 10, 10);
    GaussianBlur(recMatchedDots, recMatchedDots, Size(board_size, board_size), 10, 10);

    Mat targetROI, recMatchedROI;

    Mat targetDotsC3;
    vector<Mat> targetChannels(3, targetDots);
    merge(targetChannels, targetDotsC3);
    Mat targetFrame32F;
    targetFrame.convertTo(targetFrame32F, CV_32FC3);
    targetROI = targetFrame32F.mul(targetDotsC3);

    Mat recMatchedDotsC3;
    vector<Mat> recMatchedChannels(3, recMatchedDots);
    merge(recMatchedChannels, recMatchedDotsC3);
    Mat recMatchedFrame32F;
    recMatchedFrame.convertTo(recMatchedFrame32F, CV_32FC3);
    recMatchedROI = recMatchedFrame32F.mul(recMatchedDotsC3);

    Mat combineImg = Mat::zeros(recMatchedROI.rows, 2*recMatchedROI.cols, recMatchedROI.type());
    recMatchedROI.copyTo(combineImg(Rect(0, 0, recMatchedROI.cols, recMatchedROI.rows)));
    targetROI.copyTo(combineImg(Rect(recMatchedROI.cols, 0, recMatchedROI.cols, recMatchedROI.rows)));
    namedWindow("Pixel-wise ROI", WINDOW_NORMAL);
    imshow("Pixel-wise ROI", combineImg);

    Mat recMatchedFrame64F, targetFrame64F;
    recMatchedROI.convertTo(recMatchedFrame64F, CV_64FC3);
    targetROI.convertTo(targetFrame64F, CV_64FC3);
    Mat homo = pixelWiseMatch(recMatchedFrame64F, targetFrame64F);
    return homo;
}

//return pair<homo, diffImg>
Mat Tracker::pixelWiseMatch(const Mat& img1, const Mat& img2)
{
    Ptr<Map> res;
    MapperGradProj mapper;
    MapperPyramid mappPyr(mapper);
    mappPyr.numIterPerScale_ = 10;
    mappPyr.calculate(img1, img2, res);

    MapProjec* mapProj = dynamic_cast<MapProjec*>(res.get());
    mapProj->normalize();

    // Generate diffImg
    Mat dest;
    mapProj->inverseWarp(img2, dest);
    showDifference(img1, dest, "Pixel Difference");

    return Mat(mapProj->getProjTr());
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
    imgDiff /= 2.f;
    imgDiff += 128.f;

    Mat imgSh;
    imgDiff.convertTo(imgSh, CV_8UC3);
    namedWindow(title, WINDOW_NORMAL);
    imshow(title, imgSh);
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

    namedWindow(title, WINDOW_NORMAL);
    imshow(title, res);

    return;
}
