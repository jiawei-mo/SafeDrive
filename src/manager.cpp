#include "headers/manager.hpp"

Manager::Manager()
{
    matcher = shared_ptr<Matcher>(new Matcher());
    three_d_handler = shared_ptr<ThreeDHandler>(new ThreeDHandler(matcher));
    lane_detector = shared_ptr<LaneDetector>(new LaneDetector());
    fetcher = shared_ptr<IMGFetcher>(new IMGFetcher());
}

void Manager::initialize(string targetName, const vector<float>& K, const vector<float>& D, float lt, float ln, float hd, float ph, string sp)
{
    targetFrame = imread(targetName);
    lat = lt;
    lon = ln;
    head = hd;
    pitch = ph;
    searchPath = sp;
    fetcher->setCam(K,D);
    three_d_handler->setCamK(K);
}

void Manager::changeParam(int mnf, float ql, int md, float mtf, float mtg, float rte, float rtp)
{
    matcher->changeParam(mnf, ql, md, mtf, mtg);
    three_d_handler->changeParam(matcher, rte, rtp);
}

bool Manager::findBestMatch()
{
    Mat dbFrame;
    size_t firstIdx(-1), secondIdx(-1), firstC(0), secondC(0), curC;
    vector<String> imgNames;
    glob(searchPath, imgNames);
    for(size_t i=0;i<imgNames.size(); i++) {
        if(!fetcher->get_local(dbFrame, imgNames[i])){
            return false;
        }
        curC = matcher->matchCounter(targetFrame, dbFrame);

if(DEBUG) {
        cout<<imgNames[i]<<" "<<curC<<endl;
}

        if(curC > firstC)
        {
            matchedFrameRight = matchedFrameLeft;
            matchedFrameLeft = dbFrame;
            secondIdx = firstIdx;
            secondC = firstC;
            firstIdx = i;
            firstC = curC;
        }
        else if(curC > secondC)
        {
            matchedFrameRight = dbFrame;
            secondIdx = i;
            secondC = curC;
        }
    }

    cout<<"First best image: "<<imgNames[firstIdx]<<endl;
    cout<<"Second best image: "<<imgNames[secondIdx]<<endl;
    return true;
}

void Manager::process()
{
    if(!findBestMatch()) {
        return;
    }

    vector<Point2f> features;
    vector<Point3f> feature_pts, marker_pts;
    vector<Vec3b> marker_color;
    if(!three_d_handler->find3DPoints(matchedFrameLeft, matchedFrameRight, features, feature_pts, marker_pts, marker_color)) return;

    Mat result = targetFrame.clone();
    if(!three_d_handler->project(matchedFrameLeft, result, features, feature_pts, marker_pts, marker_color)) return;

    namedWindow("Result", WINDOW_NORMAL);
    imshow("Result", result);

}
