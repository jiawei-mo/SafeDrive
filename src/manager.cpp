#include "headers/manager.hpp"

Manager::Manager()
{
    matcher = shared_ptr<Matcher>(new Matcher());
    three_d_handler = shared_ptr<ThreeDHandler>(new ThreeDHandler(matcher));
    lane_detector = shared_ptr<LaneDetector>(new LaneDetector());
    fetcher = shared_ptr<IMGFetcher>(new IMGFetcher());
}

void Manager::initialize(string targetName, float lt, float ln, float hd, float ph, string sp)
{
    targetFrame = imread(targetName);
    lat = lt;
    lon = ln;
    head = hd;
    pitch = ph;
    searchPath = sp;
}

void Manager::changeParam(int mnf, float ql, int md, float mtf, float rtf)
{
    matcher->changeParam(mnf, ql, md, mtf);
    three_d_handler->changeParam(matcher, rtf);
}

bool Manager::findBestMatch()
{
    Mat dbFrame;
    size_t firstC(0), secondC(0), curC;
    vector<String> imgNames;
    glob(searchPath, imgNames);
    for(size_t i=0;i<imgNames.size(); i++) {
        if(!fetcher->get_local(dbFrame, imgNames[i])){
            return false;
        }
        curC = matcher->matchCounter(targetFrame, dbFrame);
        cout<<imgNames[i]<<" "<<curC<<endl;
        if(curC > firstC)
        {
            matchedFrameRight = matchedFrameLeft;
            matchedFrameLeft = dbFrame;
            secondC = firstC;
            firstC = curC;
        }
        else if(curC > secondC)
        {
            matchedFrameRight = dbFrame;
            secondC = firstC;
        }
    }

    return true;
}

void Manager::process()
{
    if(!findBestMatch()) {
        return;
    }

    Mat disp_img, Q;
    three_d_handler->findDisparity(disp_img, Q, matchedFrameLeft, matchedFrameRight);

    Mat result = targetFrame.clone();
    three_d_handler->project(matchedFrameLeft, result, disp_img, Q);

    namedWindow("Result", WINDOW_NORMAL);
    imshow("Result", result);
    waitKey(1);

}
