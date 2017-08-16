#ifndef MANAGER_H
#define MANAGER_H

#include "headers/three_d_handler.hpp"
#include "headers/matcher.hpp"
#include "headers/lane_detector.hpp"
#include "headers/img_fetcher.hpp"
#include "headers/parameters.hpp"

using namespace cv;
using namespace std;

class Manager
{
private:
    shared_ptr<ThreeDHandler> three_d_handler;
    shared_ptr<Matcher> matcher;
    shared_ptr<LaneDetector> lane_detector;
    shared_ptr<IMGFetcher> fetcher;

    string searchPath;

    float lat;
    float lon;
    float head;
    float pitch;

    Mat targetFrame;
    Mat matchedFrameLeft;
    Mat matchedFrameRight;

    bool findBestMatch();
public:
    Manager();
    void changeParam(int mnf, float ql, int md, float mtf, float mtg, float rte, float rtp, int mlrd, int mto);
    void initialize(string targetName, const vector<float>& K, const vector<float>& D, float lt, float ln, float hd, float ph, string sp);
    void process();
};

#endif // MANAGER_H
