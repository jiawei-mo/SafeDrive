#include "headers/manager.hpp"

Manager::Manager()
{
    three_d_handler = shared_ptr<ThreeDHandler>(new ThreeDHandler());
    matcher = shared_ptr<Matcher>(new Matcher());
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

void Manager::changeParam(int bs, float bv, int mnf, float ql, int md,  int ngf, float mtf, float rtf, int sws, int nd, int pfc, int mod, int ur, int sw, int sr, int dmd, int s1, int s2)
{
    blur_size = bs;
    blur_var = bv;
    matcher->changeParam(mnf, ql, md, ngf, mtf);
    three_d_handler->changeParam(rtf, sws, nd, pfc, mod, ur, sw, sr, dmd, s1, s2);
}

bool Manager::findBestMatch()
{
    //***********************************************search for most similar image***************************************************************
//    int mD(INT_MIN), dbD(INT_MIN), lD(INT_MIN), rD(INT_MIN);
//    float mLat = lat;
//    float mLon = lon;
//    float mHead = head;
//    float mPitch = pitch;
//    Mat dbFrame, lFrame, rFrame;

    //Lat/Lon grid search
//    #pragma omp parallel for
//    for(int a=0; a<MATCH_STEP_G; a++)
//    {
//        float dbLat = lat+MATCH_LAT_LENGTH*(a - MATCH_STEP_G / 2);
////        #pragma omp parallel for
//        for(int b=0; b<MATCH_STEP_G; b++)
//        {
//            float dbLon = lon+MATCH_LON_LENGTH*(b - MATCH_STEP_G / 2);
//            if(!fetcher->get(dbFrame, targetFrame.size(), dbLat, dbLon, mHead, mPitch, searchPath)) {
//                ui->text_log->appendPlainText("Frame missing!");
//                return false;
//            }
//            dbD = matcher->match(dbFrame, featureRes);
//            if(dbD > mD)
//            {
//                mD = dbD;
//                mLat = dbLat;
//                mLon = dbLon;
//            }
//        }
//    }

//    //Head
//    float lHead = mHead-10;
//    if(!fetcher->get(lFrame, targetFrame.size(), mLat, mLon, lHead, mPitch, searchPath)) {
//        ui->text_log->appendPlainText("Frame missing!");
//        return false;
//    }
//    lD = matcher->match(lFrame, featureRes);

//    float rHead = mHead+10;
//    if(!fetcher->get(rFrame, targetFrame.size(), mLat, mLon, rHead, mPitch, searchPath)) {
//        ui->text_log->appendPlainText("Frame missing!");
//        return false;
//    }
//    rD = matcher->match(rFrame, featureRes);
//    for(int c=0; c<MATCH_STEP_L; c++)
//    {
//        mHead = lHead + (rHead-lHead) / 2;
//        if(!fetcher->get(dbFrame, targetFrame.size(), mLat, mLon, mHead, mPitch, searchPath)) {
//            ui->text_log->appendPlainText("Frame missing!");
//            return false;
//        }
//        mD = matcher->match(dbFrame, featureRes);
//        if(lD > rD)
//        {
//            rHead = mHead;
//            rD = mD;
//        } else {
//            lHead = mHead;
//            lD = mD;
//        }
//    }
//    if(lD>mD && lD>rD) {
//        mHead = lHead;
//    } else if(rD>mD && rD>lD) {
//        mHead = rHead;
//    }


//    //Pitch
//    float lPitch = mPitch-5;
//    if(!fetcher->get(lFrame, targetFrame.size(), mLat, mLon, mHead, lPitch, searchPath)) {
//        ui->text_log->appendPlainText("Frame missing!");
//        return false;
//    }
//    lD = matcher->match(lFrame, featureRes);

//    float rPitch = mPitch+5;
//    if(!fetcher->get(rFrame, targetFrame.size(), mLat, mLon, mHead, rPitch, searchPath)) {
//        ui->text_log->appendPlainText("Frame missing!");
//        return false;
//    }
//    rD = matcher->match(rFrame, featureRes);
//    for(int d=0; d<MATCH_STEP_L; d++)
//    {
//        mPitch = lPitch + (rPitch-lPitch) / 2;
//        if(!fetcher->get(dbFrame, targetFrame.size(), mLat, mLon, mHead, mPitch, searchPath)) {
//            ui->text_log->appendPlainText("Frame missing!");
//            return false;
//        }
//        mD = matcher->match(dbFrame, featureRes);
//        if(lD > rD)
//        {
//            rPitch = mPitch;
//            rD = mD;
//        } else {
//            lPitch = mPitch;
//            lD = mD;
//        }
//    }
//    if(lD>mD && lD>rD) {
//        mPitch = lPitch;
//    } else if(rD>mD && rD>lD) {
//        mPitch = rPitch;
//    }

//    if(!fetcher->get(matchedFrame, targetFrame.size(), mLat, mLon, mHead, mPitch, searchPath)) {
//        ui->text_log->appendPlainText("Frame missing!");
//        return false;
//    }

    //    ui->text_log->appendPlainText("Result:");
    //    ui->text_log->appendPlainText(QString("Latitude: ") + QString::number(mLat)
    //                                  + QString(" Longitude: ") + QString::number(mLon)
    //                                  + QString(" Heading: ") + QString::number(mHead)
    //                                  + QString(" Pitch: ") + QString::number(mPitch));

    //***********************************************search for most similar image***************************************************************


    matchedFrameLeft = imread("/home/kimiwings/SafeDrive/test/1.jpg");
    GaussianBlur(matchedFrameLeft, matchedFrameLeft, Size(blur_size,blur_size), blur_var, blur_var);
    matchedFrameRight = imread("/home/kimiwings/SafeDrive/test/2.jpg");
    GaussianBlur(matchedFrameLeft, matchedFrameLeft, Size(blur_size,blur_size), blur_var, blur_var);

    return true;
}

void Manager::process()
{
    if(!findBestMatch()) {
        return;
    }

    vector<Point2f> matchedLeftKp, matchedRightKp, targetKp;

    matcher->match(matchedFrameLeft, matchedLeftKp, matchedFrameRight, matchedRightKp, false);

    Mat disp_img, Q;
    three_d_handler->findDisparity(disp_img, Q, matchedFrameLeft, matchedLeftKp, matchedFrameRight, matchedRightKp);

    matcher->match(matchedFrameLeft, matchedLeftKp, targetFrame, targetKp, true);

    Mat result = targetFrame.clone();
    three_d_handler->project(result, targetKp, disp_img, matchedFrameLeft, matchedLeftKp, Q);

#ifdef QT_DEBUG
    namedWindow("Result", WINDOW_NORMAL);
    imshow("Result", result);
    waitKey(1);
#endif

}
