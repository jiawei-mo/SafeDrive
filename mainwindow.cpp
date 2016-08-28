#include "omp.h"

#include "mainwindow.h"
#include "ui_mainwindow.h"

void MainWindow::process()
{
    tracker->setTarget(targetFrame);

    //***********************************************search for most similar image***************************************************************
    Mat featureRes;
    int mP = -1;
    float mLat = lat;
    float mLon = lon;
    float mHead = head;
    float mPitch = pitch;
    Mat curFrame, lFrame, rFrame;

    //Lat/Lon grid search
    #pragma omp parallel for
    for(int a=0; a<MATCH_STEP_G; a++)
    {
        float curLat = lat+MATCH_LAT_LENGTH*(a - MATCH_STEP_G / 2);
        #pragma omp parallel for
        for(int b=0; b<MATCH_STEP_G; b++)
        {
            float curLon = lon+MATCH_LON_LENGTH*(b - MATCH_STEP_G / 2);
            fetcher->get(curFrame, targetFrame.size(), curLat, curLon, mHead, mPitch);
            float curP = tracker->featureMatch(curFrame, featureRes);
            if(curP > mP)
            {
                mP = curP;
                mLat = curLat;
                mLon = curLon;
            }
        }
    }

    //Head
    float lHead = mHead-10;
    fetcher->get(lFrame, targetFrame.size(), mLat, mLon, lHead, mPitch);
    int lP = tracker->featureMatch(lFrame, featureRes);

    float rHead = mHead+10;
    fetcher->get(rFrame, targetFrame.size(), mLat, mLon, rHead, mPitch);
    int rP = tracker->featureMatch(rFrame, featureRes);
    for(int c=0; c<MATCH_STEP_L; c++)
    {
        if(lP > rP)
        {
            rHead = mHead;
            rP = mP;
        } else {
            lHead = mHead;
            lP = mP;
        }
        mHead = lHead + (rHead-lHead) / 2;
        fetcher->get(curFrame, targetFrame.size(), mLat, mLon, mHead, mPitch);
        mP = tracker->featureMatch(curFrame, featureRes);
//        mP = tracker->featureMatch(curFrame, featureRes, true, to_string(mHead));
    }

    //Pitch
    float lPitch = mPitch-5;
    fetcher->get(lFrame, targetFrame.size(), mLat, mLon, mHead, lPitch);
    lP = tracker->featureMatch(lFrame, featureRes);

    float rPitch = mPitch+5;
    fetcher->get(rFrame, targetFrame.size(), mLat, mLon, mHead, rPitch);
    rP = tracker->featureMatch(rFrame, featureRes);
    for(int d=0; d<MATCH_STEP_L; d++)
    {
        if(lP > rP)
        {
            rPitch = mPitch;
            rP = mP;
        } else {
            lPitch = mPitch;
            lP = mP;
        }
        mPitch = lPitch + (rPitch-lPitch) / 2;
        fetcher->get(curFrame, targetFrame.size(), mLat, mLon, mHead, mPitch);
        mP = tracker->featureMatch(curFrame, featureRes);
    }

    Mat matchedFrame;
    fetcher->get(matchedFrame, targetFrame.size(), mLat, mLon, mHead, mPitch);
    int finalP = tracker->featureMatch(matchedFrame, featureRes, true, "Match Result");
    //***********************************************search for most similar image***************************************************************



    // show result to GUI
    {
    //fail
    if(finalP < 4) {
        ui->text_log->appendPlainText("Fail!");
        return;
    }

    //success
    ui->text_log->appendPlainText("Success!");
    ui->text_log->appendPlainText(QString("Latitude: ") + QString::number(mLat)
                                  + QString(" Longitude: ") + QString::number(mLon)
                                  + QString(" Heading: ") + QString::number(mHead)
                                  + QString(" Pitch: ") + QString::number(mPitch));
    }



    //pixel-wise compare
    Mat finalHomo = tracker->pixelMatch(matchedFrame);
    cout<<"final home: "<<endl<<finalHomo<<endl;



    //pixel benchmark
//    Mat toCompare = targetFrame.clone();
//    detector->detectAndShow(matchedFrame, toCompare, featureRes, "Feature based result");
//    tracker->showDifferenceEdge(matchedFrame, toCompare, "Feature result difference");



    //detect lane and show final result
    Mat projImg = targetFrame.clone();
    detector->detectAndShow(matchedFrame, projImg, finalHomo, "Final result");

    return;
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    tracker = new Tracker();
    detector = new LaneDetector();
    fetcher = new GSVFetcher();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_button_start_clicked()
{
    string targetNameFull = ui->text_TFN->toPlainText().toStdString();
    string targetString = targetNameFull.substr(7, targetNameFull.length()-8);
    targetFrame = imread(targetString);
    if( targetFrame.empty() ) {
      ui->text_log->appendPlainText("Error occured, image not read correctly");
      return;
    }
    string paramNameFull = ui->text_PM->toPlainText().toStdString();
    string paramName = paramNameFull.substr(7, paramNameFull.length()-8);
    ifstream params(paramName);
    string param;
    params >> param;
    lat = stof(param);
    params >> param;
    lon = stof(param);
    params >> param;
    head = stof(param);
    params >> param;
    pitch = stof(param);
    params.close();

    on_button_reset_clicked();
}

void MainWindow::on_button_reset_clicked()
{
    ui->slider_BS->setValue(BS);
    ui->slider_BV->setValue(BV);
    ui->slider_MNF->setValue(MNF);
    ui->slider_QL->setValue(QL);
    ui->slider_MD->setValue(MD);
    ui->slider_NMT->setValue(NMT);
    ui->slider_RT->setValue(RT);
    ui->slider_BDS->setValue(BDS);
    ui->slider_NG->setValue(NG);
    ui->slider_PG->setValue(PG);
    ui->slider_BSG->setValue(BSG);
    ui->slider_BVG->setValue(BVG);
    ui->slider_MTG->setValue(MTG);
    ui->slider_RTG->setValue(RTG);
    changeParamAndReprocess();
}

void MainWindow::changeParamAndReprocess()
{
    ui->label_BS->setText(QString("Blur Size: ") + QString::number(2*ui->slider_BS->value() + 1));
    ui->label_BV->setText(QString("Blur Var: ") + QString::number(ui->slider_BV->value() / 10.0f));
    ui->label_MNF->setText(QString("Max Num Features: ") + QString::number(ui->slider_MNF->value()));
    ui->label_QL->setText(QString("Quality Level: ") + QString::number(ui->slider_QL->value() / 100.0f));
    ui->label_MD->setText(QString("Min Distance: ") + QString::number(ui->slider_MD->value()));
    ui->label_NMT->setText(QString("NN Match Thres: ") + QString::number(ui->slider_NMT->value() / 100.0f));
    ui->label_RT->setText(QString("RANSAC Thres: ") + QString::number(ui->slider_RT->value() / 1.0f));
    ui->label_BDS->setText(QString("Board Size: ") + QString::number(2*ui->slider_BDS->value() + 1));
    ui->label_NG->setText(QString("Num Grid: ") + QString::number(ui->slider_NG->value()));
    ui->label_PG->setText(QString("Point Grid: ") + QString::number(ui->slider_PG->value()));
    ui->label_BSG->setText(QString("Blur Size Grid: ") + QString::number(2*ui->slider_BSG->value() + 1));
    ui->label_BVG->setText(QString("Blur Var Grid: ") + QString::number(ui->slider_BVG->value() / 10.0f));
    ui->label_MTG->setText(QString("Match Thres Grid: ") + QString::number(ui->slider_MTG->value() / 100.0f));
    ui->label_RTG->setText(QString("RANSAC Thres Grid: ") + QString::number(ui->slider_RTG->value() / 1.0f));
    int bs = ui->slider_BS->value() * 2 + 1;
    float bv = ui->slider_BV->value() / 10.0f;
    int mnf = ui->slider_MNF->value();
    float ql = ui->slider_QL->value() / 100.0f;
    int md = ui->slider_MD->value();
    float nmt = ui->slider_NMT->value() / 100.0f;
    float rt = ui->slider_RT->value() / 1.0f;
    float bds = ui->slider_BDS->value() * 2 + 1;
    int ng = ui->slider_NG->value();
    int pg = ui->slider_PG->value();
    int bsg = ui->slider_BSG->value() * 2 + 1;
    float bvg = ui->slider_BVG->value() / 10.0f;
    float mtg = ui->slider_MTG->value() / 100.f;
    float rtg = ui->slider_RTG->value() / 1.0f;
    tracker->changeParam(mnf, ql, md, bs, bv, nmt, rt, bds, ng, pg, bsg, bvg, mtg, rtg);

    process();
}

void MainWindow::on_slider_MNF_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_QL_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_MD_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_NMT_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_RT_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_BS_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_BV_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_BDS_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_NG_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_PG_sliderReleased()
{
    changeParamAndReprocess();
}



void MainWindow::on_slider_BSG_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_BVG_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_MTG_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_RTG_sliderReleased()
{
    changeParamAndReprocess();
}
