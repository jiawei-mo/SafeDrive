#include "mainwindow.h"
#include "ui_mainwindow.h"

#define MATCH_STEP 1
#define MATCH_HEADING_LENGTH 2

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    int mnf = ui->slider_MNF->value();
    float ql = ui->slider_QL->value() / 100.0f;
    int md = ui->slider_MD->value();
    float nmt = ui->slider_NMT->value() / 100.0f;
    int nmn = ui->slider_NMN->value();
    float rt = ui->slider_RT->value() / 1.0f;

    tracker = new Tracker(mnf, ql, md, nmt, nmn, rt);
    detector = new LaneDetector();
    fetcher = new GSVFetcher();

    string targetName = "/home/kimiwings/workspace/SafeDrive/images/target.png";
    float lat = 44.9745000;
    float lon = -93.2704977;
    float heading = 218.36;
    float pitch = 0;

    process(targetName, lat, lon, heading, pitch);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::process(string targetName, float lat, float lon, float heading, float pitch)
{
    Mat targetFrame = imread(targetName);
    if( targetFrame.empty() ) {
      cout<<"Error occured, image not read correctly";
    }
    tracker->setTarget(targetFrame);

    int idx = 0;
    TrackRes trackRes;
    Mat minHomo;
    float minScore = HOMO_FAIL_SCORE+1;
    for(int i=0; i<MATCH_STEP; i++)
    {
        Mat curFrame = fetcher->get(Size(IMAGE_WIDTH, IMAGE_HEIGHT), lat, lon, heading+MATCH_HEADING_LENGTH*i, pitch);
        trackRes = tracker->match(curFrame);
        if(trackRes.score < minScore && norm(trackRes.homo)<HOMO_NORM_THRES)
        {
            minScore = trackRes.score;
            idx = i;
        }
    }

    cout<<"Matched Result:"<<endl;
    Mat matchedFrame = fetcher->get(Size(IMAGE_WIDTH, IMAGE_HEIGHT), lat, lon, heading+MATCH_HEADING_LENGTH*idx, pitch);
    trackRes = tracker->match(matchedFrame);

    LaneRes laneRes;
    laneRes = detector->process(matchedFrame);
    vector<Point2f> projectedPoints;
    if(trackRes.score > HOMO_FAIL_SCORE) {
        return;
    }
    perspectiveTransform(laneRes.dots, projectedPoints, trackRes.homo);
    Mat resImg = targetFrame.clone();
    for(int i=0; i<(int)projectedPoints.size(); i++)
    {
        resImg.at<Vec3b>(projectedPoints[i]) = Vec3b(0, 255, 255);
    }
//	Mat roi = lineImg(Rect(lineImg.cols/4,lineImg.rows/2,lineImg.cols/2,lineImg.rows/2));
//	roi.copyTo(matchedImg(Rect(matchedImg.cols/4,matchedImg.rows/2,matchedImg.cols/2,matchedImg.rows/2)));

    Mat matchedImg = trackRes.matchedImg;
    cvtColor(matchedImg, matchedImg, CV_BGR2RGB);
    QImage QMatchedImg((uchar*)matchedImg.data, matchedImg.cols, matchedImg.rows, matchedImg.step, QImage::Format_RGB888);
    ui->label_match->setPixmap(QPixmap::fromImage(QMatchedImg));

    Mat laneImg = laneRes.laneImg;
    cvtColor(laneImg, laneImg, CV_BGR2RGB);
    QImage QlaneImg((uchar*)laneImg.data, laneImg.cols, laneImg.rows, laneImg.step, QImage::Format_RGB888);
    ui->label_lane->setPixmap(QPixmap::fromImage(QlaneImg));

    cvtColor(resImg, resImg, CV_BGR2RGB);
    QImage QresImg((uchar*)resImg.data, resImg.cols, resImg.rows, resImg.step, QImage::Format_RGB888);
    ui->label_result->setPixmap(QPixmap::fromImage(QresImg));
}

void MainWindow::changeParamAndReprocess()
{
    int mnf = ui->slider_MNF->value();
    float ql = ui->slider_QL->value() / 100.0f;
    int md = ui->slider_MD->value();
    float nmt = ui->slider_NMT->value() / 100.0f;
    int nmn = ui->slider_NMN->value();
    float rt = ui->slider_RT->value() / 1.0f;
    tracker->changeParam(mnf, ql, md, nmt, nmn, rt);

    string targetName = "/home/kimiwings/workspace/SafeDrive/images/target.png";
    float lat = 44.9745000;
    float lon = -93.2704977;
    float heading = 218.36;
    float pitch = 0;

    process(targetName, lat, lon, heading, pitch);
}

void MainWindow::on_button_run_clicked()
{

}

void MainWindow::on_button_reset_clicked()
{
    ui->slider_MNF->setValue(1000);
    ui->slider_QL->setValue(1);
    ui->slider_MD->setValue(7);
    ui->slider_NMT->setValue(90);
    ui->slider_NMN->setValue(4);
    ui->slider_RT->setValue(20);
    changeParamAndReprocess();
}

void MainWindow::on_slider_MNF_valueChanged(int mnf)
{
    string mnfT = "Max Num Features: ";
    mnfT += to_string(mnf);
    ui->label_MNF->setText(mnfT.c_str());
    changeParamAndReprocess();
}

void MainWindow::on_slider_QL_valueChanged(int qli)
{
    string qlT = "Quality Level: ";
    float ql = qli / 100.0f;
    qlT += to_string(ql);
    ui->label_QL->setText(qlT.c_str());
    changeParamAndReprocess();
}

void MainWindow::on_slider_MD_valueChanged(int md)
{
    string mdT = "Min Distance: ";
    mdT += to_string(md);
    ui->label_MD->setText(mdT.c_str());
    changeParamAndReprocess();
}

void MainWindow::on_slider_NMT_valueChanged(int nmti)
{
    string nmtT = "NN Match Thres: ";
    float nmt = nmti / 100.0f;
    nmtT += to_string(nmt);
    ui->label_NMT->setText(nmtT.c_str());
    changeParamAndReprocess();
}

void MainWindow::on_slider_NMN_valueChanged(int nmn)
{
    string nmnT = "NN Match Number: ";
    nmnT += to_string(nmn);
    ui->label_NMN->setText(nmnT.c_str());
    changeParamAndReprocess();
}

void MainWindow::on_slider_RT_valueChanged(int rti)
{
    string rtT = "RANSAC Thres: ";
    float rt = rti / 1.0f;
    rtT += to_string(rt);
    ui->label_RT->setText(rtT.c_str());
    changeParamAndReprocess();
}
