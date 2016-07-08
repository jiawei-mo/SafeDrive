#include "mainwindow.h"
#include "ui_mainwindow.h"

#define MATCH_STEP 1
#define MATCH_head_LENGTH 2

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    int mnf = ui->slider_MNF->value();
    float ql = ui->slider_QL->value() / 100.0f;
    int md = ui->slider_MD->value();
    int bs = ui->slider_BS->value() * 2 - 1;
    float bv = ui->slider_BV->value() / 10.0f;
    float nmt = ui->slider_NMT->value() / 100.0f;
    float rt = ui->slider_RT->value() / 1.0f;

    tracker = new Tracker(mnf, ql, md, bs, bv, nmt, rt);
    detector = new LaneDetector();
    fetcher = new GSVFetcher();
}

MainWindow::~MainWindow()
{
    delete ui;
}

bool imgBoundValid(const Mat img, Point2f pt) {
    bool a = pt.x >= 0;
    bool b = pt.x < img.cols;
    bool c = pt.y >=0;
    bool d = pt.y < img.rows;
    return a && b && c && d;
}

void MainWindow::on_button_start_clicked()
{
    string targetNameFull = ui->text_TFN->toPlainText().toStdString();
    targetName = targetNameFull.substr(7, targetNameFull.length()-8);
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

    process(targetName, lat, lon, head, 0);
}

void MainWindow::process(string targetName, float lat, float lon, float head, float pitch)
{
    Mat targetFrame = imread(targetName);
    if( targetFrame.empty() ) {
      ui->text_log->appendPlainText("Error occured, image not read correctly");
      return;
    }
    tracker->setTarget(targetFrame);

    int idx = 0;
    TrackRes trackRes;
    float minScore = HOMO_FAIL_SCORE+1;
    Mat curFrame;
    for(int i=0; i<MATCH_STEP; i++)
    {
        curFrame = fetcher->get(targetFrame.size(), lat, lon, head+MATCH_head_LENGTH*i, pitch);
        trackRes = tracker->match(curFrame);
        if(trackRes.score < minScore && norm(trackRes.homo)<HOMO_NORM_THRES)
        {
            minScore = trackRes.score;
            idx = i;
        }
    }
    curFrame.release();

    ui->text_log->appendPlainText("Matched Result:");
    Mat matchedFrame = fetcher->get(targetFrame.size(), lat, lon, head+MATCH_head_LENGTH*idx, pitch);
    trackRes = tracker->match(matchedFrame);
    if(trackRes.homo.empty()) {
        ui->text_log->appendPlainText("Fail!");
    } else {
        ui->text_log->appendPlainText("Success!");
        ui->text_log->appendPlainText(QString("Latitude: ") + QString::number(lat) + QString(" Longitude: ") + QString::number(lon) + QString(" Heading: ") + QString::number(head));
        ui->text_log->appendPlainText(QString("Average projection error: ") + QString::number(trackRes.score) + QString(" Homo Norm: ") + QString::number(norm(trackRes.homo)) + QString("\n"));
    }

    LaneRes laneRes;
    laneRes = detector->process(matchedFrame);
    matchedFrame.release();
    vector<Point2f> whiteProjectedPoints, yellowProjectedPoints;
    Mat resImg = targetFrame.clone();
    targetFrame.release();
    if(trackRes.score < HOMO_FAIL_SCORE) {
        perspectiveTransform(laneRes.whitePoints, whiteProjectedPoints, trackRes.homo);
        perspectiveTransform(laneRes.yellowPoints, yellowProjectedPoints, trackRes.homo);
        for(int i=0; i<(int)whiteProjectedPoints.size(); i++)
        {
            if((imgBoundValid(resImg, whiteProjectedPoints[i]))) {
                resImg.at<Vec3b>(whiteProjectedPoints[i]) = Vec3b(255, 255, 255);
            }
        }
        for(int i=0; i<(int)yellowProjectedPoints.size(); i++)
        {
            if((imgBoundValid(resImg, yellowProjectedPoints[i]))) {
                resImg.at<Vec3b>(yellowProjectedPoints[i]) = Vec3b(0, 255, 255);
            }
        }
    }

    Mat matchedImg = trackRes.matchedImg;
    cvtColor(matchedImg, matchedImg, CV_BGR2RGB);
    QImage QMatchedImg((uchar*)matchedImg.data, matchedImg.cols, matchedImg.rows, matchedImg.step, QImage::Format_RGB888);
    ui->label_match->setPixmap(QPixmap::fromImage(QMatchedImg));
    matchedImg.release();

    Mat laneImg = laneRes.laneImg;
    cvtColor(laneImg, laneImg, CV_BGR2RGB);
    QImage QlaneImg((uchar*)laneImg.data, laneImg.cols, laneImg.rows, laneImg.step, QImage::Format_RGB888);
    ui->label_lane->setPixmap(QPixmap::fromImage(QlaneImg));
    laneImg.release();

    cvtColor(resImg, resImg, CV_BGR2RGB);
    QImage QresImg((uchar*)resImg.data, resImg.cols, resImg.rows, resImg.step, QImage::Format_RGB888);
    ui->label_result->setPixmap(QPixmap::fromImage(QresImg));
    resImg.release();
}

void MainWindow::changeParamAndReprocess()
{
    ui->label_MNF->setText(QString("Max Num Features: ") + QString::number(ui->slider_MNF->value()));
    ui->label_QL->setText(QString("Quality Level: ") + QString::number(ui->slider_QL->value() / 100.0f));
    ui->label_MD->setText(QString("Min Distance: ") + QString::number(ui->slider_MD->value()));
    ui->label_BS->setText(QString("Blur Size: ") + QString::number(ui->slider_BS->value()));
    ui->label_BV->setText(QString("Blur Var: ") + QString::number(ui->slider_BV->value() / 10.0f));
    ui->label_NMT->setText(QString("NN Match Thres: ") + QString::number(ui->slider_NMT->value() / 100.0f));
    ui->label_RT->setText(QString("RANSAC Thres: ") + QString::number(ui->slider_RT->value() / 1.0f));
    int mnf = ui->slider_MNF->value();
    float ql = ui->slider_QL->value() / 100.0f;
    int md = ui->slider_MD->value();
    int bs = ui->slider_BS->value() * 2 - 1;
    float bv = ui->slider_BV->value() / 10.0f;
    float nmt = ui->slider_NMT->value() / 100.0f;
    float rt = ui->slider_RT->value() / 1.0f;
    tracker->changeParam(mnf, ql, md, bs, bv, nmt, rt);

    process(targetName, lat, lon, head, 0);
}

void MainWindow::on_button_reset_clicked()
{
    ui->slider_MNF->setValue(1000);
    ui->slider_QL->setValue(1);
    ui->slider_MD->setValue(11);
    ui->slider_BS->setValue(3);
    ui->slider_BV->setValue(12);
    ui->slider_NMT->setValue(81);
    ui->slider_RT->setValue(20);
    changeParamAndReprocess();
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

void MainWindow::on_slider_NMN_sliderReleased()
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
