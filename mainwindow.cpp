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
    float nmt = ui->slider_NMT->value() / 100.0f;
    int nmn = ui->slider_NMN->value();
    float rt = ui->slider_RT->value() / 1.0f;

    tracker = new Tracker(mnf, ql, md, nmt, nmn, rt);
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
    targetName = targetNameFull.substr(7, targetNameFull.length()-8);
    lat = ui->text_LAT->toPlainText().toFloat();
    lon = ui->text_LON->toPlainText().toFloat();
    head = ui->text_HD->toPlainText().toFloat();

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
    Mat minHomo;
    float minScore = HOMO_FAIL_SCORE+1;
    for(int i=0; i<MATCH_STEP; i++)
    {
        Mat curFrame = fetcher->get(Size(IMAGE_WIDTH, IMAGE_HEIGHT), lat, lon, head+MATCH_head_LENGTH*i, pitch);
        trackRes = tracker->match(curFrame);
        if(trackRes.score < minScore && norm(trackRes.homo)<HOMO_NORM_THRES)
        {
            minScore = trackRes.score;
            idx = i;
        }
    }

    ui->text_log->appendPlainText("Matched Result:");
    Mat matchedFrame = fetcher->get(Size(IMAGE_WIDTH, IMAGE_HEIGHT), lat, lon, head+MATCH_head_LENGTH*idx, pitch);
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
    vector<Point2f> projectedPoints;
    Mat resImg = targetFrame.clone();
    if(trackRes.score < HOMO_FAIL_SCORE) {
        perspectiveTransform(laneRes.dots, projectedPoints, trackRes.homo);
        for(int i=0; i<(int)projectedPoints.size(); i++)
        {
            resImg.at<Vec3b>(projectedPoints[i]) = Vec3b(0, 255, 255);
        }
        //	Mat roi = lineImg(Rect(lineImg.cols/4,lineImg.rows/2,lineImg.cols/2,lineImg.rows/2));
        //	roi.copyTo(matchedImg(Rect(matchedImg.cols/4,matchedImg.rows/2,matchedImg.cols/2,matchedImg.rows/2)));
    }

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
    ui->label_MNF->setText(QString("Max Num Features: ") + QString::number(ui->slider_MNF->value()));
    ui->label_QL->setText(QString("Quality Level: ") + QString::number(ui->slider_QL->value() / 100.0f));
    ui->label_MD->setText(QString("Min Distance: ") + QString::number(ui->slider_MD->value()));
    ui->label_NMT->setText(QString("NN Match Thres: ") + QString::number(ui->slider_NMT->value() / 100.0f));
    ui->label_NMN->setText(QString("NN Match Number: ") + QString::number(ui->slider_NMN->value()));
    ui->label_RT->setText(QString("RANSAC Thres: ") + QString::number(ui->slider_RT->value() / 1.0f));
    int mnf = ui->slider_MNF->value();
    float ql = ui->slider_QL->value() / 100.0f;
    int md = ui->slider_MD->value();
    float nmt = ui->slider_NMT->value() / 100.0f;
    int nmn = ui->slider_NMN->value();
    float rt = ui->slider_RT->value() / 1.0f;
    tracker->changeParam(mnf, ql, md, nmt, nmn, rt);

    process(targetName, lat, lon, head, 0);
}

void MainWindow::on_button_reset_clicked()
{
    ui->slider_MNF->setValue(1000);
    ui->slider_QL->setValue(1);
    ui->slider_MD->setValue(15);
    ui->slider_NMT->setValue(81);
    ui->slider_NMN->setValue(4);
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

//void MainWindow::on_slider_MNF_valueChanged(int mnf)
//{
//    string mnfT = "Max Num Features: ";
//    mnfT += to_string(mnf);
//    ui->label_MNF->setText(mnfT.c_str());
//    changeParamAndReprocess();
//}

//void MainWindow::on_slider_QL_valueChanged(int qli)
//{
//    string qlT = "Quality Level: ";
//    float ql = qli / 100.0f;
//    qlT += to_string(ql);
//    ui->label_QL->setText(qlT.c_str());
//    changeParamAndReprocess();
//}

//void MainWindow::on_slider_MD_valueChanged(int md)
//{
//    string mdT = "Min Distance: ";
//    mdT += to_string(md);
//    ui->label_MD->setText(mdT.c_str());
//    changeParamAndReprocess();
//}

//void MainWindow::on_slider_NMT_valueChanged(int nmti)
//{
//    string nmtT = "NN Match Thres: ";
//    float nmt = nmti / 100.0f;
//    nmtT += to_string(nmt);
//    ui->label_NMT->setText(nmtT.c_str());
//    changeParamAndReprocess();
//}

//void MainWindow::on_slider_NMN_valueChanged(int nmn)
//{
//    string nmnT = "NN Match Number: ";
//    nmnT += to_string(nmn);
//    ui->label_NMN->setText(nmnT.c_str());
//    changeParamAndReprocess();
//}

//void MainWindow::on_slider_RT_valueChanged(int rti)
//{
//    string rtT = "RANSAC Thres: ";
//    float rt = rti / 1.0f;
//    rtT += to_string(rt);
//    ui->label_RT->setText(rtT.c_str());
//    changeParamAndReprocess();
//}
