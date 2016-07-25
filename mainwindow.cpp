#include "opencv2/reg/mapprojec.hpp"
#include "opencv2/reg/mappergradproj.hpp"
#include "opencv2/reg/mapperpyramid.hpp"

#include "mainwindow.h"
#include "ui_mainwindow.h"

#define MATCH_STEP 1
#define MATCH_LAT_LENGTH 0.0001
#define MATCH_LON_LENGTH 0.0001
#define MATCH_HEAD_LENGTH 2
#define MATCH_PITCH_LENGTH 1

using namespace cv::reg;

bool imgBoundValid(const Mat img, Point2f pt) {
    bool a = pt.x >= 0;
    bool b = pt.x < img.cols;
    bool c = pt.y >=0;
    bool d = pt.y < img.rows;
    return a && b && c && d;
}

Mat projDifference(const Mat& image1, const Mat& image2)
{
    Mat img1, img2;
    image1.convertTo(img1, CV_32FC3);
    image2.convertTo(img2, CV_32FC3);
    if(img1.channels() != 1)
        cvtColor(img1, img1, CV_RGB2GRAY);
    if(img2.channels() != 1)
        cvtColor(img2, img2, CV_RGB2GRAY);
//    Canny( image1, img1, 10, 100, 3);
//    Canny( image2, img2, 10, 100, 3);

    Mat imgDiff;
    img1.copyTo(imgDiff);
    imgDiff -= img2;
    imgDiff /= 2.f;
    imgDiff += 128.f;

    Mat imgSh;
    imgDiff.convertTo(imgSh, CV_8UC3);
    return imgSh;
}

//return pair<homo, diffImg>
pair<Mat, Mat> pixelWiseMatch(const Mat& img1, const Mat& img2)
{
    MapperGradProj mapper;
    MapperPyramid mappPyr(mapper);
    Ptr<Map> mapPtr;
    mappPyr.calculate(img1, img2, mapPtr);

    MapProjec* mapProj = dynamic_cast<MapProjec*>(mapPtr.get());
    mapProj->normalize();

    // Generate diffImg
    Mat dest;
    mapProj->inverseWarp(img2, dest);
    Mat diffImg = projDifference(img1, dest);

    return make_pair(Mat(mapProj->getProjTr()), diffImg);
}

void MainWindow::process(string targetString, float lat, float lon, float head, float pitch)
{
    Mat targetFrame = imread(targetString);
    if( targetFrame.empty() ) {
      ui->text_log->appendPlainText("Error occured, image not read correctly");
      return;
    }
    ui->text_log->appendPlainText("Finish loading image!");

    tracker->setTarget(targetFrame);

    TrackRes* trackRes;
//    float minScore = HOMO_FAIL_SCORE+1;
    float minLat = lat;
    float minLon = lon;
    float minHead = head;
    float minPitch = pitch;
    Mat curFrame;
    {
//    for(int a=0; a<MATCH_STEP; a++)
//    {
//        float curLat = lat+MATCH_LAT_LENGTH*a;
//        for(int b=0; b<MATCH_STEP; b++)
//        {
//            float curLon = lon+MATCH_LON_LENGTH*b;
//            for(int c=0; c<MATCH_STEP; c++)
//            {
//                float curHead = head+MATCH_HEAD_LENGTH*c;
//                for(int d=0; d<MATCH_STEP; d++)
//                {
//                    float curPitch = pitch+MATCH_PITCH_LENGTH*d;
//                    curFrame = fetcher->get(targetFrame.size(),
//                                            curLat,
//                                            curLon,
//                                            curHead,
//                                            curPitch);
//                    trackRes = tracker->match(curFrame);
//                    if(trackRes->score < minScore && norm(trackRes->homo)<HOMO_NORM_THRES)
//                    {
//                        minScore = trackRes->score;
//                        minLat = curLat;
//                        minLon = curLon;
//                        minHead = curHead;
//                        minPitch = curPitch;
//                    }
//                }
//            }
//        }
//    }
    }

    ui->text_log->appendPlainText("Matched Result:");
    Mat matchedFrame = fetcher->get(targetFrame.size(), minLat, minLon, minHead, minPitch);
    trackRes = tracker->match(matchedFrame);

    //fail
    if(trackRes->homo.empty() || trackRes->score > HOMO_FAIL_SCORE) {
        ui->text_log->appendPlainText("Fail!");
        return;
    }

    //success
    ui->text_log->appendPlainText("Success!");
    ui->text_log->appendPlainText(QString("Latitude: ") + QString::number(minLat) + QString(" Longitude: ") + QString::number(minLon) + QString(" Heading: ") + QString::number(minHead) + QString(" Pitch: ") + QString::number(minPitch));
    ui->text_log->appendPlainText(QString("Average projection error: ") + QString::number(trackRes->score) + QString(" Homo Norm: ") + QString::number(norm(trackRes->homo)) + QString("\n"));

    //pixelwise compare
    Mat recImg, targetTmp;
    warpPerspective(matchedFrame, recImg, trackRes->homo, targetFrame.size());
//    recImg = matchedFrame.clone();
    recImg.convertTo(recImg, CV_64FC3);
    targetFrame.convertTo(targetTmp, CV_64FC3);
    pair<Mat, Mat> pixelRes = pixelWiseMatch(recImg, targetTmp);
    Mat finalHomo = pixelRes.first;
    Mat diffImg = pixelRes.second;

    LaneRes* laneRes;
    laneRes = detector->process(matchedFrame);
    vector<Point2f> whiteProjectedPoints, yellowProjectedPoints;
    Mat matchRes = targetFrame.clone();
    perspectiveTransform(laneRes->whitePoints, whiteProjectedPoints, finalHomo);
    perspectiveTransform(laneRes->yellowPoints, yellowProjectedPoints, finalHomo);

    //draw lanes
    for(int i=0; i<(int)whiteProjectedPoints.size(); i++)
    {
        if((imgBoundValid(targetFrame, whiteProjectedPoints[i]))) {
            matchRes.at<Vec3b>(whiteProjectedPoints[i]) = Vec3b(255, 255, 255);
        }
    }
    for(int i=0; i<(int)yellowProjectedPoints.size(); i++)
    {
        if((imgBoundValid(targetFrame, yellowProjectedPoints[i]))) {
            matchRes.at<Vec3b>(yellowProjectedPoints[i]) = Vec3b(0, 255, 255);
        }
    }

    //put images onto GUI
    Mat matchedImg = trackRes->matchedImg.clone();
    cvtColor(matchedImg, matchedImg, CV_BGR2RGB);
    QImage QMatchedImg((uchar*)matchedImg.data, matchedImg.cols, matchedImg.rows, matchedImg.step, QImage::Format_RGB888);
    ui->label_match->setPixmap(QPixmap::fromImage(QMatchedImg));

    Mat laneImg = laneRes->laneImg.clone();
    cvtColor(laneImg, laneImg, CV_BGR2RGB);
    QImage QlaneImg((uchar*)laneImg.data, laneImg.cols, laneImg.rows, laneImg.step, QImage::Format_RGB888);
    ui->label_lane->setPixmap(QPixmap::fromImage(QlaneImg));

    cvtColor(diffImg, diffImg, CV_GRAY2RGB);
    QImage QpixelImg((uchar*)diffImg.data, diffImg.cols, diffImg.rows, diffImg.step, QImage::Format_RGB888);
    ui->label_pixel->setPixmap(QPixmap::fromImage(QpixelImg));

    cvtColor(matchRes, matchRes, CV_BGR2RGB);
    QImage QresImg((uchar*)matchRes.data, matchRes.cols, matchRes.rows, matchRes.step, QImage::Format_RGB888);
    ui->label_result->setPixmap(QPixmap::fromImage(QresImg));

    delete trackRes;
    delete laneRes;
    return;
}

void MainWindow::changeParamAndReprocess()
{
    ui->label_MNF->setText(QString("Max Num Features: ") + QString::number(ui->slider_MNF->value()));
    ui->label_QL->setText(QString("Quality Level: ") + QString::number(ui->slider_QL->value() / 1000.0f));
    ui->label_MD->setText(QString("Min Distance: ") + QString::number(ui->slider_MD->value()));
    ui->label_BS->setText(QString("Blur Size: ") + QString::number(ui->slider_BS->value()));
    ui->label_BV->setText(QString("Blur Var: ") + QString::number(ui->slider_BV->value() / 10.0f));
    ui->label_NMT->setText(QString("NN Match Thres: ") + QString::number(ui->slider_NMT->value() / 100.0f));
    ui->label_RT->setText(QString("RANSAC Thres: ") + QString::number(ui->slider_RT->value() / 1.0f));
    int mnf = ui->slider_MNF->value();
    float ql = ui->slider_QL->value() / 1000.0f;
    int md = ui->slider_MD->value();
    int bs = ui->slider_BS->value() * 2 - 1;
    float bv = ui->slider_BV->value() / 10.0f;
    float nmt = ui->slider_NMT->value() / 100.0f;
    float rt = ui->slider_RT->value() / 1.0f;
    tracker->changeParam(mnf, ql, md, bs, bv, nmt, rt);

    process(targetString, lat, lon, head, 0);
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    int mnf = ui->slider_MNF->value();
    float ql = ui->slider_QL->value() / 1000.0f;
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

void MainWindow::on_button_start_clicked()
{
    string targetNameFull = ui->text_TFN->toPlainText().toStdString();
    targetString = targetNameFull.substr(7, targetNameFull.length()-8);
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
    params.close();

    process(targetString, lat, lon, head, 0);
}

void MainWindow::on_button_reset_clicked()
{
    ui->slider_MNF->setValue(1000);
    ui->slider_QL->setValue(1);
    ui->slider_MD->setValue(5);
    ui->slider_BS->setValue(3);
    ui->slider_BV->setValue(12);
    ui->slider_NMT->setValue(78);
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
