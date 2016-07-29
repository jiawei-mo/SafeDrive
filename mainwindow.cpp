#include "opencv2/reg/mapprojec.hpp"
#include "opencv2/reg/mappergradproj.hpp"
#include "opencv2/reg/mapperpyramid.hpp"

#include "omp.h"

#include "mainwindow.h"
#include "ui_mainwindow.h"

#define MATCH_STEP 3
#define MATCH_LAT_LENGTH 0.0001
#define MATCH_LON_LENGTH 0.0001
#define MATCH_HEAD_LENGTH 1
#define MATCH_PITCH_LENGTH 1

using namespace cv::reg;

Mat projDifference(const Mat& image1, const Mat& image2)
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

    return res;
}

//return pair<homo, diffImg>
pair<Mat, Mat> pixelWiseMatch(const Mat& img1, const Mat& img2)
{
    MapperGradProj mapper;
    MapperPyramid mappPyr(mapper);
    mappPyr.numIterPerScale_ = 10;
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

void MainWindow::process()
{
    Mat targetFrame = imread(targetString);
    if( targetFrame.empty() ) {
      ui->text_log->appendPlainText("Error occured, image not read correctly");
      return;
    }

    tracker->setTarget(targetFrame);

    Mat trackRes;
    float minNorm = HOMO_FAIL_NORM+1;
    float minLat = lat;
    float minLon = lon;
    float minHead = head;
    float minPitch = pitch;
    Mat curFrame;
//    #pragma omp parallel for
//    for(int a=0; a<MATCH_STEP; a++)
//    {
//        float curLat = lat+MATCH_LAT_LENGTH*a;
//        #pragma omp parallel for
//        for(int b=0; b<MATCH_STEP; b++)
//        {
//            float curLon = lon+MATCH_LON_LENGTH*b;
//            #pragma omp parallel for
            for(int c=0; c<MATCH_STEP; c++)
            {
                float curHead = head+MATCH_HEAD_LENGTH*c;
//                #pragma omp parallel for
                for(int d=0; d<MATCH_STEP; d++)
                {
                    float curPitch = pitch+MATCH_PITCH_LENGTH*d;
                    curFrame = fetcher->get(targetFrame.size(),
                                            lat,
                                            lon,
                                            curHead,
                                            curPitch);
                    trackRes = tracker->match(curFrame);
                    if(norm(trackRes) < minNorm)
                    {
                        minNorm = norm(trackRes);
                        minLat = lat;
                        minLon = lon;
                        minHead = curHead;
                        minPitch = curPitch;
                    }
                }
            }
//        }
//    }


    ui->text_log->appendPlainText("Matched Result:");
    Mat matchedFrame = fetcher->get(targetFrame.size(), minLat, minLon, minHead, minPitch);
    trackRes = tracker->match(matchedFrame);

    //fail
    if(trackRes.empty() || norm(trackRes) >= HOMO_FAIL_NORM) {
        ui->text_log->appendPlainText("Fail!");
        return;
    }

    //success
    ui->text_log->appendPlainText("Success!");
    ui->text_log->appendPlainText(QString("Latitude: ") + QString::number(minLat) + QString(" Longitude: ") + QString::number(minLon) + QString(" Heading: ") + QString::number(minHead) + QString(" Pitch: ") + QString::number(minPitch));
    ui->text_log->appendPlainText(QString(" Homo Norm: ") + QString::number(norm(trackRes)) + QString("\n"));

    //coarse proj
    Mat recMatchedImg;
    warpPerspective(matchedFrame, recMatchedImg, trackRes, targetFrame.size());

    //pixelwise compare
    Mat recMatchedImgTmp, targetFrameTmp;
//    recMatchedImg = matchedFrame.clone();
//    Canny( recMatchedImg, recMatchedImgTmp, 50, 200, 3);
//    Canny( targetFrame, targetFrameTmp, 50, 200, 3);
    Rect topHalf(0, 0, recMatchedImg.cols, recMatchedImg.rows / 2);
    Mat recHalf(recMatchedImg, topHalf);
    Mat tgtHalf(targetFrame, topHalf);
    recHalf.convertTo(recMatchedImgTmp, CV_64FC3);
    tgtHalf.convertTo(targetFrameTmp, CV_64FC3);
    pair<Mat, Mat> pixelRes = pixelWiseMatch(recMatchedImgTmp, targetFrameTmp);
    Mat finalHomo = pixelRes.first;
    Mat diffImg = pixelRes.second;
    imshow("Pixel Difference", diffImg);

    //detect lane
    Mat matchRes = targetFrame.clone();
    detector->detectAndProject(recMatchedImg, matchRes, finalHomo.inv());

    //detect lane compare
    Mat matchResC = targetFrame.clone();
    detector->detectAndProject(matchedFrame, matchResC, trackRes);
    imshow("Compare", matchResC);

    //put images onto GUI
    cvtColor(matchRes, matchRes, CV_BGR2RGB);
    QImage QresImg((uchar*)matchRes.data, matchRes.cols, matchRes.rows, matchRes.step, QImage::Format_RGB888);
    ui->label_result->setPixmap(QPixmap::fromImage(QresImg));

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

    process();
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
    params >> param;
    pitch = stof(param);
    params.close();

    process();
}

void MainWindow::on_button_reset_clicked()
{
    ui->slider_BS->setValue(3);
    ui->slider_BV->setValue(12);
    ui->slider_MNF->setValue(1000);
    ui->slider_QL->setValue(10);
    ui->slider_MD->setValue(5);
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
