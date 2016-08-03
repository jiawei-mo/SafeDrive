#include "omp.h"

#include "mainwindow.h"
#include "ui_mainwindow.h"

#define MATCH_STEP 2
#define MATCH_LAT_LENGTH 0.0001
#define MATCH_LON_LENGTH 0.0001
#define MATCH_HEAD_LENGTH 2
#define MATCH_PITCH_LENGTH 1

void MainWindow::process()
{
    Mat targetFrame = imread(targetString);
    if( targetFrame.empty() ) {
      ui->text_log->appendPlainText("Error occured, image not read correctly");
      return;
    }

    tracker->setTarget(targetFrame);

    Mat featureRes;
    float minNorm = HOMO_FAIL_NORM+1;
    float minLat = lat;
    float minLon = lon;
    float minHead = head;
    float minPitch = pitch;
    Mat curFrame;
    #pragma omp parallel for
    for(int a=0; a<MATCH_STEP; a++)
    {
        float curLat = lat+MATCH_LAT_LENGTH*a;
        #pragma omp parallel for
        for(int b=0; b<MATCH_STEP; b++)
        {
            float curLon = lon+MATCH_LON_LENGTH*b;
            #pragma omp parallel for
            for(int c=0; c<MATCH_STEP; c++)
            {
                float curHead = head+MATCH_HEAD_LENGTH*c;
                #pragma omp parallel for
                for(int d=0; d<MATCH_STEP; d++)
                {
                    float curPitch = pitch+MATCH_PITCH_LENGTH*d;
                    curFrame = fetcher->get(targetFrame.size(),
                                            curLat,
                                            curLon,
                                            curHead,
                                            curPitch);
                    featureRes = tracker->featureMatch(curFrame, false);
                    if(norm(featureRes) < minNorm)
                    {
                        minNorm = norm(featureRes);
                        minLat = curLat;
                        minLon = curLon;
                        minHead = curHead;
                        minPitch = curPitch;
                    }
                }
            }
        }
    }


    ui->text_log->appendPlainText("Matched Result:");
    Mat matchedFrame = fetcher->get(targetFrame.size(), minLat, minLon, minHead, minPitch);
    featureRes = tracker->featureMatch(matchedFrame, true);

    //fail
    if(featureRes.empty() || norm(featureRes) >= HOMO_FAIL_NORM) {
        ui->text_log->appendPlainText("Fail!");
        return;
    }

    //success
    ui->text_log->appendPlainText("Success!");
    ui->text_log->appendPlainText(QString("Latitude: ") + QString::number(minLat) + QString(" Longitude: ") + QString::number(minLon) + QString(" Heading: ") + QString::number(minHead) + QString(" Pitch: ") + QString::number(minPitch));
    ui->text_log->appendPlainText(QString(" Homo Norm: ") + QString::number(norm(featureRes)) + QString("\n"));

    //pixel-wise compare
    Mat finalHomo = tracker->pixelMatch(matchedFrame, featureRes);

    //pixel benchmark
    Mat matchResC = targetFrame.clone();
    detector->detectAndProject(matchedFrame, matchResC, featureRes);
    cout<<featureRes<<endl;
    imshow("Compare", matchResC);
    tracker->showDifferenceEdge(matchedFrame, targetFrame, "compare diff");

    //detect lane
    Mat matchRes = targetFrame.clone();
    detector->detectAndProject(matchedFrame, matchRes, finalHomo);
    cout<<finalHomo<<endl;

    //put images onto GUI
    cvtColor(matchRes, matchRes, CV_BGR2RGB);
    QImage QresImg((uchar*)matchRes.data, matchRes.cols, matchRes.rows, matchRes.step, QImage::Format_RGB888);
    ui->label_result->setPixmap(QPixmap::fromImage(QresImg));

    return;
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->slider_BS->setValue(BS);
    ui->slider_BV->setValue(BV);
    ui->slider_MNF->setValue(MNF);
    ui->slider_QL->setValue(QL);
    ui->slider_MD->setValue(MD);
    ui->slider_NMT->setValue(NMT);
    ui->slider_RT->setValue(RT);
    ui->slider_BDT->setValue(BDT);
    ui->slider_BDS->setValue(BDS);
    int bs = BS * 2 - 1;
    float bv = BV / 10.0f;
    int mnf = MNF;
    float ql = QL / 1000.0f;
    int md = MD;
    float nmt = NMT / 100.0f;
    float rt = RT / 1.0f;
    int bdt = ui->slider_BDT->value();
    float bds = ui->slider_BDS->value() * 2 - 1;

    tracker = new Tracker(mnf, ql, md, bs, bv, nmt, rt, bdt, bds);
    detector = new LaneDetector();
    fetcher = new GSVFetcher();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::changeParamAndReprocess()
{
    ui->label_BS->setText(QString("Blur Size: ") + QString::number(ui->slider_BS->value()));
    ui->label_BV->setText(QString("Blur Var: ") + QString::number(ui->slider_BV->value() / 10.0f));
    ui->label_MNF->setText(QString("Max Num Features: ") + QString::number(ui->slider_MNF->value()));
    ui->label_QL->setText(QString("Quality Level: ") + QString::number(ui->slider_QL->value() / 1000.0f));
    ui->label_MD->setText(QString("Min Distance: ") + QString::number(ui->slider_MD->value()));
    ui->label_NMT->setText(QString("NN Match Thres: ") + QString::number(ui->slider_NMT->value() / 100.0f));
    ui->label_RT->setText(QString("RANSAC Thres: ") + QString::number(ui->slider_RT->value() / 1.0f));
    ui->label_BDT->setText(QString("Board Thres: ") + QString::number(ui->slider_BDT->value()));
    ui->label_BDS->setText(QString("Board Size: ") + QString::number(ui->slider_BDS->value()));
    int bs = ui->slider_BS->value() * 2 - 1;
    float bv = ui->slider_BV->value() / 10.0f;
    int mnf = ui->slider_MNF->value();
    float ql = ui->slider_QL->value() / 1000.0f;
    int md = ui->slider_MD->value();
    float nmt = ui->slider_NMT->value() / 100.0f;
    float rt = ui->slider_RT->value() / 1.0f;
    int bdt = ui->slider_BDT->value();
    float bds = ui->slider_BDS->value() * 2 - 1;
    tracker->changeParam(mnf, ql, md, bs, bv, nmt, rt, bdt, bds);

    process();
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
    ui->slider_BDT->setValue(BDT);
    ui->slider_BDS->setValue(BDS);
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

void MainWindow::on_slider_BDT_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_BDS_sliderReleased()
{
    changeParamAndReprocess();
}
