#include "omp.h"

#include "mainwindow.h"
#include "ui_mainwindow.h"

bool MainWindow::findBestMatch() {
    tracker->setTarget(targetFrame);

    //***********************************************search for most similar image***************************************************************
    Mat featureRes;
    vector<Point2f> inline_matched_useless;
    int mD(INT_MIN), curD(INT_MIN), lD(INT_MIN), rD(INT_MIN);
    float mLat = lat;
    float mLon = lon;
    float mHead = head;
    float mPitch = pitch;
    Mat curFrame, lFrame, rFrame;

    //Lat/Lon grid search
//    #pragma omp parallel for
    for(int a=0; a<MATCH_STEP_G; a++)
    {
        float curLat = lat+MATCH_LAT_LENGTH*(a - MATCH_STEP_G / 2);
//        #pragma omp parallel for
        for(int b=0; b<MATCH_STEP_G; b++)
        {
            float curLon = lon+MATCH_LON_LENGTH*(b - MATCH_STEP_G / 2);
            if(!fetcher->get(curFrame, targetFrame.size(), curLat, curLon, mHead, mPitch, searchPath)) {
                ui->text_log->appendPlainText("Frame missing!");
                return false;
            }
            curD = tracker->featureMatch(curFrame, featureRes, &inline_matched_useless);
            if(curD > mD)
            {
                mD = curD;
                mLat = curLat;
                mLon = curLon;
            }
        }
    }

    //Head
    float lHead = mHead-10;
    if(!fetcher->get(lFrame, targetFrame.size(), mLat, mLon, lHead, mPitch, searchPath)) {
        ui->text_log->appendPlainText("Frame missing!");
        return false;
    }
    lD = tracker->featureMatch(lFrame, featureRes, &inline_matched_useless);

    float rHead = mHead+10;
    if(!fetcher->get(rFrame, targetFrame.size(), mLat, mLon, rHead, mPitch, searchPath)) {
        ui->text_log->appendPlainText("Frame missing!");
        return false;
    }
    rD = tracker->featureMatch(rFrame, featureRes, &inline_matched_useless);
    for(int c=0; c<MATCH_STEP_L; c++)
    {
        mHead = lHead + (rHead-lHead) / 2;
        if(!fetcher->get(curFrame, targetFrame.size(), mLat, mLon, mHead, mPitch, searchPath)) {
            ui->text_log->appendPlainText("Frame missing!");
            return false;
        }
        mD = tracker->featureMatch(curFrame, featureRes, &inline_matched_useless);
        if(lD > rD)
        {
            rHead = mHead;
            rD = mD;
        } else {
            lHead = mHead;
            lD = mD;
        }
    }
    if(lD>mD && lD>rD) {
        mHead = lHead;
    } else if(rD>mD && rD>lD) {
        mHead = rHead;
    }


    //Pitch
    float lPitch = mPitch-5;
    if(!fetcher->get(lFrame, targetFrame.size(), mLat, mLon, mHead, lPitch, searchPath)) {
        ui->text_log->appendPlainText("Frame missing!");
        return false;
    }
    lD = tracker->featureMatch(lFrame, featureRes, &inline_matched_useless);

    float rPitch = mPitch+5;
    if(!fetcher->get(rFrame, targetFrame.size(), mLat, mLon, mHead, rPitch, searchPath)) {
        ui->text_log->appendPlainText("Frame missing!");
        return false;
    }
    rD = tracker->featureMatch(rFrame, featureRes, &inline_matched_useless);
    for(int d=0; d<MATCH_STEP_L; d++)
    {
        mPitch = lPitch + (rPitch-lPitch) / 2;
        if(!fetcher->get(curFrame, targetFrame.size(), mLat, mLon, mHead, mPitch, searchPath)) {
            ui->text_log->appendPlainText("Frame missing!");
            return false;
        }
        mD = tracker->featureMatch(curFrame, featureRes, &inline_matched_useless);
        if(lD > rD)
        {
            rPitch = mPitch;
            rD = mD;
        } else {
            lPitch = mPitch;
            lD = mD;
        }
    }
    if(lD>mD && lD>rD) {
        mPitch = lPitch;
    } else if(rD>mD && rD>lD) {
        mPitch = rPitch;
    }

    if(!fetcher->get(matchedFrame, targetFrame.size(), mLat, mLon, mHead, mPitch, searchPath)) {
        ui->text_log->appendPlainText("Frame missing!");
        return false;
    }
    if(showProcess) {
        namedWindow("Matched Frame", WINDOW_NORMAL);
        imshow("Matched Frame", matchedFrame);
    }
    tracker->featureMatch(matchedFrame, featureRes, &inline_matched_useless, showProcess, "Feature Match");
//    waitKey();
    //***********************************************search for most similar image***************************************************************


    ui->text_log->appendPlainText("Result:");
    ui->text_log->appendPlainText(QString("Latitude: ") + QString::number(mLat)
                                  + QString(" Longitude: ") + QString::number(mLon)
                                  + QString(" Heading: ") + QString::number(mHead)
                                  + QString(" Pitch: ") + QString::number(mPitch));


    //feature match results
    Mat toCompare = targetFrame.clone();
    detector->detectAndShow(matchedFrame, toCompare, featureRes);
    if(showProcess) {
        namedWindow("Feature based result", WINDOW_NORMAL);
        imshow("Feature based result", toCompare);
        tracker->showDifference(matchedFrame, targetFrame, "Feature result difference");
    } else {
        tracker->showDifference(matchedFrame, targetFrame, "");
    }

//    cout<<"Feature match homography:"<<endl<<featureRes<<endl;
    return true;
}

void MainWindow::pixelRefine() {
    Mat finalHomo = tracker->pixelMatch(matchedFrame);
//    cout<<"final home: "<<endl<<finalHomo<<endl;

    Mat finalmatchedFrame;
    warpPerspective(matchedFrame, finalmatchedFrame, finalHomo, matchedFrame.size());
    if(showProcess) {
        tracker->showDifference(finalmatchedFrame, targetFrame, "Final Difference");
    } else {
        tracker->showDifference(finalmatchedFrame, targetFrame, "");
    }

    //detect lane and show final result
    Mat projImg = targetFrame.clone();
    detector->detectAndShow(matchedFrame, projImg, finalHomo);
    if(showProcess) {
        namedWindow("Final result", WINDOW_NORMAL);
        imshow("Final result", projImg);
    } else {
        string writeName =  "/home/kimiwings/resImgs/"+targetString+"jjjjjsss.jpg";             //TODO
        imwrite(writeName, projImg);
    }
}

void MainWindow::process()
{
    if(!findBestMatch()) {
        return;
    }

    pixelRefine();
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    tracker = new Tracker();
    detector = new LaneDetector();
    fetcher = new IMGFetcher();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_button_img_clicked()
{
    showProcess = true;
    QString Qfile1Name = QFileDialog::getOpenFileName(this, tr("Open Img File"), "/home/kimiwings/SafeDrive/test/test0.jpg", tr("Img File (*.jpg)"));
    targetString = Qfile1Name.toStdString();


    QString QPosFile = QFileDialog::getOpenFileName(this, tr("Open Pos File"), "/home/kimiwings/SafeDrive/test/test0.log", tr("log Files (*.log)"));
    string posFile = QPosFile.toStdString();
    ifstream params(posFile);

    targetFrame = imread(targetString);
    if( targetFrame.empty() ) {
      ui->text_log->appendPlainText("Error occured, image not read correctly");
      return;
    }

    namedWindow("Target Frame", WINDOW_NORMAL);
    imshow("Target Frame", targetFrame);

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

void MainWindow::on_button_video_clicked()
{
    showProcess = false;
    QString QImgFolderName = QFileDialog::getExistingDirectory(this, tr("Open Img Directory"), "/home/kimiwings/SafeDrive/test/video/imgs", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    string imgFolderName = QImgFolderName.toStdString();
    vector<String> imgNames;
    glob(imgFolderName, imgNames);

    QString QPosFile = QFileDialog::getOpenFileName(this, tr("Open Pos File"), "/home/kimiwings/SafeDrive/test/video/pos.log", tr("log Files (*.log)"));
    string posFile = QPosFile.toStdString();
    ifstream infile(posFile);

    for(size_t i=0;i<imgNames.size(); i++) {
        string posName;
        getline(infile, posName);
        cout<<"Img name: "<<imgNames[i]<<endl;
        cout<<"Pos name: "<<posName<<endl;
        targetFrame = imread(imgNames[i]);
        if( targetFrame.empty() ) {
          ui->text_log->appendPlainText("Error occured, image not read correctly");
          return;
        }

        istringstream params(posName);
        string param;
        params >> param;
        lat = stof(param);
        params >> param;
        lon = stof(param);
        params >> param;
        head = stof(param);
        params >> param;
        pitch = stof(param);

        on_button_reset_clicked();
    }
}

void MainWindow::on_button_reset_clicked()
{
    ui->slider_BS->setValue(BS);
    ui->slider_BV->setValue(BV);
    ui->slider_MNF->setValue(MNF);
    ui->slider_QL->setValue(QL);
    ui->slider_MD->setValue(MD);
    ui->slider_NGF->setValue(NGF);
    ui->slider_MTF->setValue(MTF);
    ui->slider_RTF->setValue(RTF);
    ui->slider_BDS->setValue(BDS);
    ui->slider_PG->setValue(PG);
    ui->slider_NGP->setValue(NGP);
    ui->slider_MTP->setValue(MTP);
    ui->slider_RTP->setValue(RTP);
    changeParamAndReprocess(true);
}

void MainWindow::changeParamAndReprocess(bool reFind)
{
    ui->label_BS->setText(QString("Blur Size: ") + QString::number(2*ui->slider_BS->value() + 1));
    ui->label_BV->setText(QString("Blur Var: ") + QString::number(ui->slider_BV->value() / 10.0f));
    ui->label_MNF->setText(QString("Max Num Features: ") + QString::number(ui->slider_MNF->value()));
    ui->label_QL->setText(QString("Quality Level: ") + QString::number(ui->slider_QL->value() / 100.0f));
    ui->label_MD->setText(QString("Min Distance: ") + QString::number(ui->slider_MD->value()));
    ui->label_NGF->setText(QString("Num Grid Feature: ") + QString::number(ui->slider_NGF->value()));
    ui->label_MTF->setText(QString("Match Thres Feature: ") + QString::number(ui->slider_MTF->value() / 100.0f));
    ui->label_RTF->setText(QString("RANSAC Thres Feature: ") + QString::number(ui->slider_RTF->value() / 1.0f));
    ui->label_BDS->setText(QString("Board Size: ") + QString::number(2*ui->slider_BDS->value() + 1));
    ui->label_PG->setText(QString("Point Grid: ") + QString::number(ui->slider_PG->value()));
    ui->label_NGP->setText(QString("Num Grid Pixel: ") + QString::number(ui->slider_NGP->value()));
    ui->label_MTP->setText(QString("Match Thres Pixel: ") + QString::number(ui->slider_MTP->value() / 100.0f));
    ui->label_RTP->setText(QString("RANSAC Thres Pixel: ") + QString::number(ui->slider_RTP->value() / 1.0f));
    int bs = ui->slider_BS->value() * 2 + 1;
    float bv = ui->slider_BV->value() / 10.0f;
    int mnf = ui->slider_MNF->value();
    float ql = ui->slider_QL->value() / 100.0f;
    int md = ui->slider_MD->value();
    int ngf = ui->slider_NGF->value();
    float mtf = ui->slider_MTF->value() / 100.0f;
    float rtf = ui->slider_RTF->value() / 1.0f;
    float bds = ui->slider_BDS->value() * 2 + 1;
    int pg = ui->slider_PG->value();
    int ngp = ui->slider_NGP->value();
    float mtp = ui->slider_MTP->value() / 100.f;
    float rtg = ui->slider_RTP->value() / 1.0f;
    //(int bs, float bv, int mnf, float ql, int md,  int ngf, float mtf, float rtf, int bds, int pg, int ngp, float mtp, float rtp)
    tracker->changeParam(bs, bv, mnf, ql, md, ngf, mtf, rtf, bds, pg, ngp, mtp, rtg);

    if(reFind) {
        process();
    } else {
        pixelRefine();
    }
}

void MainWindow::on_slider_BS_sliderReleased()
{
    changeParamAndReprocess(true);
}

void MainWindow::on_slider_BV_sliderReleased()
{
    changeParamAndReprocess(true);
}

void MainWindow::on_slider_MNF_sliderReleased()
{
    changeParamAndReprocess(true);
}

void MainWindow::on_slider_QL_sliderReleased()
{
    changeParamAndReprocess(true);
}

void MainWindow::on_slider_MD_sliderReleased()
{
    changeParamAndReprocess(true);
}

void MainWindow::on_slider_NGF_sliderReleased()
{
    changeParamAndReprocess(true);
}

void MainWindow::on_slider_MTF_sliderReleased()
{
    changeParamAndReprocess(true);
}


void MainWindow::on_slider_RTF_sliderReleased()
{
    changeParamAndReprocess(true);
}

void MainWindow::on_slider_BDS_sliderReleased()
{
    changeParamAndReprocess(false);
}

void MainWindow::on_slider_PG_sliderReleased()
{
    changeParamAndReprocess(false);
}

void MainWindow::on_slider_NGP_sliderReleased()
{
    changeParamAndReprocess(false);
}

void MainWindow::on_slider_MTP_sliderReleased()
{
    changeParamAndReprocess(false);
}

void MainWindow::on_slider_RTP_sliderReleased()
{
    changeParamAndReprocess(false);
}


void MainWindow::on_check_local_clicked(bool checked)
{
    if(checked) {
        QString QlocalSearchFolderName = QFileDialog::getExistingDirectory(this, tr("Open Img Directory"), "/home/kimiwings/SafeDrive/test/video/imgs", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
        searchPath = QlocalSearchFolderName.toStdString();
    } else {
        searchPath = "";
    }
}
