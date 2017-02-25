#include "omp.h"

#include "mainwindow.h"
#include "ui_mainwindow.h"

bool MainWindow::findBestMatch()
{
    matcher->setTarget(targetFrame);

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
        Mat coeff = (Mat_<double>(1,5) << -0.2004, 0.1620, 0, 0, 0);
    matchedFrame = imread("/home/kimiwings/SafeDrive/test/DSC_0001.JPG");
    matcher->match(matchedFrame, targetMatchedKp, matchedKp, showProcess, "Feature Match");
//    waitKey();
    //***********************************************search for most similar image***************************************************************


    ui->text_log->appendPlainText("Result:");
//    ui->text_log->appendPlainText(QString("Latitude: ") + QString::number(mLat)
//                                  + QString(" Longitude: ") + QString::number(mLon)
//                                  + QString(" Heading: ") + QString::number(mHead)
//                                  + QString(" Pitch: ") + QString::number(mPitch));


    //feature match results
    Mat toCompare = targetFrame.clone();
    lane_detector->detectAndShow(matchedFrame, toCompare);

    if(showProcess) {
        namedWindow("Feature based result", WINDOW_NORMAL);
        imshow("Feature based result", toCompare);
        matcher->showDifference(matchedFrame, targetFrame, "Feature result difference");
    } else {
        matcher->showDifference(matchedFrame, targetFrame, "");
    }

//    cout<<"Feature match homography:"<<endl<<featureRes<<endl;
    return true;
}

void MainWindow::process()
{
    if(!findBestMatch()) {
        return;
    }
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    reconstructor = shared_ptr<Reconstructor>(new Reconstructor());
    matcher = shared_ptr<Matcher>(new Matcher());
    lane_detector = shared_ptr<LaneDetector>(new LaneDetector());
    fetcher = shared_ptr<IMGFetcher>(new IMGFetcher());

    saveFolder = "";
    camera_K = (Mat_<double>(3,3) << 1623.4, 0, 1081.9,
                                     0, 1623.7, 709.9,
                                     0, 0, 1);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_button_img_clicked()
{
    showProcess = true;
    QString Qfile1Name = QFileDialog::getOpenFileName(this, tr("Open Img File"), "/home/kimiwings/SafeDrive/test/DSC_0002.JPG", tr("Img File (*.jpg)"));
    string targetString = Qfile1Name.toStdString();


    QString QPosFile = QFileDialog::getOpenFileName(this, tr("Open Pos File"), "/home/kimiwings/SafeDrive/test/test0.log", tr("log Files (*.log)"));
    string posFile = QPosFile.toStdString();
    ifstream params(posFile);

    targetFrame = imread(targetString);
    if( targetFrame.empty() ) {
      ui->text_log->appendPlainText("Error ocdbed, image not read correctly");
      return;
    }

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

    QString QSaveFolderName = QFileDialog::getExistingDirectory(this, tr("Open Save Directory"), "/home/kimiwings/SafeDrive/test/video/res", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    saveFolder = QSaveFolderName.toStdString();

    for(size_t i=0;i<imgNames.size(); i++) {
        string posName;
        getline(infile, posName);
        cout<<"Img name: "<<imgNames[i]<<endl;
        cout<<"Pos name: "<<posName<<endl;
        targetFrame = imread(imgNames[i]);
        if( targetFrame.empty() ) {
          ui->text_log->appendPlainText("Error ocdbed, image not read correctly");
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
    ui->slider_SWS->setValue(SWS);
    ui->slider_ND->setValue(ND);
    ui->slider_PFC->setValue(PFC);
    ui->slider_MOD->setValue(MOD);
    ui->slider_UR->setValue(UR);
    ui->slider_SW->setValue(SW);
    ui->slider_SR->setValue(SR);
    ui->slider_DMD->setValue(DMD);
    ui->slider_S1->setValue(S1);
    ui->slider_S2->setValue(S2);
    changeParamAndReprocess();
}

void MainWindow::changeParamAndReprocess()
{
    ui->label_BS->setText(QString("Blur Size: ") + QString::number(2*ui->slider_BS->value() + 1));
    ui->label_BV->setText(QString("Blur Var: ") + QString::number(ui->slider_BV->value() / 10.0f));
    ui->label_MNF->setText(QString("Max Num Features: ") + QString::number(ui->slider_MNF->value()));
    ui->label_QL->setText(QString("Quality Level: ") + QString::number(ui->slider_QL->value() / 100.0f));
    ui->label_MD->setText(QString("Min Distance: ") + QString::number(ui->slider_MD->value()));
    ui->label_NGF->setText(QString("Num Grid Feature: ") + QString::number(ui->slider_NGF->value()));
    ui->label_MTF->setText(QString("Match Thres Feature: ") + QString::number(ui->slider_MTF->value() / 100.0f));
    ui->label_RTF->setText(QString("RANSAC Thres Feature: ") + QString::number(ui->slider_RTF->value() / 1.0f));
    ui->label_SWS->setText(QString("SADWindowSize: ") + QString::number(ui->slider_SWS->value()));
    ui->label_ND->setText(QString("numberOfDisparities: ") + QString::number(ui->slider_ND->value() * 16));
    ui->label_PFC->setText(QString("preFilterCap: ") + QString::number(ui->slider_PFC->value()));
    ui->label_MOD->setText(QString("minDisparity: ") + QString::number(ui->slider_MOD->value()));
    ui->label_UR->setText(QString("uniquenessRatio: ") + QString::number(ui->slider_UR->value()));
    ui->label_SW->setText(QString("speckleWindowSize: ") + QString::number(ui->slider_SW->value()));
    ui->label_SR->setText(QString("speckleRange: ") + QString::number(ui->slider_SR->value()));
    ui->label_DMD->setText(QString("disp12MaxDiff: ") + QString::number(ui->slider_DMD->value()));
    ui->label_S1->setText(QString("P1: ") + QString::number(ui->slider_S1->value()));
    ui->label_S2->setText(QString("P2: ") + QString::number(ui->slider_S2->value()));
    int bs = ui->slider_BS->value() * 2 + 1;
    float bv = ui->slider_BV->value() / 10.0f;
    int mnf = ui->slider_MNF->value();
    float ql = ui->slider_QL->value() / 100.0f;
    int md = ui->slider_MD->value();
    int ngf = ui->slider_NGF->value();
    float mtf = ui->slider_MTF->value() / 100.0f;
    float rtf = ui->slider_RTF->value() / 1.0f;
    int sws = ui->slider_SWS->value();
    int nd = ui->slider_ND->value() * 16;
    int pfc = ui->slider_PFC->value();
    int mod = ui->slider_MOD->value();
    int ur = ui->slider_UR->value();
    int sw = ui->slider_SW->value();
    int sr = ui->slider_SR->value();
    int dmd = ui->slider_DMD->value();
    int s1 = ui->slider_S1->value();
    int s2 = ui->slider_S2->value();
    matcher->changeParam(bs, bv, mnf, ql, md, ngf, mtf);
    reconstructor->changeParam(rtf, sws, nd, pfc, mod, ur, sw, sr, dmd, s1, s2);

    process();
}

void MainWindow::on_slider_BS_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_BV_sliderReleased()
{
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

void MainWindow::on_slider_NGF_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_MTF_sliderReleased()
{
    changeParamAndReprocess();
}


void MainWindow::on_slider_RTF_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_SWS_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_ND_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_PFC_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_MOD_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_UR_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_SW_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_SR_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_DMD_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_S1_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_S2_sliderReleased()
{
    changeParamAndReprocess();
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
