#include "omp.h"

#include "headers/mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    manager = shared_ptr<Manager>(new Manager());
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_button_img_clicked()
{
    QString Qfile1Name = QFileDialog::getOpenFileName(this, tr("Open Img File"), "/home/kimiwings/SafeDrive/test/3b.jpg", tr("Img File (*.jpg)"));
    string targetString = Qfile1Name.toStdString();

    QString QlocalSearchFolderName = QFileDialog::getExistingDirectory(this, tr("Open Img Directory"), "/home/kimiwings/SafeDrive/test/video/imgs", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    string searchPath = QlocalSearchFolderName.toStdString();

    QString QPosFile = QFileDialog::getOpenFileName(this, tr("Open Pos File"), "/home/kimiwings/SafeDrive/test/old/test1.log", tr("log Files (*.log)"));
    string posFile = QPosFile.toStdString();
    ifstream params(posFile);

    string param;
    params >> param;
    float lat = stof(param);
    params >> param;
    float lon = stof(param);
    params >> param;
    float head = stof(param);
    params >> param;
    float pitch = stof(param);
    params.close();

    manager->initialize(targetString, lat, lon, head, pitch, searchPath);

    on_button_reset_clicked();
}

void MainWindow::on_button_video_clicked()
{
    QString QImgFolderName = QFileDialog::getExistingDirectory(this, tr("Open Img Directory"), "/home/kimiwings/SafeDrive/test/video/imgs", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    string imgFolderName = QImgFolderName.toStdString();
    vector<String> imgNames;
    glob(imgFolderName, imgNames);

    QString QlocalSearchFolderName = QFileDialog::getExistingDirectory(this, tr("Open Img Directory"), "/home/kimiwings/SafeDrive/test/video/imgs", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    string searchPath = QlocalSearchFolderName.toStdString();

    QString QPosFile = QFileDialog::getOpenFileName(this, tr("Open Pos File"), "/home/kimiwings/SafeDrive/test/video/pos.log", tr("log Files (*.log)"));
    string posFile = QPosFile.toStdString();
    ifstream infile(posFile);

    for(size_t i=0;i<imgNames.size(); i++) {
        string posName;
        getline(infile, posName);
        cout<<"Img name: "<<imgNames[i]<<endl;
        cout<<"Pos name: "<<posName<<endl;

        istringstream params(posName);
        string param;
        params >> param;
        float lat = stof(param);
        params >> param;
        float lon = stof(param);
        params >> param;
        float head = stof(param);
        params >> param;
        float pitch = stof(param);

        manager->initialize(imgNames[i], lat, lon, head, pitch, searchPath);

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
    manager->changeParam(bs, bv, mnf, ql, md, ngf, mtf, rtf, sws, nd, pfc, mod, ur, sw, sr, dmd, s1, s2);

    manager->process();
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

