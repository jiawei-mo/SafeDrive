#include "omp.h"

#include "headers/mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    manager = shared_ptr<Manager>(new Manager());
    initialied = false;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_button_img_clicked()
{
    QString Qfile1Name = QFileDialog::getOpenFileName(this, tr("Target image"), "/home/kimiwings/workspace/SafeDrive/test/3.jpg", tr("Img File (*.jpg)"));
    string targetString = Qfile1Name.toStdString();

    if(targetString.empty()) {
        return;
    }

    QString QlocalSearchFolderName = QFileDialog::getExistingDirectory(this, tr("Open Database Directory"), "/home/kimiwings/workspace/SafeDrive/test/database", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    string searchPath = QlocalSearchFolderName.toStdString();

    if(searchPath.empty()) {
        return;
    }

    QString QPosFile = QFileDialog::getOpenFileName(this, tr("Open position File"), "/home/kimiwings/workspace/SafeDrive/test/back_up/test1.log", tr("log Files (*.log)"));
    string posFile = QPosFile.toStdString();

    if(posFile.empty()) {
        return;
    }

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

    if(!initialied) {
        on_button_reset_clicked();
    } else {
        manager->process();
    }
}

void MainWindow::on_button_video_clicked()
{
    QString QImgFolderName = QFileDialog::getExistingDirectory(this, tr("Open Img Directory"), "/home/kimiwings/workspace/SafeDrive/test/video/imgs", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    string imgFolderName = QImgFolderName.toStdString();
    vector<String> imgNames;
    glob(imgFolderName, imgNames);

    QString QlocalSearchFolderName = QFileDialog::getExistingDirectory(this, tr("Open Img Directory"), "/home/kimiwings/workspace/SafeDrive/test/video/imgs", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    string searchPath = QlocalSearchFolderName.toStdString();

    QString QPosFile = QFileDialog::getOpenFileName(this, tr("Open Pos File"), "/home/kimiwings/workspace/SafeDrive/test/video/pos.log", tr("log Files (*.log)"));
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

        if(!initialied) {
            on_button_reset_clicked();
        } else {
            manager->process();
        }
    }
}

void MainWindow::on_button_reset_clicked()
{
    ui->slider_MNF->setValue(MNF);
    ui->slider_QL->setValue(QL);
    ui->slider_MD->setValue(MD);
    ui->slider_MTF->setValue(MTF);
    ui->slider_MTG->setValue(MTG);
    ui->slider_RTE->setValue(RTE);
    ui->slider_RTP->setValue(RTP);
    initialied = true;
    changeParamAndReprocess();
}

void MainWindow::changeParamAndReprocess()
{
    ui->label_MNF->setText(QString("Max Num Features: ") + QString::number(ui->slider_MNF->value()));
    ui->label_QL->setText(QString("Quality Level: ") + QString::number(ui->slider_QL->value() / 100.0f));
    ui->label_MD->setText(QString("Min Distance: ") + QString::number(ui->slider_MD->value()));
    ui->label_MTF->setText(QString("Match Thres: ") + QString::number(ui->slider_MTF->value() / 100.0f));
    ui->label_MTG->setText(QString("Match Thres KP: ") + QString::number(ui->slider_MTG->value() / 100.0f));
    ui->label_RTE->setText(QString("RANSAC Essential: ") + QString::number(ui->slider_RTE->value() / 10.0f));
    ui->label_RTP->setText(QString("RANSAC PnP: ") + QString::number(ui->slider_RTP->value() / 10.0f));

    int mnf = ui->slider_MNF->value();
    float ql = ui->slider_QL->value() / 100.0f;
    int md = ui->slider_MD->value();
    float mtf = ui->slider_MTF->value() / 100.0f;
    float mtg = ui->slider_MTG->value() / 100.0f;
    float rte = ui->slider_RTE->value() / 10.0f;
    float rtp = ui->slider_RTP->value() / 10.0f;
    manager->changeParam(mnf, ql, md, mtf, mtg, rte, rtp);

    manager->process();
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

void MainWindow::on_slider_MTF_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_RTE_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_RTP_sliderReleased()
{
    changeParamAndReprocess();
}

void MainWindow::on_slider_MTG_sliderReleased()
{
    changeParamAndReprocess();
}
