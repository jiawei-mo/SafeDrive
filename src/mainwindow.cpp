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
    QString Qfile1Name = QFileDialog::getOpenFileName(this, tr("Target image"), "/home/kimiwings/workspace/SafeDrive/test/3.jpg", tr("Img File (*.jpg)"));
    string targetString = Qfile1Name.toStdString();

    QString QlocalSearchFolderName = QFileDialog::getExistingDirectory(this, tr("Open Database Directory"), "/home/kimiwings/workspace/SafeDrive/test/database", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    string searchPath = QlocalSearchFolderName.toStdString();

    QString QPosFile = QFileDialog::getOpenFileName(this, tr("Open position File"), "/home/kimiwings/workspace/SafeDrive/test/back_up/test1.log", tr("log Files (*.log)"));
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

        on_button_reset_clicked();
    }
}

void MainWindow::on_button_reset_clicked()
{
    ui->slider_MNF->setValue(MNF);
    ui->slider_QL->setValue(QL);
    ui->slider_MD->setValue(MD);
    ui->slider_MTF->setValue(MTF);
    ui->slider_RTF->setValue(RTF);
    changeParamAndReprocess();
}

void MainWindow::changeParamAndReprocess()
{
    ui->label_MNF->setText(QString("Max Num Features: ") + QString::number(ui->slider_MNF->value()));
    ui->label_QL->setText(QString("Quality Level: ") + QString::number(ui->slider_QL->value() / 100.0f));
    ui->label_MD->setText(QString("Min Distance: ") + QString::number(ui->slider_MD->value()));
    ui->label_MTF->setText(QString("Match Thres: ") + QString::number(ui->slider_MTF->value() / 100.0f));
    ui->label_RTF->setText(QString("RANSAC Thres: ") + QString::number(ui->slider_RTF->value() / 10.0f));

    int mnf = ui->slider_MNF->value();
    float ql = ui->slider_QL->value() / 100.0f;
    int md = ui->slider_MD->value();
    float mtf = ui->slider_MTF->value() / 100.0f;
    float rtf = ui->slider_RTF->value() / 10.0f;
    manager->changeParam(mnf, ql, md, mtf, rtf);

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

void MainWindow::on_slider_RTF_sliderReleased()
{
    changeParamAndReprocess();
}

