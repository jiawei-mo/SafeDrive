#include "omp.h"

#include "headers/mainwindow.h"
#include "ui_mainwindow.h"

bool DEBUG = false;
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
    destroyAllWindows();
    QString QFolderName = QFileDialog::getExistingDirectory(this, tr("Test folder"), "/home/kimiwings/workspace/SafeDrive/test", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    string folderName = QFolderName.toStdString();

    if(folderName.empty()) {
        return;
    }

    string defaultFileName = folderName+"/imgs/01.jpg";
    QString Qfile1Name = QFileDialog::getOpenFileName(this, tr("Target image"), tr(defaultFileName.c_str()), tr("Img File (*.*)"));
    string targetString = Qfile1Name.toStdString();

    if(targetString.empty()) {
        return;
    }

    string camFile = folderName+"/cam.log";
    ifstream camParams(camFile);

    string camParam;
    camParams >> camParam;
    float kx = stof(camParam);
    camParams >> camParam;
    float ky = stof(camParam);
    camParams >> camParam;
    float cx = stof(camParam);
    camParams >> camParam;
    float cy = stof(camParam);
    camParams >> camParam;
    float d1 = stof(camParam);
    camParams >> camParam;
    float d2 = stof(camParam);
    camParams >> camParam;
    float d3 = stof(camParam);
    camParams >> camParam;
    float d4 = stof(camParam);
    camParams >> camParam;
    float d5 = stof(camParam);
    vector<float> K={kx,ky,cx,cy};
    vector<float> D={d1,d2,d3,d4,d5};
    camParams.close();

    string posFile = targetString;
    int dotP = posFile.size()-1;
    while(posFile[dotP]!='.') dotP--;
    int slashP = dotP;
    while(posFile[slashP]!='/') slashP--;
    posFile = folderName+"/pos"+posFile.substr(slashP, dotP-slashP)+".log";
    ifstream posParams(posFile);

    string posParam;
    posParams >> posParam;
    float lat = stof(posParam);
    posParams >> posParam;
    float lon = stof(posParam);
    posParams >> posParam;
    float head = stof(posParam);
    posParams >> posParam;
    float pitch = stof(posParam);
    posParams.close();


    cout<<"Img name: "<<targetString<<endl;
    cout<<"Pos name: "<<posFile<<endl;

    string searchPath = folderName+"/database/";
    searchPath += (to_string(int(round(lat)))+"_"+to_string(int(round(lon)))+"_"+to_string(int(round(head)))+"_"+to_string(int(round(pitch))));
cout<<searchPath<<endl;
//return;
    manager->initialize(targetString, K, D, lat, lon, head, pitch, searchPath);

    if(!initialied) {
        on_button_reset_clicked();
    } else {
        manager->process();
    }
}

void MainWindow::on_button_video_clicked()
{
    destroyAllWindows();
    QString QFolderName = QFileDialog::getExistingDirectory(this, tr("Test folder"), "/home/kimiwings/workspace/SafeDrive/test", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    string folderName = QFolderName.toStdString();

    if(folderName.empty()) {
        return;
    }

    string defaultImgFolderName = folderName+"/imgs";
    QString QImgFolderName = QFileDialog::getExistingDirectory(this, tr("Open target Directory"), tr(defaultImgFolderName.c_str()), QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    string imgFolderName = QImgFolderName.toStdString();

    if(imgFolderName.empty()) {
        return;
    }

    vector<String> imgNames;
    glob(imgFolderName, imgNames);

    string camFile = folderName+"/cam.log";
    ifstream camParams(camFile);

    string camParam;
    camParams >> camParam;
    float kx = stof(camParam);
    camParams >> camParam;
    float ky = stof(camParam);
    camParams >> camParam;
    float cx = stof(camParam);
    camParams >> camParam;
    float cy = stof(camParam);
    camParams >> camParam;
    float d1 = stof(camParam);
    camParams >> camParam;
    float d2 = stof(camParam);
    camParams >> camParam;
    float d3 = stof(camParam);
    camParams >> camParam;
    float d4 = stof(camParam);
    camParams >> camParam;
    float d5 = stof(camParam);
    vector<float> K={kx,ky,cx,cy};
    vector<float> D={d1,d2,d3,d4,d5};
    camParams.close();

    for(size_t i=0;i<imgNames.size(); i++) {
        string posName = imgNames[i];
        int dotP = posName.size()-1;
        while(posName[dotP]!='.') dotP--;
        int slashP = dotP;
        while(posName[slashP]!='/') slashP--;
        posName = folderName+"/pos"+posName.substr(slashP, dotP-slashP)+".log";
        ifstream posParams(posName);

        string posParam;
        posParams >> posParam;
        float lat = stof(posParam);
        posParams >> posParam;
        float lon = stof(posParam);
        posParams >> posParam;
        float head = stof(posParam);
        posParams >> posParam;
        float pitch = stof(posParam);
        posParams.close();


        cout<<"Img name: "<<imgNames[i]<<endl;
        cout<<"Pos name: "<<posName<<endl;

        string searchPath = folderName+"/database/";
        searchPath += (to_string(int(round(lat)))+"_"+to_string(int(round(lon)))+"_"+to_string(int(round(head)))+"_"+to_string(int(round(pitch))));

        manager->initialize(imgNames[i], K, D, lat, lon, head, pitch, searchPath);

        if(!initialied) {
            on_button_reset_clicked();
        } else {
            manager->process();
        }

        waitKey();
    }
    destroyAllWindows();
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

void MainWindow::on_check_DEBUG_clicked(bool checked)
{
    DEBUG = checked;
    destroyAllWindows();
    changeParamAndReprocess();
}
