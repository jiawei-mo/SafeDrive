#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include "reconstructor.hpp"
#include "matcher.hpp"
#include "lane_detector.hpp"
#include "img_fetcher.hpp"
#include <string.h>
#include <fstream>
#include <sstream>
#include <memory>

#include "parameters.hpp"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    bool showProcess;
    string saveFolder;

    Ui::MainWindow *ui;

    shared_ptr<Reconstructor> reconstructor;
    shared_ptr<Matcher> matcher;
    shared_ptr<LaneDetector> lane_detector;
    shared_ptr<IMGFetcher> fetcher;

    Mat camera_K;
    Mat coeff;
    Mat targetFrame;
    Mat matchedFrame;
    vector<Point2f> targetKp;
    vector<Point2f> matchedKp;
    float lat;
    float lon;
    float head;
    float pitch;
    string searchPath;
    void changeParamAndReprocess();
    void process();
    bool findBestMatch();

private slots:
    void on_button_reset_clicked();
    void on_button_img_clicked();
    void on_button_video_clicked();

    void on_slider_BS_sliderReleased();
    void on_slider_BV_sliderReleased();
    void on_slider_MNF_sliderReleased();
    void on_slider_QL_sliderReleased();
    void on_slider_MD_sliderReleased();
    void on_slider_NGF_sliderReleased();
    void on_slider_MTF_sliderReleased();
    void on_slider_RTF_sliderReleased();
    void on_slider_SWS_sliderReleased();
    void on_slider_ND_sliderReleased();
    void on_slider_PFC_sliderReleased();
    void on_slider_MOD_sliderReleased();
    void on_slider_UR_sliderReleased();
    void on_slider_SW_sliderReleased();
    void on_slider_SR_sliderReleased();
    void on_slider_DMD_sliderReleased();
    void on_slider_S1_sliderReleased();
    void on_slider_S2_sliderReleased();
    void on_check_local_clicked(bool checked);
};

#endif // MAINWINDOW_H
