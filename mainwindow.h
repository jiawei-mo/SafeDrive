#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "tracker.hpp"
#include "lane_detector.hpp"
#include "gsv_fetcher.hpp"
#include "parameters.hpp"
#include <string.h>
#include <fstream>

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
    Ui::MainWindow *ui;

    Tracker *tracker;
    LaneDetector *detector;
    GSVFetcher *fetcher;

    cv::Mat matchImg;
    cv::Mat laneImg;
    cv::Mat resultImg;

    string targetName;
    float lat;
    float lon;
    float head;
    void changeParamAndReprocess();
    void process(string targetName, float lat, float lon, float heading, float pitch);

private slots:
    void on_button_reset_clicked();
//    void on_slider_MNF_valueChanged(int mnf);
//    void on_slider_QL_valueChanged(int qli);
//    void on_slider_MD_valueChanged(int md);
//    void on_slider_NMT_valueChanged(int nmti);
//    void on_slider_NMN_valueChanged(int nmn);
//    void on_slider_RT_valueChanged(int rti);
    void on_button_start_clicked();
    void on_slider_MNF_sliderReleased();
    void on_slider_QL_sliderReleased();
    void on_slider_MD_sliderReleased();
    void on_slider_NMT_sliderReleased();
    void on_slider_NMN_sliderReleased();
    void on_slider_RT_sliderReleased();
};

#endif // MAINWINDOW_H
