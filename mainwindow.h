#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "tracker.hpp"
#include "lane_detector.hpp"
#include "gsv_fetcher.hpp"
#include <string.h>
#include <fstream>

#define BS 3
#define BV 4
#define MNF 1000
#define QL 10
#define MD 5
#define NMT 74
#define RT 7
#define BDS 10

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

    string targetString;
    float lat;
    float lon;
    float head;
    float pitch;
    void changeParamAndReprocess();
    void process();

private slots:
    void on_button_reset_clicked();
    void on_button_start_clicked();
    void on_slider_MNF_sliderReleased();
    void on_slider_QL_sliderReleased();
    void on_slider_MD_sliderReleased();
    void on_slider_NMT_sliderReleased();
    void on_slider_RT_sliderReleased();
    void on_slider_BS_sliderReleased();
    void on_slider_BV_sliderReleased();
    void on_slider_BDT_sliderReleased();
    void on_slider_BDS_sliderReleased();
};

#endif // MAINWINDOW_H
