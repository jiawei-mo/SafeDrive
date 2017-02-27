#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>

#include "headers/manager.hpp"

#include <fstream>
#include <sstream>

#include "parameters.hpp"

using namespace std;

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

    shared_ptr<Manager> manager;

    void changeParamAndReprocess();
    void process();

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
};

#endif // MAINWINDOW_H
