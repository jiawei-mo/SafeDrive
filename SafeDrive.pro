#-------------------------------------------------
#
# Project created by QtCreator 2016-07-01T16:52:32
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = SafeDrive
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    gsv_fetcher.cpp \
    tracker.cpp \
    lane_detector.cpp

HEADERS  += mainwindow.h \
    tracker.hpp \
    gsv_fetcher.hpp \
    lane_detector.hpp \
    parameters.hpp

FORMS    += mainwindow.ui

INCLUDEPATH += /usr/local/include/opencv

LIBS += -L/usr/local/lib
LIBS += -lopencv_core
LIBS += -lopencv_imgproc
LIBS += -lopencv_highgui
LIBS += -lopencv_videoio
LIBS += -lopencv_features2d
LIBS += -lopencv_calib3d
LIBS += -lopencv_imgcodecs
LIBS += -lopencv_reg

CONFIG += c++11
