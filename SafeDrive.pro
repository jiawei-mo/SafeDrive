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
    lane_detector.cpp \
    img_fetcher.cpp \
    reconstructor.cpp \
    matcher.cpp

HEADERS  += mainwindow.h \
    lane_detector.hpp \
    parameters.hpp \
    img_fetcher.hpp \
    reconstructor.hpp \
    matcher.hpp

FORMS    += mainwindow.ui

INCLUDEPATH += /usr/local/include/opencv
INCLUDEPATH += /usr/include/pcl-1.7
INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += /usr/include/vtk-6.2

LIBS += -L/usr/local/lib/
LIBS += -lopencv_core
LIBS += -lopencv_imgproc
LIBS += -lopencv_highgui
LIBS += -lopencv_videoio
LIBS += -lopencv_features2d
LIBS += -lopencv_calib3d
LIBS += -lopencv_imgcodecs
LIBS += -lboost_system
LIBS += -lpcl_common
LIBS += -lpcl_visualization
LIBS += -lpcl_io


CONFIG += -std=c++11
QMAKE_CXXFLAGS += -fopenmp
LIBS += -fopenmp

DISTFILES += \
    CMakeList.txt
