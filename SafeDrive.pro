#-------------------------------------------------
#
# Project created by QtCreator 2016-07-01T16:52:32
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = SafeDrive
TEMPLATE = app


SOURCES += src/main.cpp\
        src/mainwindow.cpp \
        src/lane_detector.cpp \
        src/img_fetcher.cpp \
        src/reconstructor.cpp \
        src/matcher.cpp

HEADERS  += headers/mainwindow.h \
            headers/lane_detector.hpp \
            headers/parameters.hpp \
            headers/img_fetcher.hpp \
            headers/reconstructor.hpp \
            headers/matcher.hpp

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
