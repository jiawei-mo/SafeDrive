#include "mainwindow.h"
#include <QApplication>
using namespace std;
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}

//#include <iostream>
//#define _USE_MATH_DEFINES
//#include <cmath>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/highgui.hpp> // OpenCV window I/O
//#include <opencv2/imgproc.hpp> // OpenCV image transformations
//#include <opencv2/imgproc.hpp>
//#include <opencv2/imgproc/types_c.h>
//#include <opencv2/imgcodecs/imgcodecs_c.h>
//#include <opencv2/highgui/highgui_c.h>

//#include "opencv2/reg/mapprojec.hpp"
//#include "opencv2/reg/mappergradproj.hpp"
//#include "opencv2/reg/mapperpyramid.hpp"

//static const char* DIFF_IM = "Image difference";
//static const char* DIFF_REGPIX_IM = "Image difference: pixel registered";

//using namespace cv;
//using namespace cv::reg;
//using namespace std;

//static void showDifference(const Mat& image1, const Mat& image2, const char* title)
//{
//    Mat img1, img2;
//    image1.convertTo(img1, CV_32FC3);
//    image2.convertTo(img2, CV_32FC3);
//    if(img1.channels() != 1)
//        cvtColor(img1, img1, CV_RGB2GRAY);
//    if(img2.channels() != 1)
//        cvtColor(img2, img2, CV_RGB2GRAY);

//    Mat imgDiff;
//    img1.copyTo(imgDiff);
//    imgDiff -= img2;
//    imgDiff /= 2.f;
//    imgDiff += 128.f;

//    Mat imgSh;
//    imgDiff.convertTo(imgSh, CV_8UC3);
//    namedWindow(title, WINDOW_NORMAL);
//    imshow(title, imgSh);
//}

//void showDifferenceEdge(const Mat& image1, const Mat& image2, string title)
//{
//    Mat img1Tmp, img2Tmp, img1Edge, img2Edge;
//    image1.convertTo(img1Tmp, CV_8UC3);
//    image2.convertTo(img2Tmp, CV_8UC3);
//    Canny( img1Tmp, img1Edge, 100, 300, 3);
//    Canny( img2Tmp, img2Edge, 100, 300, 3);

//    Mat res(image1.size(), CV_8UC3, Scalar(0, 0, 0));
//    Mat redImg(image1.size(), CV_8UC3, Scalar(255, 0, 0));
//    Mat blueImg(image1.size(), CV_8UC3, Scalar(0, 0, 255));
//    redImg.copyTo(res, img1Edge);
//    blueImg.copyTo(res, img2Edge);

//    namedWindow(title, WINDOW_NORMAL);
//    imshow(title, res);

//    return;
//}
//static void testProjective(const Mat& img1)
//{
//    Mat img2;

//    // Warp original image
//    Matx<double, 3, 3> projTr(1., 0., 0., 0., 1.3, 0., 0.0001, 0.0001, 1);
//    MapProjec mapTest(projTr);
//    mapTest.warp(img1, img2);
////    showDifference(img1, img2, DIFF_IM);

//    // Register
//    MapperGradProj mapper;
//    MapperPyramid mappPyr(mapper);
//    mappPyr.numLev_ = 5;
//    mappPyr.numIterPerScale_ = 10;
//    cout<<mappPyr.numLev_<<endl;
//    cout<<mappPyr.numIterPerScale_<<endl;
//    Ptr<Map> mapPtr;
//    mappPyr.calculate(img1, img2, mapPtr);

//    // Print result
//    MapProjec* mapProj = dynamic_cast<MapProjec*>(mapPtr.get());
//    mapProj->normalize();
//    cout << endl << "--- Testing projective transformation mapper ---" << endl;
//    cout << Mat(projTr) << endl;
//    cout << Mat(mapProj->getProjTr()) << endl;

//    // Display registration accuracy
//    Mat dest;
//    mapProj->inverseWarp(img2, dest);
//    Mat img1u, img2u;
//    img1.convertTo(img1u, CV_8UC3);
//    img2.convertTo(img2u, CV_8UC3);
//    namedWindow("img1", WINDOW_NORMAL);
//    imshow("img1", img1u);
//    namedWindow("img2", WINDOW_NORMAL);
//    imshow("img2", img2u);
//    showDifferenceEdge(img1, dest, DIFF_REGPIX_IM);

//    waitKey(0);
//    cvDestroyWindow(DIFF_IM);
//    cvDestroyWindow(DIFF_REGPIX_IM);
//}

//int main(void)
//{
//    Mat img1;
//    img1 = imread("/home/kimiwings/SafeDrive/test/home.png", CV_LOAD_IMAGE_UNCHANGED);
//    if(!img1.data) {
//        cout <<  "Could not open or find file" << endl;
//        return -1;
//    }
//    // Convert to double, 3 channels
//    img1.convertTo(img1, CV_64FC3);

//    testProjective(img1);
//}
