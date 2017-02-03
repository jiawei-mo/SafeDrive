#include "rotator.h"

Rotator::Rotator()
{
    msac = shared_ptr<MSAC>(new MSAC());
}

void Rotator::process(const Mat& input, Mat& output)
{
    msac->init(MODE_NIETO, input.size(), false);

    Mat gray_img;
    cvtColor(input, gray_img, CV_BGR2GRAY);

    cv::Mat imgCanny;

        // Canny
        cv::Canny(gray_img, imgCanny, 180, 120, 3);

        // Hough
        vector<vector<cv::Point> > lineSegments;
        vector<cv::Point> aux;
        vector<Vec4i> lines;
        int houghThreshold = 70;
        if(gray_img.cols*gray_img.rows < 400*400)
            houghThreshold = 100;

        cv::HoughLinesP(imgCanny, lines, 1, CV_PI/180, houghThreshold, 10,10);

        while(lines.size() > MAX_NUM_LINES)
        {
            lines.clear();
            houghThreshold += 10;
            cv::HoughLinesP(imgCanny, lines, 1, CV_PI/180, houghThreshold, 10, 10);
        }
        for(size_t i=0; i<lines.size(); i++)
        {
            Point pt1, pt2;
            pt1.x = lines[i][0];
            pt1.y = lines[i][1];
            pt2.x = lines[i][2];
            pt2.y = lines[i][3];
            line(output, pt1, pt2, CV_RGB(0,0,0), 2);
            /*circle(outputImg, pt1, 2, CV_RGB(255,255,255), CV_FILLED);
                circle(outputImg, pt1, 3, CV_RGB(0,0,0),1);
                circle(outputImg, pt2, 2, CV_RGB(255,255,255), CV_FILLED);
                circle(outputImg, pt2, 3, CV_RGB(0,0,0),1);*/

            // Store into vector of pairs of Points for msac
            aux.clear();
            aux.push_back(pt1);
            aux.push_back(pt2);
            lineSegments.push_back(aux);
        }

        // Multiple vanishing points
        std::vector<cv::Mat> vps;			// vector of vps: vps[vpNum], with vpNum=0...numDetectedVps
        std::vector<std::vector<int> > CS;	// index of Consensus Set for all vps: CS[vpNum] is a vector containing indexes of lineSegments belonging to Consensus Set of vp numVp
        std::vector<int> numInliers;

        std::vector<std::vector<std::vector<cv::Point> > > lineSegmentsClusters;

        // Call msac function for multiple vanishing point estimation
        msac->multipleVPEstimation(lineSegments, lineSegmentsClusters, numInliers, vps, 2);
        for(int v=0; v<vps.size(); v++)
        {
            printf("VP %d (%.3f, %.3f, %.3f)", v, vps[v].at<float>(0,0), vps[v].at<float>(1,0), vps[v].at<float>(2,0));
            fflush(stdout);
            double vpNorm = cv::norm(vps[v]);
            if(fabs(vpNorm - 1) < 0.001)
            {
                printf("(INFINITE)");
                fflush(stdout);
            }
            printf("\n");
        }

        // Draw line segments according to their cluster
        msac->drawCS(output, lineSegmentsClusters, vps);
}
