#include "tracker.hpp"
#include "line_detector.hpp"
#include "gsvFetcher.hpp"
#include <dirent.h>
#include <string.h>
using namespace std;

int main(int argc, char **argv)
{
	Tracker *tracker = new Tracker();
	Line_detector *line_detector = new Line_detector();
    GSVFetcher *fetcher = new GSVFetcher();

	string targetName = argv[1];
	Mat targetFrame = imread(targetName);
	tracker->setTarget(targetFrame);

	int n = 5;
	float lan = 44.9739000;
	float lon = -93.2704977;
	float head = 220;
	float pitch = 0;
	double minNorm = HOMO_NORM_THRES+1;
	int idx = -1;
	Mat matchedNorm, minHomo;
	for(int i=0; i<n; i++)
	{
		Mat curFrame = fetcher->get(Size(640, 480), lan, lon, head, pitch);
		lan += 0.0003;
		matchedNorm = tracker->match(curFrame);
		if(!matchedNorm.empty() && norm(matchedNorm) < minNorm)
		{
			minNorm = norm(matchedNorm);
			minHomo = matchedNorm;
			idx = i;
		}
	}
	Mat matchedFrame = fetcher->get(Size(640, 480), 44.9740000+0.0003*idx, lon, head, pitch);
	vector<Point2f> lanePoints, projectedPoints;
	lanePoints = line_detector->process(matchedFrame);
//	perspectiveTransform(lanePoints, projectedPoints, minHomo);
	Mat projectedImg = targetFrame.clone();
	for(int i=0; i<(int)projectedPoints.size(); i++)
	{
//		projectedImg.at<Vec3b>(projectedPoints[i]) = Vec3b(0, 255, 255);
	}
	imshow("result", projectedImg);
	waitKey(100000000);
//	Mat roi = lineImg(Rect(lineImg.cols/4,lineImg.rows/2,lineImg.cols/2,lineImg.rows/2));
//	roi.copyTo(matchedImg(Rect(matchedImg.cols/4,matchedImg.rows/2,matchedImg.cols/2,matchedImg.rows/2)));
}
