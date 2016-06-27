#include "tracker.hpp"

#define NN_MATCH_RATIO 0.7f
#define NN_MATCH_NUMBER 10
#define RANSAC_THRES 3.5f

Tracker::Tracker()
{
	detector = ORB::create(100, 2, 8, 31, 0, 2, ORB::HARRIS_SCORE, 31, 20);
	detector->setMaxFeatures(500);
	matcher = DescriptorMatcher::create("BruteForce-Hamming");
}

void Tracker::setTarget(const Mat frame)
{
	targetFrame = frame.clone();
	GaussianBlur(targetFrame, targetFrame, Size(3,3), 1.2, 1.2);
	detector->detectAndCompute(targetFrame, noArray(), targetKp, targetDesc);
}

Mat Tracker::match(const Mat frame)
{
	Mat res = frame.clone();
//	GaussianBlur(res, res, Size(7,7), 1.5, 1.5);
	Mat curDesc;
	vector<KeyPoint> curKp;
	GaussianBlur(res, res, Size(3,3), 1.2, 1.2);
	detector->detectAndCompute(res, noArray(), curKp, curDesc);

	vector< vector<DMatch> > matches;
	matcher->knnMatch(targetDesc, curDesc, matches, 2);
	vector<Point2f> targetMatchedKp, curMatchedKp;
	for(int i=0; i<(int)matches.size(); i++)
	{
		circle(res, curKp[matches[i][0].trainIdx].pt, 0, CV_RGB(0, 0, 255));
		if(matches[i][0].distance < NN_MATCH_RATIO*matches[i][1].distance)
		{
			targetMatchedKp.push_back(targetKp[matches[i][0].queryIdx].pt);
			curMatchedKp.push_back(curKp[matches[i][0].trainIdx].pt);
		}
	}
	Mat homography, inliner_mask;
	if(targetMatchedKp.size() >= NN_MATCH_NUMBER)
	{
		homography = findHomography(targetMatchedKp, curMatchedKp, RANSAC, RANSAC_THRES, inliner_mask);
	}

	if(targetMatchedKp.size() < NN_MATCH_NUMBER || homography.empty())
	{
		cout<<"homography fail!"<<endl;
		return res;
	}

	vector<Point2f> projectedKp;
	perspectiveTransform(targetMatchedKp, projectedKp, homography);

	int count = 0;
	double dist = 0.0;
	for(int i=0; i<(int)targetMatchedKp.size(); i++)
	{
		if(inliner_mask.at<uchar>(i))
		{
			count++;
			dist += (projectedKp[i].x - curMatchedKp[i].x)*(projectedKp[i].x - curMatchedKp[i].x);
			dist += (projectedKp[i].y - curMatchedKp[i].y)*(projectedKp[i].y - curMatchedKp[i].y);
			line(res, targetMatchedKp[i], projectedKp[i], CV_RGB(255, 0, 0));
		}
	}
	dist = dist / count;
	cout<<"projection num: "<<count<<endl;
	cout<<"projection distance: "<<dist<<endl;
	return res;
}
