#include "tracker.hpp"

#define NN_MATCH_RATIO 0.75f
#define NN_MATCH_NUMBER 15
#define RANSAC_THRES 2.5f

Tracker::Tracker()
{
	detector = ORB::create();
	matcher = DescriptorMatcher::create("BruteForce-Hamming");
}

void Tracker::setTarget(const Mat frame)
{
	targetFrame = frame;
	detector->detectAndCompute(targetFrame, noArray(), targetKp, targetDesc);
}

Mat Tracker::match(const Mat frame)
{
	Mat curDesc;
	vector<KeyPoint> curKp;
	detector->detectAndCompute(frame, noArray(), curKp, curDesc);

	vector< vector<DMatch> > matches;
	matcher->knnMatch(targetDesc, curDesc, matches, 2);
	vector<Point2f> targetMatchedKp, curMatchedKp;
	for(int i=0; i<(int)matches.size(); i++)
	{
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
		Mat nullRes = frame.clone();
		cout<<"homography fail!"<<endl;
		return nullRes;
	}

	vector<Point2f> projectedKp;
	perspectiveTransform(targetMatchedKp, projectedKp, homography);

	Mat res = frame.clone();
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
