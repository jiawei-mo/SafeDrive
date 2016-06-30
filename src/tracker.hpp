#include "parameters.hpp"

struct TrackRes
{
	Mat homo;
	double score;
};

class Tracker
{
protected:
	Ptr<ORB> detector;
	Ptr<DescriptorMatcher> matcher;

	Mat targetFrame, targetDesc;
	vector<KeyPoint> targetKp;

public:
	Tracker();
	void setTarget(const Mat frame);
	TrackRes match(const Mat frame, bool showImg);
};
