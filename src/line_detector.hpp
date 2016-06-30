#include "parameters.hpp"
class Line_detector
{
public:
	Line_detector(){};
	vector<Point2f> process(const Mat img);
};
