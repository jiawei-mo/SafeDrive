#include "tracker.hpp"
#include "line_detector.hpp"
#include <dirent.h>
#include <string.h>
using namespace std;

int main(int argc, char **argv)
{
    namedWindow("result", WINDOW_NORMAL);
	Tracker *tracker = new Tracker();
	Line_detector *line_detector = new Line_detector();

	string targetName = argv[1];
	Mat targetFrame = imread(targetName);
	tracker->setTarget(targetFrame);

	string dirName = argv[2];
	struct dirent **namelist;
	int n = scandir(dirName.c_str(), &namelist, NULL, alphasort);
	if(n < 0)
	{
		perror("scandir");
	}
	else
	{
		for(int i=0; i<n; i++)
		{
			string imgStr = namelist[i]->d_name;
			if(imgStr.compare(".") == 0 || imgStr.compare("..") == 0)
			{
				continue;
			}
			string imgPath(dirName + imgStr);
			cout<<imgPath<<endl;
			Mat curFrame = imread(imgPath);
			
			Mat matchedImg, lineImg;
			matchedImg = tracker->match(curFrame);
			lineImg = line_detector->process(curFrame);
			
			Mat roi = lineImg(Rect(lineImg.cols/4,lineImg.rows/2,lineImg.cols/2,lineImg.rows/2));
			roi.copyTo(matchedImg(Rect(matchedImg.cols/4,matchedImg.rows/2,matchedImg.cols/2,matchedImg.rows/2)));

			imshow("result", matchedImg);

			delete namelist[i];
			if(waitKey(10000) >= 0) 
			{
				continue;
			}
		}
		delete namelist;
	}

	delete tracker;
}
