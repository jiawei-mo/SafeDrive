#include "tracker.hpp"
#include "line_detector.hpp"
#include "gsvFetcher.hpp"
#include <dirent.h>
#include <string.h>
using namespace std;

int main(int argc, char **argv)
{
    namedWindow("result", WINDOW_NORMAL);

	Tracker *tracker = new Tracker();
	Line_detector *line_detector = new Line_detector();
    GSVFetcher *fetcher = new GSVFetcher();

	string targetName = argv[1];
	Mat targetFrame = imread(targetName);
	tracker->setTarget(targetFrame);

//	string dirName = argv[2];
//	struct dirent **namelist;
//	int n = scandir(dirName.c_str(), &namelist, NULL, alphasort);
//	if(n < 0)
//	{
//		perror("scandir");
//	}
//	else
//	{
	int n = 10;
	float lan = 44.9740000;
	float lon = -93.2704977;
	float head = 218.36;
	float pitch = 0;
		for(int i=0; i<n; i++)
		{
//			string imgStr = namelist[i]->d_name;
//			if(imgStr.compare(".") == 0 || imgStr.compare("..") == 0)
//			{
//				continue;
//			}
//			string imgPath(dirName + imgStr);
//			cout<<imgPath<<endl;
//			Mat curFrame = imread(imgPath);

			Mat curFrame = fetcher->get(Size(640, 480), lan, lon, head, pitch);
			lan += 0.0003;
			Mat matchedImg, lineImg;
			matchedImg = tracker->match(curFrame);
			lineImg = line_detector->process(curFrame);

			Mat roi = lineImg(Rect(lineImg.cols/4,lineImg.rows/2,lineImg.cols/2,lineImg.rows/2));
//			roi.copyTo(matchedImg(Rect(matchedImg.cols/4,matchedImg.rows/2,matchedImg.cols/2,matchedImg.rows/2)));

			imshow("result", matchedImg);

//			delete namelist[i];
			if(waitKey(1000000) >= 0)
			{
				continue;
			}
		}
//		delete namelist;
//	}

//	delete tracker;
}
