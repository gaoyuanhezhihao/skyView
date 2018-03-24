#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <array>
#include <numeric>
//#include "vanish.hpp"
#include "Config.hpp"
#include "base.hpp"
using namespace std;
using namespace cv;

const int CLICK_NUM = 1;
array<cv::Point, CLICK_NUM> click_pts;
int click_cnt = 0;
int hough_thres = 150;
enum SelectResult{
    Retry,
    Success,
    Over
};

struct Sample{
    Mat rgb;
    vector<Vec2f> lines;
    vector<Point> pts;
};

