#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "track.hpp"

using namespace std;
using namespace cv;
bool Tracker::run() {
    static Size subPixWinSize(10,10), winSize(31,31);
    static TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);

    const int old_sz = _old_pts.size();
    //vector<Point2f> tracked_pts(old_sz);
    //vector<uchar> states(old_sz);
    vector<float> err(old_sz);
    calcOpticalFlowPyrLK(_old_gray, _new_gray, _old_pts,
            _tracked_pts, _states, err, winSize, 3,
            termcrit, 0, 0.001);
    return true;
}

match_pts(Tracker & tracker, vector<Point2f> & new_keyPoints) {
    
}
