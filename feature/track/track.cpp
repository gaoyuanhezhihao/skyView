#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <limits>
#include "track.hpp"
#include "base.hpp"
#include "debug.hpp"

using namespace std;
using namespace cv;

//const double DOUBLE_MAX = std::numeric_limits<double>::max();
const double FLOAT_MAX = std::numeric_limits<float>::max();
bool Tracker::run() {
    static Size subPixWinSize(10,10), winSize(31,31);
    static TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);

    CV_Assert(!_old_gray.empty());
    CV_Assert(!_new_gray.empty());

    const int old_sz = _old_pts.size();
    //vector<Point2f> tracked_pts(old_sz);
    //vector<uchar> states(old_sz);
    vector<float> err(old_sz);
    //cv::imshow("_old_gray", _old_gray);
    //cv::imshow("_new_gray", _new_gray);
    //cv::waitKey(0);
    calcOpticalFlowPyrLK(_old_gray, _new_gray, _old_pts,
            _tracked_pts, _states, err, winSize, 3,
            termcrit, 0, 0.001);
    //calcOpticalFlowPyrLK(_old_gray, _new_gray, _old_pts,
            //_tracked_pts, _states, err, winSize, 3,
            //termcrit, 0, 0.001);
    return true;
}

shared_ptr<NewMatch> match_pts(const Tracker & tracker, NewFrame & prevF, NewFrame & curF) {
    static const double dist_thres = configs["track_match_dist_thres"];
    const vector<Point2f> & track_pts = tracker.ref_tracked_pts();
    const vector<uchar> & track_status = tracker.ref_status();
    const vector<Point2f> & new_keyPts = curF.keyPts();
    const vector<bool> added(new_keyPts.size(), false);

    if(new_keyPts.empty() || track_pts.empty()) {
        return nullptr;
    }
    shared_ptr<NewMatch> pMch = std::make_shared<NewMatch>(&prevF, &curF);
    for(size_t i = 0; i < track_pts.size(); ++i) {
        float min_dist = FLOAT_MAX;
        int match_id = -1;
        if(!track_status[i]) {
            continue;
        }

        for(size_t k = 0; k < new_keyPts.size(); ++k) {
            float dist = street_dist(track_pts[i],
                    new_keyPts[k]);
            if(min_dist > dist){
                min_dist = dist;
                match_id = k;
            }
        }
        if(-1 != match_id && min_dist < dist_thres) {
            pMch->add(i, match_id);
        }
    }
    return pMch;
}

cv::Mat Tracker::draw(){
    static const double dist_thres = configs["track_match_dist_thres"];
    CV_Assert(!_old_gray.empty() && !_new_gray.empty());
    CV_Assert(_tracked_pts.size() == _states.size());
    CV_Assert(_tracked_pts.size() == _old_pts.size());
    cv::Mat img1, img2;
    cvtColor(_old_gray, img1, CV_GRAY2BGR);
    cvtColor(_new_gray, img2, CV_GRAY2BGR);
    cv::Mat imgTrack;
    cv::hconcat(img1, img2, imgTrack);
    //for(const Point2f & pt1 :_old_pts) {
    //}

    int good_track_cnt = 0;
    //SHOW(_old_pts.size());
    for(size_t i = 0; i < _states.size(); ++i) {
        const Point2f & p1 = _old_pts[i];
        if(!_states[i]) {
            circle(imgTrack, p1, 3, RED, -1, 8);
            continue;
        }
        const Point2f & p2 = _tracked_pts[i] + Point2f(img1.cols, 0);
        circle(imgTrack, p1, 3, GREEN, -1, 8);
        circle(imgTrack, p2, 3, GREEN, -1, 8);
        circle(imgTrack, p2, dist_thres, GREEN, 1);
        cv::line(imgTrack, p1, p2, rand_color(), 1, CV_AA);
        ++ good_track_cnt;
    }
    //cout << "good_track_cnt=" << good_track_cnt << endl;

    return imgTrack;
}
