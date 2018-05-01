#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <limits>
#include "match.hpp"

using namespace std;
using namespace cv;

const double FLOAT_MAX = std::numeric_limits<float>::max();
bool SimpleMatcher::match() {
    static const double dist_thres = configs["track_match_dist_thres"];
    static const size_t min_match_cnt = int(configs["minimum_of_match_cnt"]);
    const vector<Point2f> & track_pts = _trk.ref_tracked_pts();
    const vector<uchar> & track_status = _trk.ref_status();
    const vector<Point2f> & new_keyPts = _pf2->pts();
    vector<bool> added(new_keyPts.size(), false);

    if(new_keyPts.empty() || track_pts.empty()) {
        return false;
    }

    vector<int> lmk_ids(new_keyPts.size(), -1);
    const vector<int> & prev_lkm_ids  = _pf1->get_lmk_ids();
    for(size_t i = 0; i < track_pts.size(); ++i) {
        float min_dist = FLOAT_MAX;
        int match_id = -1;
        if(!track_status[i]) {
            continue;
        }

        for(size_t k = 0; k < new_keyPts.size(); ++k) {
            if(added[k]) {
                continue;
            }
            float dist = street_dist(track_pts[i],
                    new_keyPts[k]);
            if(min_dist > dist){
                min_dist = dist;
                match_id = k;
            }
        }
        if(-1 != match_id && min_dist < dist_thres) {
            _mch_ids.emplace_back(i, match_id);
            lmk_ids[match_id] = prev_lkm_ids[i];
            assert(-1 != prev_lkm_ids[i]);
            added[match_id] = true;
        }
    }
    _pf1->set_lmk_ids(std::move(lmk_ids));
    return min_match_cnt <= _mch_ids.size();
}

void SimpleMatcher::log_img() const{
    static const string dst_dir = configs["result_dir"];
    static ImgLogger logger(dst_dir, "match");
    CV_Assert(!_pf1->rgb().empty());
    CV_Assert(!_pf2->rgb().empty());
    cv::Mat imgMatches;
    cv::hconcat(_pf1->rgb(), _pf2->rgb(), imgMatches);
    const int w1 = _pf1->rgb().cols;
    //const int h1 = m.pf1->rgb.rows;
    const vector<cv::Point2f> & kp1 = _pf1->pts();
    const vector<cv::Point2f> & kp2 = _pf2->pts();
    draw_points(imgMatches, kp1, RED);
    cout << "match ---" << endl;
    for(const pair<int, int> & mch: _mch_ids) {
        cout << kp1[mch.first].x << ", " <<  kp1[mch.first].y << "-->" << kp2[mch.second].x << ", " << kp2[mch.second].y << endl;
        Point pt1(kp1[mch.first].x, kp1[mch.first].y);
        Point pt2(kp2[mch.second].x+w1, kp2[mch.second].y);
        cv::Scalar color = rand_color();
        cv::circle(imgMatches, pt1, 5, color, 2);
        cv::circle(imgMatches, pt2, 5, color, 2);
        cv::line(imgMatches, pt1, pt2, color, 1, CV_AA);
    }
    cout << "match ==="<< endl;
    //return imgMatches;
    logger.save(imgMatches, to_string(_pf1->get_id())+"--" +to_string(_pf2->get_id()) );
    return;
}
