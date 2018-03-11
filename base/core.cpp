#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "core.hpp"
#include "base.hpp"

using namespace std;
using namespace cv;
void NewMatch::add(int id1, int id2) {
    ids.emplace_back(id1, id2);
    //pf1->add_match_pts(id1, id2, pf2);
    pf2->add_match_pts(id2, id1, pf1);/* cur --> prev */
}
cv::Mat NewMatch::draw() const {
    CV_Assert(!pf1->rgb().empty());
    CV_Assert(!pf2->rgb().empty());
    cv::Mat imgMatches;
    cv::hconcat(pf1->rgb(), pf2->rgb(), imgMatches);
    const int w1 = pf1->rgb().cols;
    //const int h1 = m.pf1->rgb.rows;
    const vector<cv::Point2f> & kp1 = pf1->keyPts();
    const vector<cv::Point2f> & kp2 = pf2->keyPts();
    draw_points(imgMatches, kp1, RED);
    for(const pair<int, int> & mch: ids) {
        Point pt1(kp1[mch.first].x, kp1[mch.first].y);
        Point pt2(kp2[mch.second].x+w1, kp2[mch.second].y);
        cv::Scalar color = rand_color();
        cv::circle(imgMatches, pt1, 5, color, 2);
        cv::circle(imgMatches, pt2, 5, color, 2);
        cv::line(imgMatches, pt1, pt2, color, 1, CV_AA);
    }
    return imgMatches;
}
void NewFrame::add_match_pts(int this_id, int other, NewFrame* pF) {
    assert(_pMatchFrames.size() == _KeyPts.size());
    assert(_match_keyPt_ids.size() == _KeyPts.size());

    _pMatchFrames[this_id] = pF;
    _match_keyPt_ids[this_id] = other;
}
