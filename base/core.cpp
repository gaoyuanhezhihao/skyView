#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "core.hpp"
#include "base.hpp"

using namespace std;
using namespace cv;
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
