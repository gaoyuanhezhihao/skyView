#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "motion.hpp"
#include "pose.hpp"


using namespace std;
using namespace cv;
bool Ceres_2frame_motion::calc_cam_motion() {
    const vector<pair<int, int>> & ids = _pMch->ids();
    int cnt = ids.size();
    CV_Assert(cnt >= 2);
    vector<array<double, 2>> prev_pts(cnt);/*(x, y)*/
    vector<array<double, 2>> cur_pts(cnt);/*(x, y)*/

    const vector<Point2f> & pts1 = _pMch->pf1()->pts();
    const vector<Point2f> & pts2 = _pMch->pf2()->pts();

    for(int i = 0; i < cnt; ++i) {
        const std::pair<int, int> &  mch_pt_ids = ids[i];
        const double x1 = pts1[mch_pt_ids.first].x;
        const double y1 = pts1[mch_pt_ids.first].y;
        prev_pts[i][0] = x1;
        prev_pts[i][1] = y1;

        const double x2 = pts2[mch_pt_ids.second].x;
        const double y2 = pts2[mch_pt_ids.second].y;
        cur_pts[i][0] = x2;
        cur_pts[i][1] = y2;
    }
    double cam_theta = 0.0;
    double cam_t[2]{0., 0.};
    if(!solve_2D_pose(cur_pts, prev_pts, cam_theta, cam_t)) {
        return false;
    }
    _dx = cam_t[0];
    _dy = cam_t[1];
    _theta = cam_theta;

    R = Mat::zeros(2, 2, CV_64F);
    t = Mat::zeros(2, 1, CV_64F);
    R.at<double> (0, 0) = cos(cam_theta);
    R.at<double> (0, 1) = -sin(cam_theta);
    R.at<double> (1, 0) = sin(cam_theta);
    R.at<double> (1, 1) = cos(cam_theta);

    t.at<double> (0, 0) = cam_t[0];
    t.at<double> (1, 0) = cam_t[1];
    return true;
}

void Ceres_2frame_motion::report(ostream & out) const{
    out << "dx=" << _dx << "\ndx=" << _dy << "\ndtheta";

}

