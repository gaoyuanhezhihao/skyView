#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "motion.hpp"
#include "pose.hpp"
#include "frame_pose.hpp"
#include "boost/format.hpp"


using namespace std;
using namespace cv;


shared_ptr<Frame_Pose_Interface> Ceres_solve_im2g(Frame_Pose_Interface & prev_pose, vector<pair<Point2f, Point2f>> & mch_pts){
    const int cnt = mch_pts.size();
    assert(cnt >= 2);
    vector<array<double, 2>> global_pts(cnt);/*(x, y)*/
    vector<array<double, 2>> im_pts(cnt);/*(x, y)*/
    for(int i = 0; i < cnt; ++i) {
        global_pts[i][0] = mch_pts[i].first.x;
        global_pts[i][1] = mch_pts[i].first.y;

        im_pts[i][0] = mch_pts[i].second.x;
        im_pts[i][1] = mch_pts[i].second.y;
    }
    /* init of pose */
    double cam_theta = prev_pose.theta();
    double cam_t[2]{prev_pose.dx(),prev_pose.dy()}; 
    /*TODO use RANSAC */
    if(!solve_2D_im2g(im_pts, global_pts, cam_theta, cam_t)) {
    //if(!solve_2D_g2im(cur_pts, prev_pts, cam_theta, cam_t)) {
    //if(!solve_2D_pose(cur_pts, prev_pts, cam_theta, cam_t)) {
        return nullptr;
    }else {
        return make_shared<GlobalPose>(cam_t[0], cam_t[1], cam_theta);
    }
}

bool Ceres_global_motion::calc_cam_motion() {
    const vector<pair<int, int>> & ids = _pMch->ids();
    int cnt = ids.size();
    CV_Assert(cnt >= 2);
    vector<array<double, 2>> global_pts(cnt);/*(x, y)*/
    vector<array<double, 2>> im_pts(cnt);/*(x, y)*/

    const vector<Point2f> & pts1 = _pMch->pf1()->global_pts();
    const vector<Point2f> & pts2 = _pMch->pf2()->pts();

    for(int i = 0; i < cnt; ++i) {
        const std::pair<int, int> &  mch_pt_ids = ids[i];
        const double x1 = pts1[mch_pt_ids.first].x;
        const double y1 = pts1[mch_pt_ids.first].y;
        global_pts[i][0] = x1;
        global_pts[i][1] = y1;

        const double x2 = pts2[mch_pt_ids.second].x;
        const double y2 = pts2[mch_pt_ids.second].y;
        im_pts[i][0] = x2;
        im_pts[i][1] = y2;
    }
    /* init of pose */
    std::shared_ptr<Frame_Pose_Interface> prev_pose = _pMch->pf1()->get_pose();
    double cam_theta = prev_pose->theta();
    double cam_t[2]{prev_pose->dx(),prev_pose->dy()}; 
    /*TODO use RANSAC */
    if(!solve_2D_im2g(im_pts, global_pts, cam_theta, cam_t)) {
    //if(!solve_2D_g2im(cur_pts, prev_pts, cam_theta, cam_t)) {
    //if(!solve_2D_pose(cur_pts, prev_pts, cam_theta, cam_t)) {
        return false;
    }
    _dx = cam_t[0];
    _dy = cam_t[1];
    _theta = cam_theta;
    return true;
}


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
    out << "dx=" << _dx << "\ndy=" << _dy << "\ndtheta" << _theta << '\n';

}

void Ceres_global_motion::report(ostream & out) const {
    out << "dx=" << _dx << "\ndy=" << _dy << "\ndtheta" << _theta << '\n';
}

string Ceres_global_motion::format()const {
    static boost::format fmter("dx=%1% dy=%2% dth=%3%");
    return (fmter % _dx % _dy % _theta).str();
}

cv::Point TransformMat::operator*(const cv::Point & pt) {
    Point b;
    //b.x = _R[0][0] * pt.x + _R[0][1] * pt.y + _t[0][0];
    //b.y = _R[1][0] * pt.x + _R[1][1] * pt.y + _t[0][1];
    b.x = _R[0][0] * pt.x + _R[0][1] * pt.y + _t[0];
    b.y = _R[1][0] * pt.x + _R[1][1] * pt.y + _t[1];
    return b;
}

cv::Point2f TransformMat::operator*(const cv::Point2f & pt) {
    Point2f b;
    //b.x = _R[0][0] * pt.x + _R[0][1] * pt.y + _t[0][0];
    //b.y = _R[1][0] * pt.x + _R[1][1] * pt.y + _t[0][1];
    b.x = _R[0][0] * pt.x + _R[0][1] * pt.y + _t[0];
    b.y = _R[1][0] * pt.x + _R[1][1] * pt.y + _t[1];
    return b;
}

TransformMat::TransformMat(double dx, double dy, double dth) {
    _R[0][0] = cos(dth);
    _R[0][1] = sin(dth);
    _R[1][0] = -sin(dth);
    _R[1][1] = cos(dth);
    cout << "TransformMat:" << endl;
    cout <<  _R[0][0] << ", " <<  _R[0][1] << '\n' 
        << _R[1][0] << ", " << _R[1][1] << '\n';

    //_t[0][0] = -dx;
    //_t[1][0] = -dy;
    _t[0] = -dx;
    _t[1] = -dy;
    cout << _t[0] << ", " << _t[1] << endl;
}
MatG2IM::MatG2IM(double x, double y, double th){
    _R[0][0] = cos(th);
    _R[0][1] = sin(th);
    _R[1][0] = -sin(th);
    _R[1][1] = cos(th);
    _t[0] = -x;
    _t[1] = -y;
}

