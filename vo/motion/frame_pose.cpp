#include <cassert>
#include <memory>
#include "frame_pose.hpp"
#include "motion.hpp"
#include "boost/format.hpp"
using namespace std;

SimpleFramePose::SimpleFramePose(shared_ptr<Frame_Pose_Interface>  prev_pose, const Motion_Interface & mot){
    assert(prev_pose->is_valid());
    _t[0] = prev_pose->dx() + mot.get_dx();
    _t[1] = prev_pose->dy() + mot.get_dy();
    _t[2] = prev_pose->theta() + mot.get_theta();
    _valid = true;
}

bool SimpleFramePose::report(ostream & out) {
    assert(_valid);
    out << "x=" <<  _t[0] << " y=" << _t[1] << " theta=" << _t[2]<< endl;
    return true;
}

SimpleFramePose::SimpleFramePose(double x, double y, double theta) {
    _t[0] = x;
    _t[1] = y;
    _t[2] = theta;
    _valid = true;
}
SimpleFramePose::~SimpleFramePose(){
    //cout << "~SimpleFramePose" << endl;
}

double SimpleFramePose::dx() {return _t[0];}
double SimpleFramePose::dy() {return _t[1];}
double SimpleFramePose::theta() {return _t[2];}
bool SimpleFramePose::is_valid() {return _valid;}

GlobalPose::GlobalPose(double x, double y, double theta):_x(x), _y(y), _th(theta), _im2g(-x, -y, -theta), _g2im(x, y, theta){}

bool GlobalPose::report(ostream & out) {
    //assert(_valid);
    out << "x=" <<  _x << " y=" << _y << " theta=" << _th<< endl;
    return true;
}

Point2f GlobalPose::pt_im2g(const Point2f & pt_im) {
    return _im2g * pt_im;
}

Point2f GlobalPose::pt_g2im(const Point2f & pt_g){
    return _g2im * pt_g;
}

void SimpleFrame::set_global_pts() {
    CV_Assert(_pts.size() > 0);
    CV_Assert(_global_pts.empty());/*only set one time*/

    int sz = _pts.size();
    cout << "DEBUG---" << endl;
    for(int i = 0; i < sz; ++i) {
        //cout << "DEBUG:" << _pPos->whoami() << endl;
        _global_pts.push_back(_pPos->pt_im2g(_pts[i]));
        cout << _pts[i]  << "-->" << _global_pts[i] << endl;
    }
    cout << "DEBUG***"<< endl;
}

void SimpleFrame::set_pose(shared_ptr<Frame_Pose_Interface> pos) {
    _pPos = pos;
    set_global_pts();
}

string GlobalPose::format() {
    static boost::format fmter("dx=%1% dy=%2% dth=%3%");
    return (fmter % _x % _y % _th).str();
}
