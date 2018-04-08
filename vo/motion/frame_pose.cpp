#include <cassert>
#include <memory>
#include "frame_pose.hpp"
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
