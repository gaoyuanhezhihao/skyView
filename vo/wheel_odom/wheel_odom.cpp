#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include "wheel_odom.hpp"
#include "config/Config.hpp"
#include "motion.hpp"
#include <cassert>

using namespace std;
using namespace cv;

const double FLOAT_MAX = std::numeric_limits<float>::max();

void flip_coordinates(Odom_Pack & odm) {
    odm.dy = - odm.dy;
    odm.dth = -odm.dth;
}

void cm2pxl(Odom_Pack & odm) {
    static const int pxl_unit = get_param("pxl_unit");
    static const double cm_unit = get_param("cm_unit");
    static const double rt = pxl_unit/cm_unit;

    //cout << "cm2pxl rt=" << rt << endl;
    odm.dx *= rt;
    odm.dy *= rt;
}

WheelOdom::WheelOdom(int fromID, int toID){
    Odom_Pack from_odm = read_wheel_odom(fromID);
    Odom_Pack to_odm = read_wheel_odom(toID);

    flip_coordinates(from_odm);
    flip_coordinates(to_odm);

    cm2pxl(to_odm);
    cm2pxl(from_odm);

    Odom_Pack deta_odm = to_odm - from_odm;
    _T = std::unique_ptr<TransformMat> (new TransformMat(deta_odm.dx, deta_odm.dy, deta_odm.dth));
}

vector<Point> WheelOdom::transform(const vector<Point> & fromPts) {
    vector<Point> toPts(fromPts.size());
    for(size_t i = 0; i < fromPts.size(); ++i) {
        toPts[i] = (*_T) * fromPts[i];
    }
    return toPts;
}

vector<Point2f> WheelOdom::transform(const vector<Point2f> & fromPts) {
    vector<Point2f> toPts(fromPts.size());
    for(size_t i = 0; i < fromPts.size(); ++i) {
        toPts[i] = (*_T) * fromPts[i];
    }
    return toPts;
}



Odom_Pack read_wheel_odom(int i) {
    static const string dir(string(get_param("src_dir")));
    boost::filesystem::path fp(dir + to_string(i) + ".txt");
    assert(boost::filesystem::exists(fp));
    std::ifstream f(fp.c_str());
    Odom_Pack odm;
    f >> odm.dx >> odm.dy >> odm.dth;
    return odm;
}

vector<pair<Point2f, Point2f>> __predict_match(Frame_Interface & prev, Frame_Interface & cur) {
    static const double thres = get_param("wheel_odom_match_dist_thres");
    vector<Point2f> prev_pts = prev.pts();
    const vector<Point2f> & prev_global_pts = prev.global_pts();

    assert(prev_pts.size() == prev_global_pts.size());
    WheelOdom odm(prev.get_id(), cur.get_id());
    const vector<Point2f> & cur_pts = cur.pts();
    cout << "before transform prev_pts:\n";
    for(Point2f & pt: prev_pts) {
        cout << pt << "\n";
    }
    prev_pts = odm.transform(prev_pts);

    const int prev_sz = prev_pts.size();
    const int cur_sz = cur_pts.size();
    cout << "prev_pts:\n";
    for(Point2f & pt: prev_pts) {
        cout << pt << "\n";
    }
    cout << "cur pts:\n";
    for(const Point2f & pt: cur_pts) {
        cout << pt << "\n";
    }
    vector<pair<Point2f, Point2f>> mch_pts;
    for(int i = 0; i < prev_sz; ++i) {
        float min_dist = FLOAT_MAX;
        int mch_id = -1;
        for(int j = 0; j < cur_sz; ++j) {
            float dist = street_dist(prev_pts[i], cur_pts[j]);
            if(dist < min_dist) {
                min_dist = dist;
                mch_id = j;
            }
        }

        cout << "min_dist=" << min_dist << endl;
        if(min_dist < thres) {
            mch_pts.push_back({prev_global_pts[i], cur_pts[mch_id]});
        }
    }

    return mch_pts;
}


shared_ptr<Frame_Pose_Interface> WheelOdom::predict_pose(Frame_Interface & prev, Frame_Interface & cur) {
    vector<pair<Point2f, Point2f>> mch_pts = __predict_match(prev, cur);
    if(mch_pts.size() >= 2) {
        return Ceres_solve_im2g(*prev.get_pose(), mch_pts);
    }else {
        return nullptr;
    }
}
