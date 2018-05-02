#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <cassert>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include "base.hpp"
#include "wheel_odom.hpp"
#include "config/Config.hpp"
#include "motion.hpp"
#include "frame_map.hpp"

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

struct GlobalMatchPt{
    double dist;
    cv::Point2f g_pt;
    GlobalMatchPt(double dist, Point2f global_pt):dist(dist), g_pt(global_pt){}
    GlobalMatchPt():dist(DOUBLE_MAX), g_pt(-1,-1){}
};

void __predict_match_one(const Frame_Interface & prev, Frame_Interface & cur, vector<GlobalMatchPt> & matches) {
    assert(cur.pts().size() == matches.size());
    const vector<Point2f> & prev_global_pts = prev.global_pts();
    const vector<Point2f> & cur_pts = cur.pts();
    const vector<Point2f> & prev_pts = prev.pts();
    const int prev_sz = prev_pts.size();
    const int cur_sz = cur_pts.size();
    WheelOdom odm(prev.get_id(), cur.get_id());
    vector<Point2f> warped_pts = odm.transform(prev_pts);
    for(int i = 0; i < prev_sz; ++i) {
        double min_dist = DOUBLE_MAX;
        int m = -1;
        for(int j = 0; j < cur_sz; ++j) {
            double dist = street_dist(warped_pts[i], cur_pts[j]);
            if(dist < min_dist) {
                min_dist = dist;
                m = j;
            }
        }
        if(-1 != m && matches[m].dist < min_dist) {
            matches[m].dist = min_dist;
            matches[m].g_pt = prev_global_pts[i];
        }
    }
    return;
}

int cnt_valid_match(const vector<GlobalMatchPt> & matches) {
    static const double thres = get_param("wheel_odom_match_dist_thres");
    int cnt = 0;
    for(const GlobalMatchPt & mch: matches) {
        if(mch.dist < thres) {
            ++cnt;
        }
    }
    return cnt;
}
vector<pair<Point2f, Point2f>> dyn_predict_match(Frame_Interface & cur) {
    static const int max_try = get_param("wheel_odom_dyn_match_max_try");
    static const double thres = get_param("wheel_odom_match_dist_thres");
    vector<GlobalMatchPt> matches(cur.pts().size());
    for(int i = 1; i <= max_try; ++i) {
        if(cur.get_id() - i < 0)  {
            break;
        }
        shared_ptr<Frame_Interface> prev = get_frame(cur.get_id()-i);
        if(nullptr == prev) {
            continue;
        }

        __predict_match_one(*prev, cur, matches);
    }

    const vector<Point2f> & cur_pts = cur.pts();
    vector<pair<Point2f, Point2f>> rst;
    for(size_t i = 0; i < cur_pts.size(); ++i) {
        GlobalMatchPt & mch = matches[i];
        if(mch.dist < thres) {
            rst.emplace_back(mch.g_pt, cur_pts[i]);
        }
    }
    return rst;
}

vector<pair<Point2f, Point2f>> __predict_matchN(const vector<shared_ptr<Frame_Interface>> & prevs, Frame_Interface & cur) {
    static const double thres = get_param("wheel_odom_match_dist_thres");
    vector<GlobalMatchPt> matches(cur.pts().size());
    for(shared_ptr<Frame_Interface> pf: prevs) {
        __predict_match_one(*pf, cur, matches);
    }
    const vector<Point2f> & cur_pts = cur.pts();
    vector<pair<Point2f, Point2f>> rst;
    for(size_t i = 0; i < cur_pts.size(); ++i) {
        GlobalMatchPt & mch = matches[i];
        if(mch.dist < thres) {
            rst.emplace_back(mch.g_pt, cur_pts[i]);
        }
    }
    return rst;
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
    vector<int> lmk_ids(cur_sz, -1);
    const vector<int> & prev_lmk_ids = prev.get_lmk_ids();
    vector<bool> added(cur_sz, false);
    for(int i = 0; i < prev_sz; ++i) {
        float min_dist = FLOAT_MAX;
        int mch_id = -1;
        for(int j = 0; j < cur_sz; ++j) {
            if(added[j]) {
                continue;
            }
            float dist = street_dist(prev_pts[i], cur_pts[j]);
            if(dist < min_dist) {
                min_dist = dist;
                mch_id = j;
            }
        }

        cout << "min_dist=" << min_dist << endl;
        if(min_dist < thres) {
            mch_pts.push_back({prev_global_pts[i], cur_pts[mch_id]});
            lmk_ids[mch_id] = prev_lmk_ids[i];
            assert(-1 != prev_lmk_ids[i]);
            added[mch_id] = true;
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


shared_ptr<Frame_Pose_Interface> WheelOdom::predict_pose_N(Frame_Interface & cur) {
    static const int match_thres = get_param("minimum_of_match_cnt");
    vector<pair<Point2f, Point2f>> matches = dyn_predict_match(cur);
    if(int(matches.size()) >= match_thres) {
        shared_ptr<Frame_Interface> prev = get_first_prev_frame(cur);
        return Ceres_solve_im2g(*prev->get_pose(), matches);
    }else {
        return nullptr;
    }
}
