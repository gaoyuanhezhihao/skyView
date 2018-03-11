#include <random>
#include <iostream>
#include <vector>
#include <array>

#include "ceres/ceres.h"
#include "glog/logging.h"

#include "pose.hpp"
#include "core.hpp"

using namespace std;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;

struct Affine2D {
    Affine2D(const double cur_in[2], const double prev_in[2]) {
        cur[0] = cur_in[0];
        cur[1] = cur_in[1];
        prev[0] = prev_in[0];
        prev[1] = prev_in[1];
    }

    template <typename T>
        bool operator()(const T* theta,
                const T* t,
                T* residuals) const {
            const T q_0 =  cos(theta[0]) * cur[0] - sin(theta[0]) * cur[1] + t[0];
            const T q_1 =  sin(theta[0]) * cur[0] + cos(theta[0]) * cur[1] + t[1];
            //const T f = TemplatedComputeDistortion(q_0 * q_0 + q_1 * q_1);
            //residuals[0] = prev[0] - f * q_0;
            //residuals[1] = prev[1] - f * q_1;
            residuals[0] = prev[0] - q_0;
            residuals[1] = prev[1] - q_1;
            return true;
        }

    double cur[2];
    double prev[2];
};

bool solve_2D_pose(const vector<array<double, 2>> & cur_pts, const vector<array<double, 2>> & prev_pts, double & _theta, double _t[2]) {
    Problem problem;
    const int cnt = prev_pts.size();


    //double _theta=0;
    //double _t[2]={0., 0.};
    for(int i = 0; i < cnt; ++i){
        ceres::CostFunction * cost_func = new AutoDiffCostFunction<Affine2D, 2, 1, 2> (new Affine2D(cur_pts[i].data(), prev_pts[i].data()));
        problem.AddResidualBlock(cost_func, NULL, &_theta, _t);
    }
    Solver::Options opt;
    opt.max_num_iterations = 25;
    opt.linear_solver_type = ceres::DENSE_QR;
    opt.minimizer_progress_to_stdout = false;
    Solver::Summary summary;
    Solve(opt, &problem, &summary);
    //std::cout << summary.BriefReport() << "\n";
    return true;
}

//bool Pose2D::solve(const vector<array<double, 2>> & cur_pts,
        //const vector<array<double, 2>> & prev_pts) {
    //return solve_2D_pose(cur_pts, prev_pts, _theta, _t);
//}

//bool solve_BA(const vector<array<double, 2>> & pt_global, const vector<array<double, 2>> & pt_cur_local, double & _theta, double _t[2]) {
    //assert(pt_global.size() == pt_cur_local.size());
    //return solve_2D_pose(pt_cur_local, pt_global, _theta, _t);
//}


void NewFrame::add_restrict(vector<array<double, 2>> & pts_cur_local,
        const cv::Point2f & pt_local, NewFrame* pF, const int id,
        vector<array<double,2>> & pts_global) {
    if(nullptr == pF) {
        return;
    }
    assert(pF->_pMatchFrames.size() == pF->_global_pts.size());
    assert(pF->_match_keyPt_ids.size() == pF->_global_pts.size());
    assert(pF->_global_pts.size() == pF->_KeyPts.size());
    pts_cur_local.emplace_back(pt_local.x, pt_local.y);
    pts_global.emplace_back(pF->_global_pts[id].x, pF->_global_pts.pts[id].y);
    add_restrict(pts_cur_local, pt_local, pF->_pMatchFrames[id], pF->_match_keyPt_ids[id], pts_global);
}

bool NewFrame::neib_BA() {
    const int kp_cnt = _KeyPts.size();
    vector<array<double, 2>> pts_ref_global;
    vector<array<double, 2>> pts_cur_local;
    for(int i = 0; i < kp_cnt; ++i) {
        add_restrict(pts_cur_local, _KeyPts[i],
                _pMatchFrames[i], _match_keyPt_ids[i], pts_ref_global);
    }
    solve_2D_pose(pts_cur_local, pts_ref_global, _theta, _t);
    return true;
}
