#ifndef POSE_H
#define POSE_H

#include <vector>
#include <array>

using namespace std;
class Pose2D{
    private:
        double _theta;
        double _t[2];/* x, y */
    public:
        bool solve(const vector<array<double, 2>> & cur_pts,
                const vector<array<double, 2>> & prev_pts);
};

bool solve_2D_pose(const vector<array<double, 2>> & cur_pts, const vector<array<double, 2>> & prev_pts, double & _theta, double _t[2]);
bool solve_2D_g2im(const vector<array<double, 2>> & cur_pts, const vector<array<double, 2>> & prev_pts, double & _theta, double _t[2]);
bool solve_2D_im2g(const vector<array<double, 2>> & im_pts, const vector<array<double, 2>> & global_pts, double & _theta, double _t[2]);
#endif //POSE_H
