#include <iostream>
#include <cmath>
#include "pose.hpp"

vector<array<double,2>> create_data(vector<array<double, 2>> & cur_pts, double theta, double t[2]) {
    vector<array<double, 2>> prev_pts(cur_pts.size());
    for(size_t i = 0; i < cur_pts.size(); ++i) {
        prev_pts[i][0] = cos(theta) * cur_pts[i][0] - sin(theta)*cur_pts[i][1]  + t[0];
        prev_pts[i][1] = sin(theta) * cur_pts[i][0] + cos(theta)*cur_pts[i][1]  + t[1];
    }
    return prev_pts;
}
int main(int argc, char ** argv) {
    /* data */
    //vector<array<double, 2>> cur_pts{{1.0, 1.0}, {1.0, 4.0}, {4.0, 1.0}, {4.0, 4.0}};
    //vector<array<double, 2>> cur_pts{{1.0, 1.0}, {1.0, 4.0}};
    vector<array<double, 2>> cur_pts{{1.0, 1.0}, {1.0, 4.0}};
    double theta = 3.1415926/4;
    double t[2] = {1.0, 3.0};
    vector<array<double, 2>> prev_pts = create_data(cur_pts, theta, t);
    /* problem */
    double _theta=0;
    double _t[2]={0., 0.};
    solve_2D_pose(cur_pts, prev_pts, _theta, _t);
    cout << "real:\ntheta=" << theta << "\nt=" << t[0] << "," << t[1] << endl;
    cout << "predicted:\ntheta=" << _theta << "\nt=" << _t[0] << "," << _t[1] << endl;
}
