#ifndef PREDICTOR_H
#define PREDICTOR_H

#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "track.hpp"
#include "base.hpp"

using namespace std;
using namespace cv;
class OpticalLinePredictor{
    private:
        //const SimpleFrame & prevF;
        const vector<vector<int>> & _line_pts_map;
        const Tracker & _trk;
        vector<pair<double, double>>  _theta_rgs;
        vector<Vec2f>  _tracked_lines;
        bool runned = false;
    public:
        OpticalLinePredictor(const vector<vector<int>> & line_pts_map, const Tracker & tracker):_line_pts_map(line_pts_map), _trk(tracker){}

        bool run();
        const vector<pair<double, double>> & theta_rgs()const{
            CV_Assert(runned);
            return _theta_rgs;
        }
        const vector<Vec2f> & tracked_lines()const {
            CV_Assert(runned);
            return _tracked_lines;
        }
};
#endif //PREDICTOR_H