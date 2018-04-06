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
        vector<pair<double, double>> _v_theta_rgs;
        vector<Vec2f> _v_tracked_lines;
        vector<Vec2f>  _tracked_lines;
        bool runned = false;
        vector<pair<double, double>> get_vertical() const{return _v_theta_rgs;}
    public: OpticalLinePredictor(const vector<vector<int>> & line_pts_map, const Tracker & tracker):_line_pts_map(line_pts_map), _trk(tracker){}

        bool run();
        bool is_failed() const {return _theta_rgs.empty();}
        const vector<pair<double, double>> & theta_rgs()const{
            CV_Assert(runned);
            return _theta_rgs;
        }
        const vector<Vec2f> & tracked_lines()const {
            CV_Assert(runned);
            return _tracked_lines;
        }
        bool predict_from_vertical(const OpticalLinePredictor & vl_p);
};
#endif //PREDICTOR_H
