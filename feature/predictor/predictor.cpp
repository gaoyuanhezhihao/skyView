#include "predictor.hpp"
#include "line.hpp"
#include "range_hough.hpp"

//void add_theta_ranges(const double theta, vector<pair<double, double>> & theta_rgs) {
    //static const double theta_width = double(configs["theta_predict_width"]) * CV_PI/180;
    //double theta_l = theta - theta_width;
    //double theta_r = theta+ theta_width;

    //if(theta_l < 0.0) {
        //theta_rgs.emplace_back(CV_PI + theta_l, CV_PI);
        //theta_rgs.emplace_back(0.0, theta_r);
    //}else if(theta_r > CV_PI) {
        //theta_rgs.emplace_back(0.0, theta_r - CV_PI);
        //theta_rgs.emplace_back(theta_l, CV_PI);
    //}else {
        //theta_rgs.emplace_back(theta_l, theta_r);
    //}
//}

bool OpticalLinePredictor::run(){
    runned = true;

    const vector<Point2f> & tracked_pts = _trk.get_tracked_pts();

    for(size_t i = 0; i < _line_pts_map.size(); ++i) {
        /* find two points in good status */
        int pt_cnt = 0;
        int pt_ids[2] = {-1, -2};
        for(int pt_i: _line_pts_map[i]) {
            if(_trk.check_status(pt_i)) {
                pt_ids[pt_cnt++] = pt_i;
            }

            if(pt_cnt == 2) {
                break;
            }
        }

        if(pt_cnt < 2) {
            continue;
        }
        const int u = pt_ids[0];
        const int v = pt_ids[1];
        _tracked_lines.emplace_back(get_line_from_endPts(tracked_pts[u], tracked_pts[v]));

        //const int u = line_pts_map[i].first;
        //const int v = line_pts_map[i].second;

        //if( !tracker.check_status(u) || !tracker.check_status(v) ) {
            //continue;
        //}
        const double theta = theta_from_endPoint(tracked_pts[u], tracked_pts[v]);
        add_theta_ranges(theta, _theta_rgs);

        double v_theta = theta>CV_PI/2 ? theta-CV_PI/2: theta+CV_PI/2;
        add_theta_ranges(v_theta, _v_theta_rgs);
    }

    if(_theta_rgs.empty()) {
        return false;
    }
    /* merge overlaped ranges */
    merge_ranges(_theta_rgs);
    return true;

}

bool OpticalLinePredictor::predict_from_vertical(const OpticalLinePredictor & other) {
    CV_Assert(_theta_rgs.empty());
    CV_Assert(!other._theta_rgs.empty());
    _theta_rgs = other.get_vertical();
    return true;
}

