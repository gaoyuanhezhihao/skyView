#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <list>
#include <cstring>
#include <cstdlib>
#include "line.hpp"
#include "debug.hpp"
#include "Config.hpp"
#include "base.hpp"
#include "track.hpp"
#include "core.hpp"

using namespace cv;
using namespace std;

void add_theta_ranges(const double theta, vector<pair<double, double>> & theta_rgs) {
    static const double theta_width = double(configs["theta_predict_width"]) * CV_PI/180;
    double theta_l = theta - theta_width;
    double theta_r = theta+ theta_width;

    if(theta_l < 0.0) {
        theta_rgs.emplace_back(CV_PI + theta_l, CV_PI);
        theta_rgs.emplace_back(0.0, theta_r);
    }else if(theta_r > CV_PI) {
        theta_rgs.emplace_back(0.0, theta_r - CV_PI);
        theta_rgs.emplace_back(theta_l, CV_PI);
    }else {
        theta_rgs.emplace_back(theta_l, theta_r);
    }
}
bool predict_lines(const vector<vector<int>> & line_pts_map, Tracker & tracker, vector<pair<double, double>> & theta_rgs) {
    const vector<Point2f> & tracked_pts = tracker.get_tracked_pts();

    for(size_t i = 0; i < line_pts_map.size(); ++i) {
        /* find two points in good status */
        int pt_cnt = 0;
        int pt_ids[2] = {-1, -2};
        for(int pt_i: line_pts_map[i]) {
            if(tracker.check_status(pt_i)) {
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

        //const int u = line_pts_map[i].first;
        //const int v = line_pts_map[i].second;

        //if( !tracker.check_status(u) || !tracker.check_status(v) ) {
            //continue;
        //}
        const double theta = theta_from_endPoint(tracked_pts[u], tracked_pts[v]);
        add_theta_ranges(theta, theta_rgs);
        /* vertical line*/
        double v_theta = theta>CV_PI/2 ? theta-CV_PI/2: theta+CV_PI/2;
        add_theta_ranges(v_theta, theta_rgs);
    }

    if(theta_rgs.empty()) {
        return false;
    }
    /* merge overlaped ranges */
    merge_ranges(theta_rgs);
    return true;
}

bool predict_lines(const vector<vector<int>> & line_pts_map, Tracker & tracker, vector<pair<double, double>> & theta_rgs, vector<Vec2f> & tracked_lines) {
    const vector<Point2f> & tracked_pts = tracker.get_tracked_pts();

    for(size_t i = 0; i < line_pts_map.size(); ++i) {
        /* find two points in good status */
        int pt_cnt = 0;
        int pt_ids[2] = {-1, -2};
        for(int pt_i: line_pts_map[i]) {
            if(tracker.check_status(pt_i)) {
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
        tracked_lines.emplace_back(get_line_from_endPts(tracked_pts[u], tracked_pts[v]));

        //const int u = line_pts_map[i].first;
        //const int v = line_pts_map[i].second;

        //if( !tracker.check_status(u) || !tracker.check_status(v) ) {
            //continue;
        //}
        const double theta = theta_from_endPoint(tracked_pts[u], tracked_pts[v]);
        add_theta_ranges(theta, theta_rgs);
        /* vertical line*/
        double v_theta = theta>CV_PI/2 ? theta-CV_PI/2: theta+CV_PI/2;
        add_theta_ranges(v_theta, theta_rgs);
    }

    if(theta_rgs.empty()) {
        return false;
    }
    /* merge overlaped ranges */
    merge_ranges(theta_rgs);
    return true;
}

bool range_hough(const cv::Mat & edge_im, const vector<pair<double, double>> & theta_ranges, const int threshold, vector<Vec2f> & lines) {
    static const double theta_resolution = double(configs["theta_resolution"]) * CV_PI / 180;
    static const double rho_resolution = configs["rho_resolution"];

    //SHOW(theta_resolution);
    //SHOW(rho_resolution);
    //const string dst_dir = configs["result_dir"];
    //imwrite(dst_dir+"range_hough_debug.jpg", edge_im);
    const int width = edge_im.cols;
    const int height = edge_im.rows;
    //CvMat c_image = edge_im;
    const uchar * image = edge_im.ptr();
    //image = c_image.data.ptr;

    vector<double> theta_vec;
    for(const pair<double, double> & rg: theta_ranges)  {
        double t = rg.first;
        while(t < rg.second) {
            theta_vec.push_back(t);
            t += theta_resolution;
        }
    }

    int numrho = cvRound(((width + height) * 2 + 1) / rho_resolution);
    if(theta_vec.empty()) {
        return false;
    }

    const float irho_rsv = 1/rho_resolution;
    int numangle = theta_vec.size();
    float * tabSin = new float[numangle];
    float * tabCos = new float[numangle];
    for(int i = 0; i < numangle; ++i) {
        double theta = theta_vec[i];
        tabSin[i] = (float) (sin(theta)*irho_rsv);
        tabCos[i] = (float) (cos(theta)*irho_rsv);
    }

    int * accum = new int[(numangle+2)*(numrho+2)];
    memset(accum, 0, sizeof(accum[0]) * (numangle+2)*(numrho+2));
    //vector<int*> cnt_ptr_vec;
    //for(int i = 0; i < numangle; ++i) {
        //cnt_ptr_vec[i] = accum + numrho * (i+1);
    //}

    // stage 1. fill accumulator.
    const int step = edge_im.step;
    const int zero_rho_idx = (numrho-1)/2;
    for(int r = 0; r < height; ++r) {
        for(int c = 0; c < width; ++c) {
            if(image[r * step + c] != 0) {
                for(int i = 0; i < numangle; ++i) {
                    int rho_id = cvRound(c * tabCos[i] + r * tabSin[i]);
                    rho_id += zero_rho_idx;
                    ++accum[(i+1)* (numrho+2)+ rho_id+1];
                }
            }
        }
    }
    
    //vector<int> base_vec;
    // stage 2. find local maximums
    //float max_theta = 0.0;
    //float max_rho = 0.0;
    //double max_cnt = 0;
    for(int t = 1; t <= numangle; ++t) {
        for(int r = 1; r <= numrho; ++r) {
            int base = t * (numrho+2) + r;
            if(accum[base] > threshold &&
                    accum[base] > accum[base-1] &&
                    accum[base] > accum[base+1] &&
                    accum[base] > accum[base-numrho-2] &&
                    accum[base] > accum[base+numrho+2]) {
                float theta = theta_vec[t-1];
                float rho = (r-1-zero_rho_idx)* rho_resolution;
                lines.push_back({rho, theta});
            }
            //if(accum[base] > max_cnt) {
                //max_cnt = accum[base];
                //max_theta = theta_vec[t-1];
                //max_rho = (r-1-zero_rho_idx)* rho_resolution;
            //}
        }
    }

    /* debug */
    //for(int t = 1; t <= numangle; ++t) {
        //int max_cnt = 0;
        //double rho_best = 0.0;
        //for(int r = 1; r <= numrho; ++r) {
            //int base = t * (numrho+2) + r;
            //if(accum[base] > max_cnt) {
                //max_cnt = accum[base];
                //rho_best = (r-1-zero_rho_idx)* rho_resolution;
            //}
        //}
        //cout << "theta:" << theta_vec[t-1] << ", max_cnt=" << max_cnt << ", rho_best=" << rho_best << '\n';
    //}
    /* ------*/
    //cout << "range_hough: \n"; 
    //SHOW(max_cnt);
    //SHOW(max_theta);
    //SHOW(max_rho);
    //for(int ro = 0; ro < numrho; ++ro) {
        //for(int th = 0; th < numangle; ++th) {
            //int base = (th+1) * (numrho+2) + (ro+1);
            //if(accum[base] > threshold &&
                    //accum[base] > accum[base-1] &&
                    //accum[base] > accum[base+1] &&
                    //accum[base] > accum[base-numrho-2] &&
                    //accum[base] > accum[base+numrho+2]) {
                ////base_vec.push_back(base);
                //float theta = theta_vec[th];
                //float rho = (ro - zero_rho_idx) * rho_resolution;
                //lines.push_back({rho, theta});
            //}
        //}
    //}

    //const double scale = 1./(numrho+2);
    //for(int base : base_vec) {
        //int theta_id = cvFloor(base * scale) - 1;
        //int rho_id = base - (theta_id+1) * (numrho+2) -1;
        //float theta = theta_vec[theta_id];
        //float rho = (rho_id - zero_rho_idx) * rho_resolution;
        //lines.push_back({rho, theta});
    //}
    delete[] tabCos;
    delete[] tabSin;
    delete[] accum;
    return true;
}


