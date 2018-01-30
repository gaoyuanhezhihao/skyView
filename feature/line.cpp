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

const int RHO=0;
const int THETA=1;
bool init_frame(NewFrame & f) {
    static const int init_keyPt_thres = configs["init_keyPt_thres"];
    f.detect_lines();
    f.calc_keyPts();
    return int(f.keyPts().size()) > init_keyPt_thres;
}
void merge_ranges(vector<pair<double, double>> & ranges) {
    if(ranges.empty()) {
        return ;
    }
    std::sort(ranges.begin(), ranges.end());
    int k = 0;
    pair<double, double> cur = ranges[0];
    for(size_t i = 1; i < ranges.size(); ++i) {
        if(ranges[i].first <= cur.second) {
            cur.second = max(cur.second, ranges[i].second);
        }else {
            ranges[k++] = cur;
            cur = ranges[i];
        }
    }
    ranges[k++] = cur;
    ranges.resize(k);
}
bool intersect(const Vec2f & l1, const Vec2f & l2, Point2f & pt) {
    const double r1 = l1[0];
    const double theta1 = l1[1];
    const double c1 = cos(theta1);
    const double s1 = sin(theta1);

    const double r2 = l2[0];
    const double theta2 = l2[1];
    const double c2 = cos(theta2);
    const double s2 = sin(theta2);

    if(abs(theta1-theta2) < 0.01) {
        /* they are parallel */
        return false;
    }

    /* x */
    double denom = c1*s2-s1*c2;
    double nume = r1 * s2 - r2 * s1;
    pt.x = nume/denom;
    /* y */
    denom = (s1*c2-s2*c1);
    nume = (r1*c2-r2*c1);
    pt.y = nume/denom;
    //cout << "pt=" << pt << endl;
    return true;
}

bool get_inlier_intersects(const vector<Vec2f> & lines, vector<Point2f> & keyPts, vector<vector<int>> & line_endPt_id_map, const cv::Size & img_size) {
    const int sz = lines.size();
    cv::Point2f pt;
    //vector<Point2f> inliers;
    //line_endPt_id_map.reserve(sz);
    line_endPt_id_map = vector<vector<int>>(sz);
    for(int i = 0; i < sz; ++i) {
        for(int j = i+1; j < sz; ++ j){
            //cv::Mat tmp_img = debug_img.clone();
            //cout << "=====" << lines[i] << "---" << lines[j] << endl;
            //vector<Vec2f> tmp_lines{lines[i], lines[j]};
            //draw_lines(tmp_img, tmp_lines);
            if(intersect(lines[i], lines[j], pt) &&
                    0.0f <= pt.x && pt.x < img_size.width &&
                    0.0f <= pt.y && pt.y < img_size.height) {
                keyPts.push_back(pt);
                line_endPt_id_map[i].push_back(keyPts.size()-1);
                line_endPt_id_map[j].push_back(keyPts.size()-1);
                //vector<Point2f> tmp_pts{pt};
                //draw_points(tmp_img, tmp_pts);
            }
            //cv::imshow("debug img", tmp_img);
            //cv::waitKey(0);
        }
    }
    return true;
}
bool get_inlier_intersects(Frame & f)  {
    const cv::Size img_size = f.rgb.size();
    const int sz = f.lines.size();
    cv::Point2f pt;
    //vector<Point2f> inliers;
    //f.line_endPt_id_map.reserve(f.lines.size());
    //f.line_endPt_id_map.reserve(f.lines.size());
    f.line_endPt_id_map = vector<vector<int>>(sz);
    for(int i = 0; i < sz; ++i) {
        for(int j = i+1; j < sz; ++ j){
            //cv::Mat tmp_img = debug_img.clone();
            //cout << "=====" << lines[i] << "---" << lines[j] << endl;
            //vector<Vec2f> tmp_lines{lines[i], lines[j]};
            //draw_lines(tmp_img, tmp_lines);
            if(intersect(f.lines[i], f.lines[j], pt) &&
                    0.0f <= pt.x && pt.x < img_size.width &&
                    0.0f <= pt.y && pt.y < img_size.height) {
                f.keyPts.push_back(pt);
                f.line_endPt_id_map[i].push_back(f.keyPts.size()-1);
                f.line_endPt_id_map[j].push_back(f.keyPts.size()-1);
                //vector<Point2f> tmp_pts{pt};
                //draw_points(tmp_img, tmp_pts);
            }
            //cv::imshow("debug img", tmp_img);
            //cv::waitKey(0);
        }
    }
    return true;
}

void detect_RangHoughLine(const Mat & rgb, Mat & gray, Mat & edge, vector<Vec2f> & lines) {
    CV_Assert(!rgb.empty());
    cvtColor(rgb, gray, CV_BGR2GRAY);
    blur(gray, gray, Size(5, 5));
    Canny(gray, edge, 50, 100, 5);

}

void detect_lines(Frame & f) {
    CV_Assert(!f.rgb.empty());
    cvtColor(f.rgb, f.gray, CV_BGR2GRAY);
    blur(f.gray, f.gray, Size(5,5) );
    Canny(f.gray, f.edge, 50, 100, 5);
    //Mat cdst;
    //cvtColor(f.gray, cdst, CV_GRAY2BGR);
    //cv::imshow("edge", cdst);
    //cv::waitKey(1);
    //vector<Vec4i> linesP; 
    //HoughLinesP(f.gray, linesP, 1, CV_PI/180, 50, 50, 10 ); 
    //vector<Vec2f> lines;
    HoughLines(f.edge, f.lines, 0.5, CV_PI/180, 200, 0, 0 );
}


vector<Vec2f> merge_close_lines(vector<Vec2f> & lines) {
    static double rho_thres = configs["merge_line_rho_threshold"];
    static double theta_thres = configs["merge_line_theta_threshold"];

    if(lines.size() < 2) {
        return lines;
    }

    for(size_t i = 0; i < lines.size(); ++i) {
        if(lines[i][0] < 0.0) {
            lines[i][0] = -lines[i][0];
            lines[i][1] = lines[i][1] - CV_PI;
        }
    } 

    auto cmpor = [](const Vec2f & l1, const Vec2f & l2) {
        return l1[0] == l2[0] ? l1[1] < l2[1] : l1[0] < l2[0];
    };
    std::sort(lines.begin(), lines.end(), cmpor);
    const int sz = lines.size();
    //cout << lines[p] << "\n";
    vector<bool> keep(sz, true);
    for(int p = 0; p < sz; ++p) {
        //cout << lines[p];
        if(!keep[p]) {
            continue;
        }
        for(int c = p+1; c < sz; ++c) {
            if(!keep[c]) {
                continue;
            }
            if(abs(lines[p][RHO] - lines[c][RHO]) > rho_thres) {
                break;
            }
            if(abs(lines[p][THETA] - lines[c][THETA]) < theta_thres) {
                lines[p][RHO] = (lines[p][RHO]+lines[c][RHO])/2;
                lines[p][THETA] = (lines[p][THETA]+lines[c][THETA])/2;
                keep[c] = false;
            }
        }
    }
    vector<Vec2f> merged;
    for(int i = 0; i < sz; ++i){
        if(keep[i]) {
            merged.push_back(lines[i]);
        }
    }
    return merged;
}

double theta_from_endPoint(const Point2f & pt1, const Point2f & pt2) {
    if(pt2.y == pt1.y) {
        return 0.0;
    }
    double theta = atan((pt1.x - pt2.x)/ (pt2.y-pt1.y));
    if(theta < 0.0) {
        theta += CV_PI;
    }
    return theta;
}

double rho_from_endPoint(const Point2f & pt1, const Point2f & pt2) {
    double theta = theta_from_endPoint(pt1, pt2);
    double rho = pt1.x * cos(theta) + pt1.y * sin(theta);
    return rho;
}

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


bool range_hough(const cv::Mat & edge_im, const vector<pair<double, double>> & theta_ranges, const int threshold, vector<Vec2f> & lines) {
    static const double theta_resolution = double(configs["theta_resolution"]) * CV_PI / 180;
    static const double rho_resolution = configs["rho_resolution"];

    SHOW(theta_resolution);
    SHOW(rho_resolution);
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
    for(int t = 1; t <= numangle; ++t) {
        int max_cnt = 0;
        double rho_best = 0.0;
        for(int r = 1; r <= numrho; ++r) {
            int base = t * (numrho+2) + r;
            if(accum[base] > max_cnt) {
                max_cnt = accum[base];
                rho_best = (r-1-zero_rho_idx)* rho_resolution;
            }
        }
        cout << "theta:" << theta_vec[t-1] << ", max_cnt=" << max_cnt << ", rho_best=" << rho_best << '\n';
    }
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

bool NewFrame::calc_keyPts() {
    get_inlier_intersects(_lines, _KeyPts, _Line_endPt_id_map, _rgb.size());
    return true;
}


bool NewFrame::detect_lines(const vector<pair<double, double>> & theta_rgs) {
    static const int hough_thres = configs["hough_threshold"];
    //cout << "before range_hough line cout =" << _lines.size() << endl;
    if(!range_hough(_edge, theta_rgs, hough_thres, _lines)){
        return false;
    }
    cout << "range_hough line count="  << _lines.size() << endl;
    cout << "range hough detected lines:\n";
    for(const Vec2f & l : _lines) {
        cout << l[0] << ", " << l[1] << '\n';
    }

    _lines = merge_close_lines(_lines);
    //cout << "after merge line count = " << _lines.size() << endl;
    return true;
}

bool NewFrame::detect_lines() {
    static const int hough_thres = configs["hough_threshold"];
    //cout << "hough_thres = " << hough_thres << endl;

    HoughLines(_edge, _lines, 0.5, CV_PI/180, hough_thres, 0, 0 );
    //if(!range_hough(_edge, theta_rgs, hough_thres, _lines)){
        //return false;
    //}

    _lines = merge_close_lines(_lines);
    return true;
}
