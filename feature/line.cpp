#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <list>
#include <cstring>
#include <cstdlib>
#include <limits>
#include "line.hpp"
#include "debug.hpp"
#include "config/Config.hpp"
#include "base.hpp"
//#include "track.hpp"
#include "core.hpp"
#include "line_cross_filter.hpp"

using namespace cv;
using namespace std;

const int RHO=0;
const int THETA=1;
const double MIN_DOUBLE = std::numeric_limits<double>::min();


bool SimpleFrame::range_hough(const vector<pair<double, double>> & h_theta_rgs, const vector<pair<double, double>> & v_theta_rgs) {
    static const int hough_thres = configs["hough_threshold"];

    if(!::range_hough(_edge, h_theta_rgs, hough_thres, _hl)){
        return false;
    }
    if(!::range_hough(_edge, v_theta_rgs, hough_thres, _vl)) {
        return false;
    }
    if(_hl.empty() || _vl.empty()) {
        return false;
    }
    return true;
}

bool total_hough(const int hough_thres, const Mat & edge, vector<Vec2f> & rst) {
    static double theta_resolution = configs["theta_resolution"];
    static double rho_resolution = configs["rho_resolution"];

    HoughLines(edge, rst, rho_resolution, theta_resolution*CV_PI/180, hough_thres, 0, 0 );
    //cout << "HoughLines:" << rst.size() << endl;
    //if(!range_hough(_edge, theta_rgs, hough_thres, _lines)){
        //return false;
    //}
    //_lines = merge_close_lines(_lines);
    return true;
}

bool find_perpendicualr(const vector<Vec2f> & lines, vector<double> & ppr_max, array<Point2f, 2> & dir_vec) {
    static  const double PPR_THRES = double(get_param("perpendicular_thres")) * CV_PI /180;
    CV_Assert(lines.size() == ppr_max.size());
    const int sz = lines.size();
    double best_ppr = 0.0;
    array<int, 2> best_ppr_ids{{-1, -1}};
    for(int i = 0; i < sz; ++i) {
        for(int j = i + 1; j < sz; ++j)  {
            double ppr = perpendicular_ratio(lines[i], lines[j]);
            if(ppr > ppr_max[i]) {
                ppr_max[i] = ppr;
            }

            if(ppr > ppr_max[j]) {
                ppr_max[j] = ppr;
            }
            if(ppr > best_ppr) {
                best_ppr = ppr;
                best_ppr_ids[0] = i;
                best_ppr_ids[1] = j;
            }
        }
    }

    //SHOW(best_ppr);
    //SHOW(PPR_THRES);
    if(best_ppr < PPR_THRES) {
        return false;
    }
    dir_vec[0] = vec_of_line(lines[best_ppr_ids[0]]);
    dir_vec[1] = vec_of_line(lines[best_ppr_ids[1]]);
    return true;
}

void rm_noise_line(vector<Vec2f> & lines, vector<double> & ppr) {
    static  const double PPR_THRES = double(get_param("perpendicular_thres")) * CV_PI /180;
    const int sz = lines.size();
    //vector<double> ppr_max(sz, MIN_DOUBLE);
    //array<Point2f, 2> dir_vec;
    //if(!find_perpendicualr(lines, ppr_max, dir_vec)) {
        //return false;
    //}

    int p = 0;
    for(int q = 0; q < sz; ++q) {
        if(ppr[q] > PPR_THRES) {
            lines[p] = lines[q];
            ppr[p] = ppr[q];
            ++p;
        }
    }
    lines.resize(p);
}

void classify_lines(const vector<Vec2f> & lines,
        vector<Vec2f> & set1, vector<Vec2f> & set2,
        const array<Point2f, 2> & dir_vec){
    //CV_Assert(set1.empty());
    //CV_Assert(set2.empty());

    for(const Vec2f & l: lines) {
        Point2f v = vec_of_line(l);
        if(angle_of_vecs(v, dir_vec[0]) <\
                angle_of_vecs(v, dir_vec[1])) {
            set1.push_back(l);
        }else {
            set2.push_back(l);
        }
    }
}

bool SimpleFrame::init() {
    static const int hough_thres = configs["hough_threshold"];
    vector<Vec2f> lines;
    total_hough(hough_thres, _edge, lines);
    const int sz = lines.size();
    array<Point2f, 2> dir_vec;
    vector<double> ppr(sz, MIN_DOUBLE);
    if(!find_perpendicualr(lines, ppr, dir_vec)) {
        return false;
    }
    //debug_show_img(_rgb.clone(), lines, "find_perpendicualr");
    rm_noise_line(lines, ppr);
    //debug_show_img(_rgb.clone(), lines, "rm_noise_line");
    classify_lines(lines, _hl, _vl, dir_vec);
    //imshow("classify_lines", draw_lines());
    //waitKey(0);
    SHOW(_hl.size());
    SHOW(_vl.size());
    filter_by_line_cross(_edge.size(), _hl, _vl);
    //imshow("filter_by_line_cross", draw_lines());
    //waitKey(0);
    /*TODO dec hough thre if fail or inc if too many */
    return calc_keyPts();
}

Mat SimpleFrame::draw_lines() const {
    cv::Mat line_img = _rgb.clone();
    ::draw_lines(line_img, _hl, GREEN);
    ::draw_lines(line_img, _vl, BLUE);
    return line_img;
}

void SimpleFrame::merge_old_hl(const vector<Vec2f> & old_hl) {
    if(!_hl.empty()) {
        return;
    }
    for(const Vec2f & l: old_hl) {
        _hl.push_back(l);
    }
}

void SimpleFrame::merge_old_vl(const vector<Vec2f> & old_vl) {
    if(!_vl.empty()) {
        return;
    }
    for(const Vec2f & l: old_vl) {
        _vl.push_back(l);
    }
}

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

bool check_vertical(const Vec2f & l1, const Vec2f & l2) {
    static const double _thres = configs["vertical_theta_thres"];
    return abs(abs(l1[1] - l2[1]) - CV_PI/2)< _thres;
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

bool SimpleFrame::calc_keyPts() {
    static const int init_keyPt_thres = get_param("init_keyPt_thres");
    static  const double PPR_THRES = double(get_param("perpendicular_thres")) * CV_PI /180;
    const int sz1 = _hl.size();
    const int sz2 = _vl.size();
    _hl_pt_map = decltype(_hl_pt_map)(sz1);
    _vl_pt_map = decltype(_vl_pt_map)(sz2);

    Point2f pt;
    const int width = _edge.cols;
    const int height = _edge.rows;
    for(int i = 0; i < sz1; ++i) {
        for(int j = 0; j < sz2; ++j) {
            if(PPR_THRES <= perpendicular_ratio(_hl[i], _vl[j])) {
                if(intersect(_hl[i], _vl[j], pt) &&
                    0.0f <= pt.x && pt.x < width &&
                    0.0f <= pt.y && pt.y < height) {
                    _pts.push_back(pt);
                    _lmk_ids.push_back(-1);
                    _hl_pt_map[i].push_back(_pts.size()-1);
                    _vl_pt_map[j].push_back(_pts.size()-1);
                }
            }
        }
    }
    return _pts.size() >= size_t(init_keyPt_thres);
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
            if(!check_vertical(lines[i], lines[j])) {
                continue;
            }
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

void NewFrame::merge_tracked_lines(const std::vector<cv::Vec2f> & tracked_lines) {
    for(const Vec2f & l: tracked_lines) {
        _lines.push_back(l);
    }
    _lines = merge_close_lines(_lines);
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


Vec2f get_line_from_endPts(const Point2f & pt1, const Point2f & pt2) {
    return Vec2f{float(rho_from_endPoint(pt1, pt2)), float(theta_from_endPoint(pt1, pt2))};
}


bool NewFrame::calc_keyPts() {
    get_inlier_intersects(_lines, _KeyPts, _Line_endPt_id_map, _rgb.size());
    _pMatchFrames = vector<NewFrame*>(_KeyPts.size(), nullptr);
    _match_keyPt_ids = vector<int> (_KeyPts.size(), -1);
    return true;
}


bool NewFrame::detect_lines(const vector<pair<double, double>> & theta_rgs) {
    static const int hough_thres = configs["hough_threshold"];
    //cout << "before range_hough line cout =" << _lines.size() << endl;
    if(!range_hough(_edge, theta_rgs, hough_thres, _lines)){
        return false;
    }
    cout << "range_hough line count="  << _lines.size() << endl;
    //cout << "range hough detected lines:\n";
    //for(const Vec2f & l : _lines) {
        //cout << l[0] << ", " << l[1] << '\n';
    //}

    _lines = merge_close_lines(_lines);
    //cout << "after merge line count = " << _lines.size() << endl;
    return true;
}

bool NewFrame::detect_lines(const int hough_thres) {
    static double theta_resolution = configs["theta_resolution"];
    static double rho_resolution = configs["rho_resolution"];

    HoughLines(_edge, _lines, rho_resolution, theta_resolution*CV_PI/180, hough_thres, 0, 0 );
    cout << "HoughLines:" << _lines.size() << endl;
    //if(!range_hough(_edge, theta_rgs, hough_thres, _lines)){
        //return false;
    //}

    _lines = merge_close_lines(_lines);
    return true;
}
bool NewFrame::detect_lines() {
    static const int hough_thres = configs["hough_threshold"];
    //cout << "hough_thres = " << hough_thres << endl;
    static double theta_resolution = configs["theta_resolution"];
    static double rho_resolution = configs["rho_resolution"];

    HoughLines(_edge, _lines, rho_resolution, theta_resolution*CV_PI/180, hough_thres, 0, 0 );
    cout << "HoughLines:" << _lines.size() << endl;
    //if(!range_hough(_edge, theta_rgs, hough_thres, _lines)){
        //return false;
    //}

    _lines = merge_close_lines(_lines);
    return true;
}

void SimpleFrame::filter_line() {
    filter_by_line_cross(_edge.size(), _hl, _vl);
}

void SimpleFrame::rm_extra_line() {
    keep_best_two(_hl, _vl);
}
