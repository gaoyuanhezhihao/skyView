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
