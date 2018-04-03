#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Config.hpp"
#include "base.hpp"
#include "debug.hpp"

using namespace std;
using namespace cv;

const int RHO=0;
const int THETA=1;
const double PI_2 = CV_PI/2;

inline double solve_y(const int x, const double rho, const double cos_theta, const double sin_theta) {
    return (rho - x*cos_theta)/sin_theta;
}
inline double solve_x(const int y, const double rho, const double cos_theta, const double sin_theta) {
    return (rho - y*sin_theta)/cos_theta;
}
vector<Point> line_endPoint_in_img(const cv::Size & img_size, const Vec2f & line) {
    const int cols = img_size.width;
    const int rows = img_size.height;
    const double rho = line[RHO];
    const double theta = line[THETA];
    const double cos_theta = cos(theta);
    const double sin_theta = sin(theta);
    vector<Point> candi{{0, -1}, {cols-1, -1}, {-1, 0}, {-1, rows-1}}; 
    vector<Point> rst;
    //cout << "-----\n";
    for(Point & pt: candi) {
        if(pt.x == -1) {
            pt.x = solve_x(pt.y, rho, cos_theta, sin_theta);
            //SHOW(pt.x);
            if(0 <= pt.x && pt.x <=cols+1) {
                rst.push_back(pt);
            }
        }else {
            pt.y = solve_y(pt.x, rho, cos_theta, sin_theta);
            //SHOW(pt.y);
            if(0 <= pt.y && pt.y <=rows+1) {
                rst.push_back(pt);
            }
        }
        if(rst.size() == 2) {
            break;
        }
    }
    //SHOW(rst.size());
    CV_Assert(2 == rst.size());
    return rst;
}
double angle_of_vecs(const Point2f & vec1, const Point2f & vec2) {
    double nume = vec1.dot(vec2);
    double denom = cv::norm(vec1) * cv::norm(vec2);
    return std::acos(nume/denom);
}

Point2f vec_of_line(const Vec2f & l) {
    return {sin(l[THETA]), -cos(l[THETA])};
}
double perpendicular_ratio(const Vec2f & l1, const Vec2f & l2) {
    Point2f vec1 = vec_of_line(l1);
    Point2f vec2 = vec_of_line(l2);

    const double agl = angle_of_vecs(vec1, vec2);
    return PI_2 - abs(PI_2 - agl);
}

static void keep_perpendicular_line(vector<Vec2f> & lines, vector<double> & pprs) {
    static  const double PPR_THRES = double(get_param("perpendicular_thres")) * CV_PI/180;
    CV_Assert(lines.size() == pprs.size());
    const int sz = pprs.size();
    vector<Vec2f> good_lines;
    vector<double> good_pprs;
    for(int i = 0; i < sz; ++i) {
        if(pprs[i] > PPR_THRES) {
            good_lines.push_back(lines[i]);
            good_pprs.push_back(pprs[i]);
        }
    }
    lines = good_lines;
    pprs = good_pprs;
}

bool is_close_line(const cv::Size img_sz, const Vec2f & l1, const Vec2f & l2) {
    static const int THRES = get_param("close_line_dist_thres");
    vector<Point> endPt1 = line_endPoint_in_img(img_sz, l1);
    //SHOW(endPt1.size());
    CV_Assert(2 == endPt1.size());
    return dist_pt2line(l2, endPt1[0]) < THRES || dist_pt2line(l2, endPt1[1]) < THRES;
    //double d_sum = dist_pt2line(l2, endPt1[0]) + dist_pt2line(l2, endPt1[1]);
    //return d_sum < THRES;
}

void remove_close(vector<Vec2f> & lines, const vector<double> & ppr, const Size img_sz) {
    CV_Assert(lines.size() == ppr.size());
    const int sz = lines.size();
    vector<bool> keep(sz, true);
    for(int i = 0; i < sz; ++i) {
        if(!keep[i]) {
            continue;
        }
        for(int j = i+1; j < sz; ++j) {
            if(!keep[j]) {
                continue;
            }
            if(is_close_line(img_sz, lines[i], lines[j])) {
                if(ppr[i] >= ppr[j]) {
                    keep[j] = false;
                }else {
                    keep[i] = false;
                    break;
                }
            }
        }
    }

    auto p = lines.begin();
    auto q = lines.begin();
    auto kp= keep.cbegin();
    for(; q != lines.end(); ++q, ++kp) {
        if(*kp) {
            *p++ = *q;
        }
    }
    lines.resize(p-lines.begin());
    return;
}

void filter_by_line_cross(Size img_sz, vector<Vec2f> & h_lines, vector<Vec2f> & v_lines) {
    vector<double> ppr_h(h_lines.size(), 0);
    vector<int> match_h(h_lines.size(), -1);

    vector<double> ppr_v(v_lines.size(), 0);
    vector<int> match_v(v_lines.size(), -1);

    const int h_sz = h_lines.size();
    const int v_sz = v_lines.size();
    for(int i = 0; i < h_sz; ++i ) {
        for(int j = 0; j < v_sz; ++j) {
            
            const double ppr = perpendicular_ratio(h_lines[i], v_lines[j]);
            if(ppr > ppr_h[i]) {
                ppr_h[i] = ppr;
                match_h[i] = j;
            }

            if(ppr > ppr_v[j]) {
                ppr_v[j] = ppr;
                match_v[j] = i;
            }
        }
    }

    keep_perpendicular_line(h_lines, ppr_h);
    keep_perpendicular_line(v_lines, ppr_v);
    remove_close(h_lines, ppr_h, img_sz);
    remove_close(v_lines, ppr_v, img_sz);

}
