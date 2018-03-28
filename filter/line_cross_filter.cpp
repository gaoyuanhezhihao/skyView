#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

const int RHO=0;
const int THETA=1;
const double PI_2 = CV_PI/2;

static double angle_of_vecs(const Point & vec1, const Point & vec2) {
    double nume = vec1.dot(vec2);
    double denom = cv::norm(vec1) * cv::norm(vec2);
    return std::acos(nume/denom);
}

static double perpendicular_ratio(const Vec2f & l1, const Vec2f & l2) {
    Point vec1(cos(l1[THETA]), sin(l1[THETA]));
    Point vec2(cos(l2[THETA]), sin(l2[THETA]));

    const double agl = angle_of_vecs(vec1, vec2);

    return PI_2 - abs(PI_2 - agl);
}

int filter_by_line_cross(vector<Vec2f> & h_lines, vector<Vec2f> & v_lines) {
    static  const double PPR_THRES = get_param("perpendicular_thres");
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

    vector<int> good_h_ids;
    for(int i = 0; i < h_sz; ++i) {
        if(ppr_h[i] > PPR_THRES) {
            good_h_ids.push_back(i);
        }
    }
    vector<int> good_v_ids;
    for(int i = 0; i < v_sz; ++i) {
        if(ppr_v[i] > PPR_THRES) {
            good_v_ids.push_back(i);
        }
    }
}
