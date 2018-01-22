#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <list>
#include "line.hpp"
#include "debug.hpp"
#include "Config.hpp"
#include "base.hpp"
#include "track.hpp"

using namespace cv;
using namespace std;

const int RHO=0;
const int THETA=1;
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

bool get_inlier_intersects(Frame & f)  {
    const cv::Size img_size = f.rgb.size();
    const int sz = f.lines.size();
    cv::Point2f pt;
    //vector<Point2f> inliers;
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
bool RangeHoughLine(const Mat & edge, const double rho_resolution, 
        const double theta_resolution) {

}

void detect_lines(Frame & f) {
    CV_Assert(!f.rgb.empty());
    cvtColor(f.rgb, f.gray, CV_BGR2GRAY);
    blur(f.gray, f.gray, Size(5,5) );
    Canny(f.gray, f.gray, 50, 100, 5);
    //Mat cdst;
    //cvtColor(f.gray, cdst, CV_GRAY2BGR);
    //cv::imshow("edge", cdst);
    //cv::waitKey(1);
    //vector<Vec4i> linesP; 
    //HoughLinesP(f.gray, linesP, 1, CV_PI/180, 50, 50, 10 ); 
    //vector<Vec2f> lines;
    HoughLines(f.gray, f.lines, 0.5, CV_PI/180, 200, 0, 0 );
}


vector<Vec2f> merge_close_lines(vector<Vec2f> & lines) {
    static double rho_thres = configs["merge_line_rho_threshold"];
    static double theta_thres = configs["merge_line_theta_threshold"];

    if(lines.size() < 2) {
        return lines;
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
    return atan((pt1.x - pt2.x)/ (pt2.y-pt1.y));
}


bool predict_lines(vector<pair<int, int>> & line_pts_map, Tracker & tracker, vector<pair<double, double>> & theta_rgs) {
    static const double theta_width = double(configs["theta_width"]) * CV_PI/180;
    const vector<Point2f> & tracked_pts = tracker.get_tracked_pts();

    for(size_t i = 0; i < line_pts_map.size(); ++i) {
        const int u = line_pts_map[i].first;
        const int v = line_pts_map[i].second;

        if( !tracker.check_status(u) || !tracker.check_status(v) ) {
            continue;
        }
        const double theta = theta_from_endPoint(tracked_pts[u], tracked_pts[v]);
        double theta_l = max(0.0, theta - theta_width);
        double theta_r = min(2*CV_PI, theta+ theta_width);
        theta_rgs.emplace_back(theta_l, theta_r);
    }

    if(theta_rgs.empty()) {
        return false;
    }
    /* merge overlaped ranges */
    std::sort(theta_rgs.begin(), theta_rgs.end());
    int k = 0;
    pair<double, double> cur = theta_rgs[0];
    for(size_t i = 1; i < theta_rgs.size(); ++i) {
        if(theta_rgs[i].first <= cur.second) {
            cur.second = max(cur.second, theta_rgs[i].second);
        }else {
            theta_rgs[k++] = cur;
            cur = theta_rgs[i];
        }
    }
    return true;
}
