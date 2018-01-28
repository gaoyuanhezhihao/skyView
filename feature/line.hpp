#ifndef LINE_H
#define LINE_H
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "base.hpp"
#include "track.hpp"

using namespace cv;
using namespace std;
void detect_lines(Frame & f);



bool get_inlier_intersects(Frame & f);
vector<Vec2f> merge_close_lines(vector<Vec2f> & lines);
double theta_from_endPoint(const Point2f & pt1, const Point2f & pt2);
double rho_from_endPoint(const Point2f & pt1, const Point2f & pt2);
bool range_hough(const cv::Mat & edge_im, const vector<pair<double, double>> & theta_ranges, const int threshold, vector<Vec2f> & lines);
void merge_ranges(vector<pair<double, double>> & ranges);
bool predict_lines(const vector<vector<int>> & line_pts_map, Tracker & tracker, vector<pair<double, double>> & theta_rgs);
bool get_inlier_intersects(const vector<Vec2f> & lines, vector<Point2f> & keyPts, vector<vector<int>> & line_endPt_id_map, cv::Size & img_size);
#endif //LINE_H
