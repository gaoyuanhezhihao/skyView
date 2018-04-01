#ifndef RANGE_HOUGH_H
#define RANGE_HOUGH_H


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "track.hpp"

#include <iostream>
#include <vector>
bool predict_lines(const std::vector<std::vector<int>> & line_pts_map, Tracker & tracker, std::vector<std::pair<double, double>> & theta_rgs);

bool range_hough(const cv::Mat & edge_im, const std::vector<std::pair<double, double>> & theta_ranges, const int threshold, std::vector<cv::Vec2f> & lines);
bool predict_lines(const vector<vector<int>> & line_pts_map, Tracker & tracker, vector<pair<double, double>> & theta_rgs, vector<Vec2f> & tracked_lines);
void add_theta_ranges(const double theta, vector<pair<double, double>> & theta_rgs);
#endif //RANGE_HOUGH_H
