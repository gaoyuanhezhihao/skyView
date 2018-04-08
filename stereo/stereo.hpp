#ifndef STEREO_H
#define STEREO_H

#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "config/Config.hpp"
using namespace std;
using namespace cv;

cv::Mat get_sky_view(cv::Mat & src_img, int cols, int rows);
cv::Mat calibrate();
cv::Mat calibrate(string H_mat_path, Mat & src_img, vector<Point2f> mark_points, vector<Point2f> sky_view_pts);
#endif //STEREO_H
