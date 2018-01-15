#include <iostream>
#include <string>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "panoramas.h"
#include "Config.hpp"
#include "base.hpp"
using namespace cv;
using namespace std;
cv::Mat load_mat(const string file_name, const string mat_name) {
    cv::Mat mat;
    cv::FileStorage fs(file_name, cv::FileStorage::READ);
    fs[mat_name] >> mat;
    return mat;
}
cv::Mat get_sky_view(cv::Mat & src_img, int cols, int rows) {
    static Mat H = load_mat("H_mat.yml", "H_mat");
    cv::Mat warped_sky_view;
    back_warp(H, cols, rows, src_img, warped_sky_view);
    return warped_sky_view;
}
bool read_mark_pts(const string file_path, vector<cv::Point2f> & vec){
    ifstream file(file_path);
    if(file.is_open()) {
        string line;
        while(std::getline(file, line)) {
            std::istringstream iss(line);
            double x = -1;
            double y = -1;
            iss >> x;
            iss >> y;
            vec.push_back(Point2f(x, y));
        }
    }
    return true;
}
bool dump_mat(const string file_name, const string mat_name, const cv::Mat & mat) {
    cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
    fs << mat_name << mat;
    return true;
}
bool trsf2sky_view(cv::Mat & src_img, Mat & H, int cols, int rows) {

    cv::Mat warped_sky_view;
    back_warp(H, cols, rows, src_img, warped_sky_view);
    cv::imshow("warped to sky view", warped_sky_view);
    cv::imwrite("sky_view.png", warped_sky_view);
    cv::waitKey(0);
    return true;
}


bool calibrate(){
    /* --read configurations-- */
    string img_src_path = configs["src_img_path"];
    string result_prefix = configs["result_prefix"];
    string mark_pts_path = configs["mark_points"];
    string sky_view_pts_path = configs["sky_view_pts_path"];

    vector<cv::Point2f> mark_points;
    vector<cv::Point2f> sky_view_pts;
    read_mark_pts(mark_pts_path, mark_points);
    read_mark_pts(sky_view_pts_path, sky_view_pts);
    int * p_sample_ids = new int[mark_points.size()];
    for (size_t i = 0; i< mark_points.size(); ++i) {
        p_sample_ids[i] = i;
    }
    Mat H;
    H = get_the_rotation_param(mark_points, sky_view_pts, p_sample_ids, mark_points.size());
    Mat src_img = imread(img_src_path);
    int cols_sky_im = configs["cols_sky_im"];
    int rows_sky_im = configs["rows_sky_im"];
    trsf2sky_view(src_img, H, cols_sky_im, rows_sky_im);
    dump_mat("H_mat.yml", "H_mat", H);
    return true;
}
