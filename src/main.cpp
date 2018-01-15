#include <dirent.h>
#include <errno.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "Config.hpp"
#include "panoramas.h"
#include "line.hpp"
#include "debug.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <fstream>

using namespace cv;
using namespace std;

bool train_model();
void test(); 
bool trsf2sky_view(cv::Mat & src_img, const Mat & H, int cols, int rows);
bool dump_mat(const string file_name, const string mat_name, const cv::Mat & mat);
bool load_mat(const string file_name, const string mat_name, cv::Mat & mat);
bool trsf2sky_view(cv::Mat & src_img, Mat & H, int cols, int rows);



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
int main(int argc, char ** argv) {
    if( argc != 2) {
        cout <<" Error! not enough param \n Usage: runner config.txt" << endl;
        return -1;

    }
    configs.init(argv[1]);
    string option =configs["option"];
    if(string("train") == option) {
        train_model();
    }else if(string("test") == option) {
        test();
    }
}

std::vector<string> get_img_pathes(string dir_path) {
    DIR *dp = opendir(dir_path.c_str());
    if(NULL == dp)  {
        cout << "Error(" << errno << ") opening" << dir_path << endl;
        exit(errno);
    }
    struct dirent *dirp = readdir(dp);
    std::vector<string> files_path;
    for( ;NULL != dirp;dirp = readdir(dp)) {
        std::string file_name=dirp->d_name;
        if(file_name == string(".")){
            continue;
        }
        if(file_name == string("..")) {
            continue;
        }
        files_path.push_back(dir_path + string("/")+ file_name);
    }
    return files_path;
}

cv::Mat get_sky_view(cv::Mat & src_img, Mat & H, int cols, int rows) {
    cv::Mat warped_sky_view;
    back_warp(H, cols, rows, src_img, warped_sky_view);
    return warped_sky_view;
}
void test() {
    string samples_dir = configs["samples"];
    int cols_sky_im = configs["cols_sky_im"];
    int rows_sky_im = configs["rows_sky_im"];
    int id_start = configs["start_id"];
    int id_last = configs["last_id"];
    Mat H;
    load_mat("H_mat.yml", "H_mat", H);

    for(int i = id_start; i<= id_last; ++i) {
        cout << i<< "\n";
        cv::Mat src_img = cv::imread(samples_dir + to_string(i) + ".jpg");
        cv::Mat warped_sky_view = get_sky_view(src_img, H, cols_sky_im, rows_sky_im);
        warped_sky_view.copyTo(debug_img);
        vector<Vec2f> lines = detect_lines(warped_sky_view);
        draw_lines(warped_sky_view, lines);
        vector<Point2f> pts = get_inlier_intersects(lines, src_img.size());
        draw_points(warped_sky_view, pts);
        cv::imshow("dst",warped_sky_view);
        cv::waitKey(0);
    }
}

bool load_mat(const string file_name, const string mat_name, cv::Mat & mat) {
    cv::FileStorage fs(file_name, cv::FileStorage::READ);
    fs[mat_name] >> mat;
    return true;
}

bool train_model(){
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
}
bool trsf2sky_view(cv::Mat & src_img, Mat & H, int cols, int rows) {

    cv::Mat warped_sky_view;
    back_warp(H, cols, rows, src_img, warped_sky_view);
    cv::imshow("warped to sky view", warped_sky_view);
    cv::imwrite("sky_view.png", warped_sky_view);
    cv::waitKey(0);
    return true;
}
bool dump_mat(const string file_name, const string mat_name, const cv::Mat & mat) {
    cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
    fs << mat_name << mat;
    return true;
}
