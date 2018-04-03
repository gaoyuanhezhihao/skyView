#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <cstdio>
#include <dirent.h>
#include <errno.h>
#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <boost/format.hpp>

#include "base.hpp"
#include "Config.hpp"
#include "debug.hpp"
#include "stereo.hpp"
#include "core.hpp"


using namespace std;
using namespace cv;

void log_edge(const Frame_Interface & f) {
    static const string dst_dir = configs["result_dir"];
    static ImgLogger im_log(dst_dir, "edge_skyView");
    im_log.save(f.edge(), f.get_id());
}
cv::Mat calc_edge(cv::Mat rgb) {
    static const int blur_radius = configs["blur_radius"];
    static const int Canny_low_thres = configs["Canny_low_thres"];
    static const int Canny_high_thres = configs["Canny_high_thres"];
    static const int Canny_krn_sz= configs["Canny_krn_sz"];
    cv::Mat gray, edge;
    cvtColor(rgb, gray, CV_BGR2GRAY);
    cv::medianBlur(gray, gray, blur_radius);
    Canny(gray, edge, Canny_low_thres, Canny_high_thres, Canny_krn_sz);
    return edge;
}
void test() {
    static const string samples_dir = configs["samples"];
    static const string dst_dir = configs["result_dir"];
    static const ImgLogger track_im_log(dst_dir, "track");
    static const ImgLogger match_im_log(dst_dir, "match");
    static ImgLogger im_log(dst_dir, "edge");

    std::ofstream vo_log(dst_dir+"vo_log.txt");
    int id_start = configs["start_id"];
    const int id_last = configs["last_id"];

    for(int i = id_start; i < id_last; ++i) {
        cv::Mat raw_im = cv::imread(samples_dir + to_string(i) + ".jpg");
        SimpleFrame cur{i};
        cur.read_frame();
        log_edge(cur);
        im_log.save(calc_edge(raw_im), i);
        cout << i << "\n";
    }
}

int main(int argc, char ** argv) {
    //google::InitGoogleLogging(argv[0]);
    if( argc != 2) {
        cout <<" Error! not enough param \n Usage: runner config.txt" << endl;
        return -1;

    }
    configs.init(argv[1]);
    test();
}
