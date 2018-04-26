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
#include "config/Config.hpp"
#include "debug.hpp"
#include "stereo.hpp"
#include "core.hpp"
#include "edge_filter.hpp"


using namespace std;
using namespace cv;

template <const char * subfix>
void log_line_img(const SimpleFrame & f) {
    static const string dst_dir = configs["result_dir"];
    static ImgLogger im_log(dst_dir, string("line_")+subfix);
    Mat line_img = f.draw_lines();
    im_log.save(line_img, f.get_id());
}

constexpr const char before[] = "before";
constexpr const char after[] = "after";
constexpr const char filtered[] = "filtered";

template <const char * subfix>
void log_edge(const Frame_Interface & f) {
    static const string dst_dir = configs["result_dir"];
    static ImgLogger im_log(dst_dir, string("edge_")+subfix);
    im_log.save(f.edge(), f.get_id());
}

void test() {
    static const string samples_dir = configs["samples"];
    static const string dst_dir = configs["result_dir"];
    std::ofstream vo_log(dst_dir+"vo_log.txt");
    int id_start = configs["start_id"];
    const int id_last = configs["last_id"];

    //redirect_cout();

    for(int i = id_start; i <= id_last; ++i) {
        SimpleFrame cur{i};
        cur.read_frame();

        log_edge<before>(cur);
        imshow("edge_before", cur.edge());
        cur.filter_edge();
        log_edge<after>(cur);
        imshow("edge_after", cur.edge());
        waitKey(10);

        cur.init();
        log_line_img<before>(cur);
    }
    //set_cout_default();
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

