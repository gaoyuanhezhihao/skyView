#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <cstdio>
#include <dirent.h>
#include <errno.h>
#include <memory>
#include <list>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "base.hpp"
#include "Config.hpp"
#include "debug.hpp"
#include "vo.hpp"
#include "line.hpp"
#include "stereo.hpp"

using namespace cv;
using namespace std;
std::streambuf *coutbuf = nullptr;
std::shared_ptr<std::ofstream> p_out = nullptr;
void preprocess(Frame & f) {
    detect_lines(f);
    f.lines = merge_close_lines(f.lines);
    get_inlier_intersects(f);
}

void redirect_cout() {
    const string dst_dir = configs["result_dir"];
    //std::ofstream out(dst_dir+"cout.txt");
    p_out = std::make_shared<std::ofstream>(dst_dir+"cout.txt");
    coutbuf= std::cout.rdbuf(); //save old buf
    std::cout.rdbuf(p_out->rdbuf()); 
}

void set_cout_default() {
    std::cout.rdbuf(coutbuf);
}

void test() {
    const string dst_dir = configs["result_dir"];
    const int id_start = configs["start_id"];
    const int id_last = configs["last_id"];
    const bool show_match = configs["show_match"];
    char buf[80] = {0};
    redirect_cout();

    std::vector<Frame> frame_chain;
    Frame first(id_start);
    read_frame(id_start, first);
    preprocess(first);
    frame_chain.push_back(first);
    for(int i = id_start+1; i<= id_last; ++i) {
        Frame & prev = frame_chain.back();
        cout << i<< "\n";
        printf("%d\n", i);
        Frame cur(i);
        read_frame(i, cur);
        preprocess(cur);
        //cv::imshow("prev", draw_frame(prev));
        //cv::imshow("cur", draw_frame(cur));
        //waitKey(0);
        shared_ptr<Match> pm = match_keyPoints(prev, cur);
        cv::Mat imgMatches;
        if(show_match) {
            draw_matches(*pm, imgMatches);
            std::sprintf(buf, "%d--%d.jpg", prev.id, cur.id);
            cv::imwrite(dst_dir + buf, imgMatches);
            //cv::imshow("match", imgMatches);
            //cv::waitKey(0);
        }

        get_motion(*pm);
        cur.R_global = pm->R.inv() * prev.R_global;
        cur.t_global = pm->t + prev.t_global;
        cout << "R=" << pm->R.inv() << "\n";
        cout << "t=" << pm->t << "\n";
        cout << "cur.R= " << cur.R_global << "\n";
        cout << "cur.t= " << cur.t_global << "\n";
        frame_chain.push_back(cur);
    }
    set_cout_default();
}

int main(int argc, char ** argv) {
    if( argc != 2) {
        cout <<" Error! not enough param \n Usage: runner config.txt" << endl;
        return -1;

    }
    configs.init(argv[1]);
    test();
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
