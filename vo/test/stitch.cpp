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

#include "Config.hpp"
#include "debug.hpp"
#include "vo.hpp"
#include "line.hpp"
#include "stereo.hpp"

using namespace cv;
using namespace std;

void preprocess(Frame & f) {
    detect_lines(f);
    f.lines = merge_close_lines(f.lines);
    get_inlier_intersects(f);
}

void test() {
    const string dst_dir = configs["result_dir"];
    const int id_start = configs["start_id"];
    const int id_last = configs["last_id"];
    //std::ofstream out(dst_dir+"cout.txt");
    //std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    //std::cout.rdbuf(out.rdbuf()); 
    
    Frame prev;
    read_frame(id_start, prev);
    preprocess(prev);
    for(int i = id_start+1; i<= id_last; ++i) {
        cout << i<< "\n";
        printf("%d\n", i);
        Frame cur;
        read_frame(i, cur);
        preprocess(cur);
        //cv::imshow("prev", draw_frame(prev));
        //cv::imshow("cur", draw_frame(cur));
        waitKey(0);
        shared_ptr<Match> pm = match_keyPoints(prev, cur);
        cv::Mat imgMatches;

        swap(prev, cur);
    }
}

int main(int argc, char ** argv) {
    if( argc != 2) {
        cout <<" Error! not enough param \n Usage: runner config.txt" << endl;
        return -1;

    }
    configs.init(argv[1]);
    test();
}
