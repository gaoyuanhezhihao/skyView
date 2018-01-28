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

#include "base.hpp"
#include "Config.hpp"
#include "debug.hpp"
#include "line.hpp"
#include "stereo.hpp"
#include "track.hpp"
#include "core.hpp"

using namespace cv;
using namespace std;

//void read_frame(const int i, Frame & f) {
    //static const string samples_dir = configs["samples"];
    //static const string dst_dir = configs["result_dir"];
    //static const int cols_sky_im = configs["cols_sky_im"];
    //static const int rows_sky_im = configs["rows_sky_im"];

    //f.rgb = cv::imread(samples_dir + to_string(i) + ".jpg");
    //f.rgb = get_sky_view(f.rgb, cols_sky_im, rows_sky_im);
//}

//void preprocess(Frame & f) {
    //detect_lines(f);
    //f.lines = merge_close_lines(f.lines);
    //get_inlier_intersects(f);
//}

void test() {
    static const string samples_dir = configs["samples"];
    const string dst_dir = configs["result_dir"];
    const int id_start = configs["start_id"];
    const int id_last = configs["last_id"];

    NewFrame prevFrame{id_start};
    prevFrame.read_frame();
    prevFrame.detect_lines();
    prevFrame.calc_keyPts();
    SHOW(prevFrame.lines().size());
    SHOW(prevFrame.keyPts().size());
    int fail_track_cnt = 0;
    for(int i = id_start+1; i <= id_last; ++i) {
        cout << prevFrame.get_id() << "--" << i << endl;
        NewFrame cur{i};
        cur.read_frame();

        printf("%d\n", i);
        cout << "tracking" << endl;
        Tracker tk(prevFrame.keyPts(), prevFrame.edge(),
                cur.edge());
        if(! tk.run()){
            ++ fail_track_cnt;
            continue;
        }
        fail_track_cnt = 0;
        cv::Mat imgTrack = tk.draw();
        imwrite(dst_dir+to_string(i-1) + "--" + to_string(i) + "_track.jpg", imgTrack);
        //cv::imshow("track result", imgTrack);
        //waitKey(0);

        cout << "predicting" << endl;
        vector<pair<double, double>> theta_rgs;
        if(!predict_lines(prevFrame.line_endPt_id_map(), tk, theta_rgs)){
            continue;
        }
        for(pair<double, double> & rg: theta_rgs) {
            cout << rg.first << ", " << rg.second << "\n";
        }




        cout << "detecting lines" << endl;
        if(!cur.detect_lines(theta_rgs)) {
            continue;
        }
        cout << "count of lines=" << cur.lines().size() << endl;

        cout << "calc keyPts" << endl;
        if(!cur.calc_keyPts()) {
            continue;
        }
        SHOW(cur.keyPts().size());


        cout << " matching " << endl;
        shared_ptr<NewMatch> pMch = match_pts(tk, prevFrame, cur);
        if(nullptr == pMch) {
            continue;
            cout << "fail match\n";
        }
        cout << "success match\n";

        cv::Mat img_mch = pMch->draw();
        cv::imwrite(dst_dir+to_string(i-1) + "--" + to_string(i) + ".jpg", img_mch);
        swap(prevFrame, cur);
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

//std::vector<string> get_img_pathes(string dir_path) {
    //DIR *dp = opendir(dir_path.c_str());
    //if(NULL == dp)  {
        //cout << "Error(" << errno << ") opening" << dir_path << endl;
        //exit(errno);
    //}
    //struct dirent *dirp = readdir(dp);
    //std::vector<string> files_path;
    //for( ;NULL != dirp;dirp = readdir(dp)) {
        //std::string file_name=dirp->d_name;
        //if(file_name == string(".")){
            //continue;
        //}
        //if(file_name == string("..")) {
            //continue;
        //}
        //files_path.push_back(dir_path + string("/")+ file_name);
    //}
    //return files_path;
//}
