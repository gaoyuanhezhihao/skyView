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

//#include <glog/logging.h>

#include "base.hpp"
#include "Config.hpp"
#include "debug.hpp"
#include "line.hpp"
#include "stereo.hpp"
#include "track.hpp"
#include "core.hpp"
#include "range_hough.hpp"

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

void log_line_img(const NewFrame & f) {
    static const string dst_dir = configs["result_dir"];
    cv::Mat line_img = f.rgb();
    draw_lines(line_img, f.lines(), GREEN);
    imwrite(dst_dir+"line_"+to_string(f.get_id())+".jpg", line_img);
}

void log_keyPt_img(const NewFrame & f) {
    static const string dst_dir = configs["result_dir"];
    cv::Mat keyPt_img = f.rgb();
    draw_points(keyPt_img, f.keyPts());
    imwrite(dst_dir+"keyPt_"+to_string(f.get_id()) + ".jpg", keyPt_img);
}
void test() {
    static const int init_keyPt_thres = configs["init_keyPt_thres"];
    static const string samples_dir = configs["samples"];
    const string dst_dir = configs["result_dir"];
    const int id_start = configs["start_id"];
    const int id_last = configs["last_id"];

    redirect_cout();
    NewFrame prevFrame{id_start};
    prevFrame.read_frame();
    init_frame(prevFrame);
    //prevFrame.detect_lines();
    //prevFrame.calc_keyPts();
    SHOW(prevFrame.lines().size());
    SHOW(prevFrame.keyPts().size());
    for(int i = id_start+1; i <= id_last; ++i) {
        cout << prevFrame.get_id() << "--" << i << endl;
        NewFrame cur{i};
        cur.read_frame();
        imwrite(dst_dir+"rgb_"+to_string(cur.get_id())+".jpg", cur.rgb());


        printf("%d\n", i);
        Tracker tk(prevFrame.keyPts(), prevFrame.edge(),
                cur.edge());
        if(! tk.run()){
            if(init_frame(cur)){
                swap(prevFrame, cur);
            }
            continue;
        }
        cout << "track ok" << endl;
        cv::Mat imgTrack = tk.draw();
        imwrite(dst_dir+"track_"+to_string(i-1) + "--" + to_string(i) + ".jpg", imgTrack);
        //cv::imshow("track result", imgTrack);
        //waitKey(0);

        vector<pair<double, double>> theta_rgs;
        if(!predict_lines(prevFrame.line_endPt_id_map(), tk, theta_rgs)){
            if(init_frame(cur)){
                swap(prevFrame, cur);
            }
            continue;
        }
        cout << "predict ok" << endl;
        for(pair<double, double> & rg: theta_rgs) {
            cout << rg.first << ", " << rg.second << "\n";
        }




        if(!cur.detect_lines(theta_rgs)) {
            if(init_frame(cur)){
                swap(prevFrame, cur);
            }
            continue;
        }
        cout << "detecting lines ok" << endl;
        log_line_img(cur);
        cout << "lines:\n";
        for(const Vec2f & l : cur.lines()) {
            cout << l[0] << ", " << l[1] << '\n';
        }
        //cout << "count of lines=" << cur.lines().size() << endl;
        SHOW(cur.lines().size());

        if(!cur.calc_keyPts() || int(cur.keyPts().size()) < init_keyPt_thres) {
            if(init_frame(cur)){
                swap(prevFrame, cur);
            }
            continue;
        }
        cout << "calc keyPts ok" << endl;
        SHOW(cur.keyPts().size());
        log_keyPt_img(cur);


        shared_ptr<NewMatch> pMch = match_pts(tk, prevFrame, cur);
        if(nullptr == pMch) {
            cout << "fail match\n";
            if(init_frame(cur)){
                swap(prevFrame, cur);
            }
            continue;
        }
        cout << "success match\n";

        cv::Mat img_mch = pMch->draw();
        cv::imwrite(dst_dir+"match_"+to_string(i-1) + "--" + to_string(i) + ".jpg", img_mch);
        swap(prevFrame, cur);
    }

    set_cout_default();
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
