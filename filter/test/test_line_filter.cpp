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
#include "line.hpp"
#include "stereo.hpp"
#include "track.hpp"
#include "core.hpp"
#include "range_hough.hpp"
#include "match.hpp"
#include "motion.hpp"
#include "predictor.hpp"


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


void log_keyPt_img(const Frame_Interface& f) {
    static const string dst_dir = configs["result_dir"];
    static ImgLogger im_log(dst_dir, "keyPt");

    cv::Mat keyPt_img = f.rgb().clone();
    draw_points(keyPt_img, f.pts());
    im_log.save(keyPt_img, f.get_id());
}
void test() {
    static const int init_keyPt_thres = configs["init_keyPt_thres"];
    static const string samples_dir = configs["samples"];
    static const string dst_dir = configs["result_dir"];
    static const ImgLogger track_im_log(dst_dir, "track");
    static const ImgLogger match_im_log(dst_dir, "match");

    std::ofstream vo_log(dst_dir+"vo_log.txt");
    int id_start = configs["start_id"];
    const int id_last = configs["last_id"];

    //redirect_cout();
    SimpleFrame prevFrame{id_start};
    prevFrame.read_frame();
    while(!prevFrame.init()){
        cerr << "fail to init_frame " << id_start << "\n";
        prevFrame = SimpleFrame(++id_start);
        prevFrame.read_frame();
    }

    log_line_img<before>(prevFrame);
    log_keyPt_img(prevFrame);
    for(int i = id_start+1; i <= id_last; ++i) {
        cout << prevFrame.get_id() << "--" << i << endl;
        SimpleFrame cur{i};
        cur.read_frame();

        printf("%d--%d\n", prevFrame.get_id(), i);
        /* track */
        /* TODO try to use gray image */
        Tracker tk(prevFrame.pts(), prevFrame.edge(),
                cur.edge());
        if(! tk.run()){
            if(cur.init()){
                swap(prevFrame, cur);
            }
            continue;
        }
        cout << "track ok" << endl;
        cv::Mat imgTrack = tk.draw();
        
        boost::format fmter{"%1%--%2%"};
        string id_name = str(fmter%prevFrame.get_id()%cur.get_id());
        track_im_log.save(imgTrack, id_name);

        /* predict lines */
        OpticalLinePredictor h_predictor(prevFrame.get_hl_pt_map(), tk);
        OpticalLinePredictor v_predictor(prevFrame.get_vl_pt_map(), tk);
        //vector<pair<double, double>> h_theta_rgs;
        //vector<pair<double, double>> v_theta_rgs;
        //vector<Vec2f> h_tracked_lines;
        //vector<Vec2f> v_tracked_lines;

        //predict_lines(prevFrame.get_hl_pt_map(), tk, h_theta_rgs, h_tracked_lines);
        //predict_lines(prevFrame.get_vl_pt_map(), tk, v_theta_rgs, v_tracked_lines);
        //SHOW(h_tracked_lines.size());
        //SHOW(v_tracked_lines.size());
        /* detect lines */
        if(h_predictor.run() && v_predictor.run()) {
            cout << "predict ok" << endl;
            if(cur.range_hough(h_predictor.theta_rgs(), v_predictor.theta_rgs())) {
                cout << "detecting lines ok" << endl;
                log_line_img<before>(cur);
                cur.merge_old_hl(h_predictor.tracked_lines());
                cur.merge_old_vl(v_predictor.tracked_lines());
                log_line_img<after>(cur);
                cur.filter_line();
                log_line_img<filtered>(cur);
            }else {
                cout << "FAIL range hough, try init()\n";
                cur.init();
            }
        }else {
            cout << "FAIL predict, try init()\n";
            cur.init();
        }
        /* key Points */
        if(!cur.calc_keyPts()) {
            cout << "FAIL calc_keyPts" << endl;
            if(cur.init()) {
                swap(prevFrame, cur);
            }
            continue;
        }
        cout << "calc keyPts ok" << endl;
        log_keyPt_img(cur);


        /* match */
        SimpleMatcher matcher(&prevFrame, &cur, tk);
        bool match_succeed = matcher.match();
        if(match_succeed) {
            cout << "success match\n";
            matcher.log_img();
        }else {
            cout << "fail match\n";
        }
        /* motion of car*/
        if(match_succeed) {
            /*calc new*/
            Ceres_2frame_motion motion(&matcher);
            motion.calc_cam_motion();
        }else {
            /* predict from history */
        }
        swap(prevFrame, cur);
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
