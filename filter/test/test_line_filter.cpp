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
#include "line.hpp"
#include "stereo.hpp"
#include "track.hpp"
#include "core.hpp"
#include "range_hough.hpp"
#include "match.hpp"
#include "motion.hpp"
#include "predictor.hpp"
#include "frame_pose.hpp"


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

void log_rgb(const Frame_Interface & f) {
    static const string dst_dir = configs["result_dir"];
    static ImgLogger im_log(dst_dir, "rgb");
    im_log.save(f.rgb(), f.get_id());
}
void test() {
    static const string samples_dir = configs["samples"];
    static const string dst_dir = configs["result_dir"];
    static const ImgLogger track_im_log(dst_dir, "track");
    static const ImgLogger match_im_log(dst_dir, "match");
    ofstream fail_records(dst_dir + "fails.txt", std::ios::out);
    ofstream vo_records(dst_dir + "vo.txt", std::ios::out);

    std::ofstream vo_log(dst_dir+"vo_log.txt");
    int id_start = configs["start_id"];
    const int id_last = configs["last_id"];

    //redirect_cout();
    SimpleFrame prevFrame{id_start};
    prevFrame.read_frame();
    shared_ptr<Frame_Pose_Interface> pos = std::make_shared<SimpleFramePose>(0, 0, 0);

    prevFrame.set_pose(pos);
    while(!prevFrame.init()){
        cerr << "fail to init_frame " << id_start << "\n";
        prevFrame = SimpleFrame(++id_start);
        prevFrame.read_frame();
    }

    log_line_img<before>(prevFrame);
    log_keyPt_img(prevFrame);
    log_rgb(prevFrame);
    for(int i = id_start+1; i <= id_last; ++i) {
        cout << prevFrame.get_id() << "--" << i << endl;
        string cur_pair = to_string(prevFrame.get_id()) + "--" + to_string(i);
        SimpleFrame cur{i};
        cur.read_frame();

        log_rgb(cur);
        printf("%d--%d\n", prevFrame.get_id(), i);
        /* track */
        /* TODO try to use gray image */
        Tracker tk(prevFrame.pts(), prevFrame.edge(),
                cur.edge());
        if(! tk.run()){
            fail_records << cur_pair << ":" << "track fail\n";
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
        /* detect lines */
        bool hstat = h_predictor.run();
        bool vstat = v_predictor.run();
        if(!hstat && !vstat) {
            cout << "FAIL predict, try init()\n";
            fail_records << cur_pair << ":" << "FAIL predict, try init()\n";
            cur.init();
        }else {
            if(h_predictor.is_failed()) {
                h_predictor.predict_from_vertical(v_predictor);
            }

            if(v_predictor.is_failed()) {
                v_predictor.predict_from_vertical(h_predictor);
            }
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
                fail_records << cur_pair << ":" << "FAIL range hough, try init()\n";
                cur.init();
            }
        }
        /* key Points */
        if(!cur.calc_keyPts()) {
            cout << "FAIL calc_keyPts" << endl;
            fail_records << cur_pair << ":FAIL calc_keyPts\n";
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
            fail_records << cur_pair << "fail match\n";
            cout << "fail match\n";
        }
        /* motion of car*/
        if(match_succeed) {
            /*calc new*/
            Ceres_2frame_motion motion(&matcher);
            motion.calc_cam_motion();
            //assert(nullptr != prevFrame.get_pose());
            //prevFrame.get_pose()->report(cout);
            motion.report(cout);
            shared_ptr<Frame_Pose_Interface> cur_pose = make_shared<SimpleFramePose> (prevFrame.get_pose(), motion);
            cur.set_pose(cur_pose);
            //Motion_Interface & ref_m = motion;
            //cur_pose->init_by_odom(prevFrame.get_pose(), ref_m);
            cur_pose->report(cout);
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
