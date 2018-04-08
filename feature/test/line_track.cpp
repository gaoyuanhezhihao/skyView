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

using namespace cv;
using namespace std;

template <const char * subfix>
void log_line_img(const NewFrame & f) {
    static const string dst_dir = configs["result_dir"];
    static ImgLogger im_log(dst_dir, string("line_")+subfix);
    cv::Mat line_img = f.rgb().clone();
    draw_lines(line_img, f.lines(), GREEN);
    im_log.save(line_img, f.get_id());
    //imwrite(dst_dir+"line_"+to_string(f.get_id())+".jpg", line_img);
}

constexpr const char before[] = "before";
constexpr const char after[] = "after";

void log_keyPt_img(const NewFrame & f) {
    static const string dst_dir = configs["result_dir"];
    static ImgLogger im_log(dst_dir, "keyPt");

    cv::Mat keyPt_img = f.rgb().clone();
    draw_points(keyPt_img, f.keyPts());
    im_log.save(keyPt_img, f.get_id());
    //imwrite(dst_dir+"keyPt_"+to_string(f.get_id()) + ".jpg", keyPt_img);
}
void test() {
    static const int init_keyPt_thres = configs["init_keyPt_thres"];
    static const string samples_dir = configs["samples"];
    static const string dst_dir = configs["result_dir"];
    static const ImgLogger track_im_log(dst_dir, "track");
    static const ImgLogger match_im_log(dst_dir, "match");

    int id_start = configs["start_id"];
    const int id_last = configs["last_id"];

    //redirect_cout();
    
    NewFrame prevFrame{id_start};
    prevFrame.read_frame();
    while(!init_frame(prevFrame)){
        cerr << "fail to init_frame " << id_start << "\n";
        imshow("edge", prevFrame.edge());
        imshow("rgb", prevFrame.rgb());
        waitKey(0);
        prevFrame = NewFrame(++id_start);
        prevFrame.read_frame();
    }
    //prevFrame.detect_lines();
    //prevFrame.calc_keyPts();
    SHOW(prevFrame.lines().size());
    SHOW(prevFrame.keyPts().size());
    for(int i = id_start+1; i <= id_last; ++i) {
        cout << prevFrame.get_id() << "--" << i << endl;
        NewFrame cur{i};
        cur.read_frame();
        //imwrite(dst_dir+"rgb_"+to_string(cur.get_id())+".jpg", cur.rgb());


        printf("%d--%d\n", prevFrame.get_id(), i);
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
        
        boost::format fmter{"%1%--%2%"};
        string id_name = str(fmter%prevFrame.get_id()%cur.get_id());
        track_im_log.save(imgTrack, id_name);
        /* predict lines */
        vector<pair<double, double>> theta_rgs;
        vector<Vec2f> tracked_lines;
        if(!predict_lines(prevFrame.line_endPt_id_map(), tk, theta_rgs, tracked_lines)){
            if(init_frame(cur)){
                swap(prevFrame, cur);
            }
            continue;
        }
        cout << "predict ok" << endl;
        //for(pair<double, double> & rg: theta_rgs) {
            //cout << rg.first << ", " << rg.second << "\n";
        //}




        
        if(!cur.detect_lines(theta_rgs)) {
            cur.detect_lines();
            //if(init_frame(cur)){
                //swap(prevFrame, cur);
            //}else {
                //continue;
            //}
        }
        log_line_img<before>(cur);
        cur.merge_tracked_lines(tracked_lines);
        log_line_img<after>(cur);
        cout << "detecting lines ok" << endl;
        //cout << "lines:\n";
        //for(const Vec2f & l : cur.lines()) {
            //cout << l[0] << ", " << l[1] << '\n';
        //}
        //cout << "count of lines=" << cur.lines().size() << endl;
        SHOW(cur.lines().size());

        if(!cur.calc_keyPts() || int(cur.keyPts().size()) < init_keyPt_thres) {
            if(init_frame(cur)) {
                swap(prevFrame, cur);
            }
            continue;
            //cout << "WARN: calc_keyPts Fail. try common hough\n";
            //if(!init_frame(cur)){
                //continue;
                ////swap(prevFrame, cur);
            //}
        }
        cout << "calc keyPts ok" << endl;
        SHOW(cur.keyPts().size());
        log_keyPt_img(cur);


        static const int match_num_thres = configs["match_num_thres"];
        shared_ptr<NewMatch> pMch = match_pts(tk, prevFrame, cur);
        if(nullptr == pMch || pMch->match_num() < match_num_thres) {
            cout << "fail match\n";
            if(nullptr != pMch) {
                cv::Mat img_mch = pMch->draw();
                {
                boost::format fmter{"%1%--%2%"};
                string id_name = str(fmter%prevFrame.get_id()%cur.get_id());
                match_im_log.save(img_mch, id_name);
                }
            }
            if(init_frame(cur)){
                swap(prevFrame, cur);
            }
            continue;
        }
        cout << "success match\n";

        cv::Mat img_mch = pMch->draw();


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


