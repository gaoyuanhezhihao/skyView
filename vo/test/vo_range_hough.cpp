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

using namespace cv;
using namespace std;


void log_line_img(const NewFrame & f) {
    static const string dst_dir = configs["result_dir"];
    static ImgLogger im_log(dst_dir, "line");
    cv::Mat line_img = f.rgb().clone();
    draw_lines(line_img, f.lines(), GREEN);
    im_log.save(line_img, f.get_id());
    //imwrite(dst_dir+"line_"+to_string(f.get_id())+".jpg", line_img);
}

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

    const int id_start = configs["start_id"];
    const int id_last = configs["last_id"];

    //redirect_cout();
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
        //imwrite(dst_dir+"track_"+to_string(i-1) + "--" + to_string(i) + ".jpg", imgTrack);
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
        cout << "detecting lines ok" << endl;
        log_line_img(cur);
        //cout << "lines:\n";
        //for(const Vec2f & l : cur.lines()) {
            //cout << l[0] << ", " << l[1] << '\n';
        //}
        //cout << "count of lines=" << cur.lines().size() << endl;
        SHOW(cur.lines().size());

        if(!cur.calc_keyPts() || int(cur.keyPts().size()) < init_keyPt_thres) {
            cout << "WARN: calc_keyPts Fail. try common hough\n";
            if(!init_frame(cur)){
                continue;
                //swap(prevFrame, cur);
            }
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

        /* motion of car*/

        {
        boost::format fmter{"%1%--%2%"};
        string id_name = str(fmter%prevFrame.get_id()%cur.get_id());
        match_im_log.save(img_mch, id_name);
        }
        //cv::imwrite(dst_dir+"match_"+to_string(i-1) + "--" + to_string(i) + ".jpg", img_mch);
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
//}
