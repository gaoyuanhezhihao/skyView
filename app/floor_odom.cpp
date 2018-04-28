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
#include <boost/timer/timer.hpp>

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
namespace bt=boost::timer;
//using namespace haoLib;
class TimeRecord{
    public:
        TimeRecord():_ms_sum(0.0), _cnt(0){}
        void add(bt::nanosecond_type elaps) {
            _ms_sum += elaps/1000;
            ++_cnt;
        }
        double average_ms(){
            return _ms_sum / _cnt;
        }
    private:
        double _ms_sum;
        int _cnt;
};

TimeRecord trk_tm;
TimeRecord line_tm;
TimeRecord pt_tm;
TimeRecord motion_tm;
TimeRecord match_tm;


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

shared_ptr<Tracker> track(SimpleFrame & prevFrame, SimpleFrame & cur, ofstream & fail_records){
    static const string dst_dir = get_param("result_dir");
    static const ImgLogger track_im_log(dst_dir, "track");
    bt::cpu_timer tm;

    //tk = Tracker(prevFrame.pts(), prevFrame.edge(), cur.edge());
    shared_ptr<Tracker> pTrk = make_shared<Tracker>(prevFrame.pts(), prevFrame.edge(), cur.edge());
    bool state = pTrk->run();
    tm.stop();
    trk_tm.add(tm.elapsed().wall);
    if(state) {
        cout << "track ok" << endl;
        cv::Mat imgTrack = pTrk->draw();
        boost::format fmter{"%1%--%2%"};
        string id_name = str(fmter%prevFrame.get_id()%cur.get_id());
        track_im_log.save(imgTrack, id_name);
        return pTrk;
    }else {
        string cur_pair = to_string(prevFrame.get_id()) + "--" + to_string(cur.get_id());
        fail_records << cur_pair << ":" << "track fail\n";
        return nullptr;
    }
}
bool predict_line(SimpleFrame & prevFrame, shared_ptr<OpticalLinePredictor> & h_predictor,
        shared_ptr<OpticalLinePredictor> & v_predictor, Tracker & tk, ofstream & fail_records) {
    /* predict lines */
    h_predictor = make_shared<OpticalLinePredictor>(prevFrame.get_hl_pt_map(), tk);
    v_predictor = make_shared<OpticalLinePredictor>(prevFrame.get_vl_pt_map(), tk);

    bool hstat = h_predictor->run();
    bool vstat = v_predictor->run();
    if(hstat || vstat) {
        if(h_predictor->is_failed()) {
            h_predictor->predict_from_vertical(*v_predictor);
        }

        if(v_predictor->is_failed()) {
            v_predictor->predict_from_vertical(*h_predictor);
        }
        cout << "predict ok" << endl;
        return true;
    }else {
        fail_records << prevFrame.get_id() << " failed to track\n";
        return false;
    }
}

bool detect_lines(SimpleFrame & prevFrame, SimpleFrame & cur, Tracker & tk, ofstream & fail_records) {
    bt::cpu_timer tm;
    shared_ptr<OpticalLinePredictor> pPrdH = nullptr;
    shared_ptr<OpticalLinePredictor> pPrdV = nullptr;
    if(predict_line(prevFrame, pPrdH, pPrdV, tk, fail_records)) {
        assert(nullptr != pPrdH);
        assert(nullptr != pPrdV);
        if(cur.range_hough(pPrdH->theta_rgs(), pPrdV->theta_rgs())) {
            tm.stop();
            cout << "detecting lines ok" << endl;
            log_line_img<before>(cur);
            tm.resume();
            cur.merge_old_hl(pPrdH->tracked_lines());
            cur.merge_old_vl(pPrdV->tracked_lines());
            tm.stop();
            log_line_img<after>(cur);
            tm.resume();
            cur.filter_line();
            tm.stop();
            line_tm.add(tm.elapsed().wall);
            log_line_img<filtered>(cur);
            return true;
        }else {
            fail_records << "FAIL range hough, try init()\n";
        }
    }else {
        fail_records << "FAIL predict, try init()\n";
    }

    bool state = cur.init();
    tm.stop();
    line_tm.add(tm.elapsed().wall);
    return state;
}

bool detect_key_pts(SimpleFrame & prevFrame, SimpleFrame & cur, ofstream & fail_records) {
    bt::cpu_timer tm;
    if(!cur.calc_keyPts()) {
        tm.stop();
        pt_tm.add(tm.elapsed().wall);
        cout << "FAIL calc_keyPts" << endl;
        string cur_pair = to_string(prevFrame.get_id()) + "--" + to_string(cur.get_id());
        fail_records << cur_pair << ":FAIL calc_keyPts\n";
        return false;
    }
    tm.stop();
    pt_tm.add(tm.elapsed().wall);

    cout << "calc keyPts ok" << endl;
    log_keyPt_img(cur);
    return true;
}

shared_ptr<SimpleMatcher> match(SimpleFrame & prevFrame, SimpleFrame & cur, ofstream & fail_records, Tracker & tk) {
    //SimpleMatcher matcher(&prevFrame, &cur, tk);
    
    bt::cpu_timer tm;
    shared_ptr<SimpleMatcher> pm = make_shared<SimpleMatcher> (&prevFrame, & cur, tk);
    bool match_succeed = pm->match();
    tm.stop();
    match_tm.add(tm.elapsed().wall);
    if(match_succeed) {
        cout << "success match\n";
        pm->log_img();
        return pm;
    }else {
        string cur_pair = to_string(prevFrame.get_id()) + "--" + to_string(cur.get_id());
        fail_records << cur_pair << "fail match\n";
        cout << "fail match\n";
        return nullptr;
    }
}

bool calc_motion(SimpleFrame & prevFrame, SimpleFrame & cur, shared_ptr<SimpleMatcher> pMch) {
    bt::cpu_timer tm;
    /*calc new*/
    Ceres_2frame_motion motion(pMch.get());
    motion.calc_cam_motion();
    assert(nullptr != prevFrame.get_pose());
    //prevFrame.get_pose()->report(cout);
    tm.stop();
    motion.report(cout);
    tm.resume();
    shared_ptr<Frame_Pose_Interface> cur_pose = make_shared<SimpleFramePose> (prevFrame.get_pose(), motion);
    motion_tm.add(tm.elapsed().wall);
    cur.set_pose(cur_pose);
    //Motion_Interface & ref_m = motion;
    //cur_pose->init_by_odom(prevFrame.get_pose(), ref_m);
    cur_pose->report(cout);
    return true;
}


shared_ptr<SimpleFrame> init_head_frame(int & id, const int last_id) {
    shared_ptr<SimpleFrame> pf = make_shared<SimpleFrame>(id);
    pf->read_frame();
    while(!pf->init() && ++id < last_id){
        cerr << "fail to init_frame " << id<< "\n";
        pf = make_shared<SimpleFrame>(id);
        pf->read_frame();
    }

    return pf;
}

void test() {
    static const string samples_dir = configs["samples"];
    static const string dst_dir = configs["result_dir"];
    static const ImgLogger track_im_log(dst_dir, "track");
    static const ImgLogger match_im_log(dst_dir, "match");
    ofstream fail_records(dst_dir + "fails.txt", std::ios::out);
    ofstream vo_records(dst_dir + "vo.txt", std::ios::out);

    std::ofstream vo_log(dst_dir+"vo_log.txt");
    int id = configs["start_id"];
    const int id_last = configs["last_id"];
    shared_ptr<SimpleFrame> pPrev = init_head_frame(id, id_last);
    shared_ptr<Frame_Pose_Interface> pos = std::make_shared<SimpleFramePose>(0, 0, 0);
    pPrev->set_pose(pos);
    for(;id <= id_last; ++id) {
        //SimpleFrame cur{id};
        if(id - pPrev->get_id() > 3) {
            cout << "Warning!! Broken chain " << pPrev->get_id() << "-X->" << id << endl;
            pPrev = init_head_frame(id, id_last);
            continue;
        }
        shared_ptr<SimpleFrame> pCur = make_shared<SimpleFrame> (id);
        pCur->read_frame();

        log_rgb(*pCur);
        cout << pPrev->get_id() << "--"  << pCur->get_id() << endl;
        shared_ptr<Tracker> pTrk = track(*pPrev, *pCur, fail_records);
        if(nullptr == pTrk) {
            continue;
        }

        if(!detect_lines(*pPrev, *pCur, *pTrk, fail_records)) {
            continue;
        }

        if(!detect_key_pts(*pPrev, *pCur, fail_records)) {
            continue;
        }

        shared_ptr<SimpleMatcher> pMch = match(*pPrev, *pCur, fail_records, *pTrk);
        if(nullptr == pMch) {
            continue;
        }

        if(calc_motion(*pPrev, *pCur, pMch)) {
            pPrev = pCur;
        }
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
