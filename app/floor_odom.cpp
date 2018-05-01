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
#include "wheel_odom.hpp"


using namespace std;
using namespace cv;
using POSE_TYPE=GlobalPose;
namespace bt=boost::timer;
//using namespace haoLib;
class LoggerStream{
    public:
        LoggerStream(std::initializer_list<std::ostream*>  handlers):_hdl(handlers) {
            //for(size_t i = 0; i < handlers.size(); ++i) {
                //_hdl.push_back(&handlers[i]);
            //}
        }
        template<typename T> 
            LoggerStream & operator<<(const T& data) {
                for(size_t i = 0; i < _hdl.size(); ++i) {
                    (*_hdl[i]) << data;

                }
                return *this;
                //for(std::ostream & hd: _hdl) {
                    //hd << data;
                //}
            }
    private:
        std::vector<std::ostream *>  _hdl;
};

//template<>
//LoggerStream & LoggerStream::operator <<(const decltype<endl>& e) {
    //for(std::ostream & hd: _hdl) {
        //hd << endl;
    //}
//}

class TimeRecord{
    public:
        TimeRecord(string name):_ms_sum(0.0), _cnt(0), _name(name){}
        void add(bt::nanosecond_type elaps) {
            //cout << _name  << "cost " << elaps/(1000*1000) << " ms\n";
            _ms_sum += double(elaps)/(1000*1000);
            ++_cnt;
        }
        double average_ms(){
            return _ms_sum / _cnt;
        }
        void report(ostream & out) {
            out << _name << " cost " << _ms_sum / _cnt << " ms/frame\n";
        }
    private:
        double _ms_sum;
        int _cnt;
        string _name;
};

TimeRecord trk_tm("track");
TimeRecord line_tm("detect line");
TimeRecord pt_tm("detect keyPoint");
TimeRecord motion_tm("motion");
TimeRecord match_tm("match");


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
constexpr const char keep2[] = "keep2";


void log_keyPt_img(const Frame_Interface& f) {
    static const string dst_dir = configs["result_dir"];
    static ImgLogger im_log(dst_dir, "keyPt");

    cv::Mat keyPt_img = f.rgb().clone();
    draw_points(keyPt_img, f.pts());
    im_log.save(keyPt_img, f.get_id());
}

void print_keypt(const Frame_Interface & f) {
    cout << "key pts ---\n";
    for(const Point2f & pt: f.pts()) {
        cout << pt << endl;
    }
    cout << "key pts ===\n";
}

void log_rgb(const Frame_Interface & f) {
    static const string dst_dir = configs["result_dir"];
    static ImgLogger im_log(dst_dir, "rgb");
    im_log.save(f.rgb(), f.get_id());
}

shared_ptr<Tracker> track(SimpleFrame & prevFrame, SimpleFrame & cur, LoggerStream & log){
    static const string dst_dir = get_param("result_dir");
    static const ImgLogger track_im_log(dst_dir, "track");
    bt::cpu_timer tm;

    //tk = Tracker(prevFrame.pts(), prevFrame.edge(), cur.edge());
    shared_ptr<Tracker> pTrk = make_shared<Tracker>(prevFrame.pts(), prevFrame.edge(), cur.edge());
    bool state = pTrk->run();
    tm.stop();
    trk_tm.add(tm.elapsed().wall);
    if(state) {
        log<< "track ok" << '\n';
        cv::Mat imgTrack = pTrk->draw();
        boost::format fmter{"%1%--%2%"};
        string id_name = str(fmter%prevFrame.get_id()%cur.get_id());
        track_im_log.save(imgTrack, id_name);
        return pTrk;
    }else {
        string cur_pair = to_string(prevFrame.get_id()) + "--" + to_string(cur.get_id());
        log << cur_pair << ":" << "track fail\n";
        return nullptr;
    }
}
bool predict_line(SimpleFrame & prevFrame, shared_ptr<OpticalLinePredictor> & h_predictor,
        shared_ptr<OpticalLinePredictor> & v_predictor, Tracker & tk, LoggerStream & log) {
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
        log<< "predict ok" << '\n';
        return true;
    }else {
        log << prevFrame.get_id() << " failed to track\n";
        return false;
    }
}

bool detect_lines(SimpleFrame & prevFrame, SimpleFrame & cur, Tracker & tk, LoggerStream & log) {
    bt::cpu_timer tm;
    shared_ptr<OpticalLinePredictor> pPrdH = nullptr;
    shared_ptr<OpticalLinePredictor> pPrdV = nullptr;
    if(predict_line(prevFrame, pPrdH, pPrdV, tk, log)) {
        assert(nullptr != pPrdH);
        assert(nullptr != pPrdV);
        if(cur.range_hough(pPrdH->theta_rgs(), pPrdV->theta_rgs())) {
            tm.stop();
            log << "detecting lines ok" << '\n';
            log_line_img<before>(cur);
            tm.resume();
            cur.merge_old_hl(pPrdH->tracked_lines());
            cur.merge_old_vl(pPrdV->tracked_lines());
            tm.stop();
            log_line_img<after>(cur);
            tm.resume();
            cur.filter_line();
            tm.stop();
            log_line_img<filtered>(cur);
            cur.rm_extra_line();
            log_line_img<keep2>(cur);

            line_tm.add(tm.elapsed().wall);
            return true;
        }else {
            log << "FAIL range hough, try init()\n";
        }
    }else {
        log << "FAIL predict, try init()\n";
    }

    bool state = cur.init();
    tm.stop();
    line_tm.add(tm.elapsed().wall);
    return state;
}

bool detect_key_pts(SimpleFrame & prevFrame, SimpleFrame & cur, LoggerStream & log) {
    bt::cpu_timer tm;
    if(!cur.calc_keyPts()) {
        tm.stop();
        pt_tm.add(tm.elapsed().wall);
        log<< "FAIL calc_keyPts" << '\n';
        string cur_pair = to_string(prevFrame.get_id()) + "--" + to_string(cur.get_id());
        log << cur_pair << ":FAIL calc_keyPts\n";
        return false;
    }
    tm.stop();
    pt_tm.add(tm.elapsed().wall);

    log << "calc keyPts ok" << '\n';
    log_keyPt_img(cur);
    //print_keypt(cur);
    return true;
}

shared_ptr<SimpleMatcher> match(SimpleFrame & prevFrame, SimpleFrame & cur, LoggerStream & log, Tracker & tk) {
    //SimpleMatcher matcher(&prevFrame, &cur, tk);

    bt::cpu_timer tm;
    shared_ptr<SimpleMatcher> pm = make_shared<SimpleMatcher> (&prevFrame, & cur, tk);
    bool match_succeed = pm->match();
    tm.stop();
    match_tm.add(tm.elapsed().wall);
    //cout << tm.format() << endl;
    if(match_succeed) {
        log << "success match\n";
        pm->log_img();
        return pm;
    }else {
        string cur_pair = to_string(prevFrame.get_id()) + "--" + to_string(cur.get_id());
        log << cur_pair << "fail match\n";
        log << "fail match\n";
        return nullptr;
    }
}

bool calc_motion(SimpleFrame & prevFrame, SimpleFrame & cur, shared_ptr<SimpleMatcher> pMch, LoggerStream& log) {
    bt::cpu_timer tm;
    /*calc new*/
    //Ceres_2frame_motion motion(pMch.get());
    Ceres_global_motion mot(pMch.get());
    if(!mot.calc_cam_motion()) {
        log << "Warning! calc_cam_motion failed\n";
        return false;
    }
    assert(nullptr != prevFrame.get_pose());
    //prevFrame.get_pose()->report(cout);
    tm.stop();
    //mot.report(cout);
    log << mot.format() << '\n';
    
    tm.resume();
    shared_ptr<Frame_Pose_Interface> cur_pose = make_shared<POSE_TYPE> (mot.get_dx(), mot.get_dy(), mot.get_theta());

    motion_tm.add(tm.elapsed().wall);
    cur.set_pose(cur_pose);
    //Motion_Interface & ref_m = motion;
    //cur_pose->init_by_odom(prevFrame.get_pose(), ref_m);
    //cur_pose->report(cout);
    log << cur_pose->format() << '\n';
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

void init_pose(shared_ptr<SimpleFrame> brk_head, shared_ptr<SimpleFrame> brk_tail) {
    /*TODO use more powerfull method*/
    shared_ptr<Frame_Pose_Interface> pp = WheelOdom::predict_pose(*brk_head, *brk_tail);
    if(nullptr != pp) {
        brk_tail->set_pose(pp);
    }else {
        throw(std::runtime_error("not implemented"));
        //brk_tail->set_pose(brk_head->get_pose());
    }
}

void test() {
    static const string samples_dir = configs["samples"];
    static const string dst_dir = configs["result_dir"];
    static const ImgLogger track_im_log(dst_dir, "track");
    static const ImgLogger match_im_log(dst_dir, "match");
    cout << "dst_dir = " << dst_dir << endl;
    ofstream file_log(dst_dir + "log.txt");
    //file_log << "hello log" << endl;
    //file_log.close();
    //return ;
    //std::initializer_list<ostream&> handlers{file_log, cout};
    //LoggerStream log(handlers);
    LoggerStream log{&file_log, &cout};

    int id = configs["start_id"];
    const int id_last = configs["last_id"];
    shared_ptr<SimpleFrame> pPrev = init_head_frame(id, id_last);
    shared_ptr<Frame_Pose_Interface> pos = std::make_shared<POSE_TYPE>(0, 0, 0);
    //print_keypt(*pPrev);
    pPrev->set_pose(pos);
    for(;id <= id_last; ++id) {
        //SimpleFrame cur{id};
        if(id - pPrev->get_id() > 3) {
            log<< "Warning!! Broken chain " << pPrev->get_id() << "-X->" << id << '\n';
            shared_ptr<SimpleFrame> pNewPrev = init_head_frame(id, id_last);
            init_pose(pPrev, pNewPrev);
            pPrev = pNewPrev;
            //throw logic_error("vo chain break");
            continue;
        }
        shared_ptr<SimpleFrame> pCur = make_shared<SimpleFrame> (id);
        pCur->read_frame();

        log_rgb(*pCur);
        log<< '[' << pPrev->get_id() << "--"  << pCur->get_id() << "]\n";
        shared_ptr<Tracker> pTrk = track(*pPrev, *pCur, log);
        if(nullptr == pTrk) {
            continue;
        }

        if(!detect_lines(*pPrev, *pCur, *pTrk, log)) {
            continue;
        }

        if(!detect_key_pts(*pPrev, *pCur, log)) {
            continue;
        }

        shared_ptr<SimpleMatcher> pMch = match(*pPrev, *pCur, log, *pTrk);
        if(nullptr == pMch) {
            continue;
        }

        if(calc_motion(*pPrev, *pCur, pMch, log)) {
            pPrev = pCur;
        }
    }

    trk_tm.report(cout);
    line_tm.report(cout);
    pt_tm.report(cout);
    motion_tm.report(cout);
    match_tm.report(cout);
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
