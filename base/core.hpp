#ifndef CORE_H
#define CORE_H

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "config/Config.hpp"
//#include "frame_pose.hpp"
//#include "Pose.hpp"
using namespace std;
using namespace cv;

class Frame_Pose_Interface;
class NewMatch;
class NewFrame;
class Frame_Interface{
    public:
        virtual ~Frame_Interface(){}
        virtual const std::vector<cv::Point2f> & pts() const = 0;
        virtual int get_id() const= 0;
        virtual const cv::Mat & rgb()const=0;
        virtual const cv::Mat & edge()const=0;
        virtual const cv::Mat & gray() const = 0;
        //virtual void set_pose(shared_ptr<Frame_Pose_Interface> pos) = 0;
        virtual shared_ptr<Frame_Pose_Interface> get_pose() const = 0;
        virtual const std::vector<cv::Point2f> & global_pts()const = 0;
        virtual void set_lmk_id(int pt_id, int lmk_id){
            throw std::logic_error("set_lmk_ids not implemented");
        }
        virtual const vector<int> & get_lmk_ids()const {
            throw std::logic_error("get_lmk_ids not implemented");
        }
};

class SimpleFrame: public Frame_Interface{
    private:
        int _id;
        cv::Mat _rgb;
        cv::Mat _gray;
        cv::Mat _edge;
        std::vector<cv::Vec2f> _hl;
        std::vector<cv::Vec2f> _vl;
        std::vector<cv::Point2f> _pts;
        std::vector<int> _lmk_ids;
        std::vector<cv::Point2f> _global_pts;
        std::vector<std::vector<int>> _hl_pt_map;
        std::vector<std::vector<int>> _vl_pt_map;
        std::shared_ptr<Frame_Pose_Interface> _pPos=nullptr;
        void set_global_pts();
    public:
        void set_lmk_id(int pt_id, int lmk_id)override{
            CV_Assert(_lmk_ids.size() == _pts.size());
            //CV_Assert(_lmk_ids.empty());
            _lmk_ids[pt_id] = lmk_id;
        }
        const vector<int> & get_lmk_ids()const override{
            return _lmk_ids;
        }
        void init_pose();
        void set_pose(shared_ptr<Frame_Pose_Interface> pos);
        shared_ptr<Frame_Pose_Interface> get_pose()const override{return _pPos;}
        SimpleFrame(const int id):_id(id){}
        bool range_hough(const std::vector<std::pair<double, double>> & h_theta_rgs, const std::vector<std::pair<double, double>> & v_theta_rgs);
        bool init();
        bool calc_keyPts();
        void read_frame();
        cv::Mat draw_lines() const ;
        const std::vector<std::vector<int>> & get_hl_pt_map() {
            return _hl_pt_map;
        }
        const std::vector<std::vector<int>> & get_vl_pt_map() {
            return _vl_pt_map;
        }
        bool range_hough(const std::vector<std::pair<double, double>> & theta_rgs);
        void merge_old_hl(const std::vector<cv::Vec2f> & old_hl);
        void merge_old_vl(const std::vector<cv::Vec2f> & old_vl);
        void filter_line();
        void rm_extra_line();
        void filter_edge();
        virtual const cv::Mat & rgb()const override {return _rgb;}
        virtual const vector<Point2f> & pts() const override{return _pts;}
        virtual const vector<Point2f> & global_pts() const override{return _global_pts;}
        virtual int get_id()const override{return _id;}
        virtual const cv::Mat & edge()const override{return _edge;}
        virtual const cv::Mat & gray() const override{return _gray;}
};

class NewFrame{
    private:
        int _id;
        cv::Mat _rgb;
        cv::Mat _gray;
        cv::Mat _edge;
        std::vector<cv::Point2f> _KeyPts;
        std::vector<cv::Vec2f> _lines;
        std::vector<std::vector<int>> _Line_endPt_id_map;
        cv::Mat _R_global;
        cv::Mat _t_global;
        std::vector<cv::Point2f> _global_pts;
        std::vector<NewFrame*> _pMatchFrames;
        std::vector<int> _match_keyPt_ids;
        double _theta;
        double _t[2];
        void add_restrict(vector<array<double, 2>> & pt_cur_local, const cv::Point2f & pt_local, NewFrame* pF, const int id, vector<array<double,2>> & pts_global);
    public:
        bool init();

        bool neib_BA();
        void add_match_pts(int this_id, int other, NewFrame* pF);

        void merge_tracked_lines(const std::vector<cv::Vec2f> & tracked_lines); 
        NewFrame(int id):_id(id), _R_global(cv::Mat::eye(2, 2, CV_64F)), _t_global(cv::Mat::zeros(2, 1, CV_64F)) {}

        //void set_lines(std::shared_ptr<std::vector<cv::Vec2f>> pl) {
        //assert(nullptr == _pLines);
        //_pLines = pl;
        //}

        bool calc_keyPts(); 
        bool detect_lines(const vector<pair<double, double>> & theta_rgs);
        bool detect_lines();
        bool detect_lines(const int hough_thres);

        //void set_key_pts(std::shared_ptr<std::vector<cv::Point2f>> pKeyPts,
        //std::shared_ptr<std::vector<std::vector<int>>> pLine_endPt_id_map) {
        //assert(nullptr == _pKeyPts);
        //assert(nullptr == _pLine_endPt_id_map);
        //_pKeyPts = pKeyPts;
        //_pLine_endPt_id_map = pLine_endPt_id_map;
        //}
        const std::vector<cv::Point2f> & keyPts()const {
            return _KeyPts;
        }
        //const std::vector<cv::Point2f> & keyPts()const {
        //return *_pKeyPts;
        //}
        const cv::Mat & rgb()const {
            return _rgb;
        }

        const cv::Mat & gray() const {
            return _gray;
        }
        const std::vector<std::vector<int>> line_endPt_id_map() const {
            return _Line_endPt_id_map;
        }
        //const std::vector<std::vector<int>> line_endPt_id_map() const {
        //return *_pLine_endPt_id_map;
        //}
        const cv::Mat & edge()const {
            return _edge;
        }
        const std::vector<cv::Vec2f> lines()const {
            //return *_pLines;
            return _lines;
        }
        int get_id() const{
            return _id;
        }

        void read_frame();
        void filer_edge();

};

class NewMatch{
    private:
        NewFrame * pf1;
        NewFrame * pf2;
        std::vector<std::pair<int, int>> ids;
        cv::Mat R;
        cv::Mat t;
        double _dx=.0;
        double _dy=.0;
        double _theta=.0;
        //Pose2D _cam_mot;
        //Pose2D _car_mot;
    public:
        NewMatch(NewFrame* pframe1, NewFrame * pframe2):pf1(pframe1), pf2(pframe2) {

        }

        void add(int id1, int id2) ;
        cv::Mat draw() const; 
        bool calc_cam_motion();
        bool calc_car_motion();
        bool ceres_solve_cam_motion();
        double get_dx()const{return _dx;}
        double get_dy()const{return _dy;}
        double get_theta()const {return _theta;}
        int match_num()const {return ids.size();}
};

cv::Scalar rand_color(); 

cv::Mat draw_frame(const NewFrame & f);
#endif //CORE_H
