#ifndef CORE_H
#define CORE_H

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Config.hpp"


class NewFrame{
    private:
        int _id;
        cv::Mat _rgb;
        cv::Mat _gray;
        cv::Mat _edge;
        std::vector<cv::Point2f> _KeyPts;
        //std::shared_ptr<std::vector<cv::Point2f>> _pKeyPts;
        std::vector<cv::Vec2f> _lines;
        //std::shared_ptr<std::vector<cv::Vec2f>> _pLines;
        //std::shared_ptr<std::vector<std::vector<int>>> _pLine_endPt_id_map;
        std::vector<std::vector<int>> _Line_endPt_id_map;
        cv::Mat _R_global;
        cv::Mat _t_global;
    public:
        NewFrame(int id):_id(id), _R_global(cv::Mat::eye(2, 2, CV_64F)), _t_global(cv::Mat::zeros(2, 1, CV_64F)) {}

        //void set_lines(std::shared_ptr<std::vector<cv::Vec2f>> pl) {
            //assert(nullptr == _pLines);
            //_pLines = pl;
        //}

        bool calc_keyPts(); 
        bool detect_lines(const vector<pair<double, double>> & theta_rgs);
        bool detect_lines();

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
        int get_id() {
            return _id;
        }

        void read_frame();

};

class NewMatch{
    private:
        const NewFrame * pf1;
        const NewFrame * pf2;
        std::vector<std::pair<int, int>> ids;
        cv::Mat R;
        cv::Mat t;
    public:
        NewMatch(const NewFrame* pframe1, const NewFrame * pframe2):pf1(pframe1), pf2(pframe2) {

        }

        void add(int id1, int id2) {
            ids.emplace_back(id1, id2);
        }
        cv::Mat draw() const; 
};

cv::Scalar rand_color(); 

cv::Mat draw_frame(const NewFrame & f);
#endif //CORE_H