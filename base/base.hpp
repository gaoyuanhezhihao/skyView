#ifndef BASE_H
#define BASE_H

#include <iostream>
#include <string>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <boost/filesystem.hpp>
#include <limits>

#define DOUBLE_MAX std::numeric_limits<double>::max()

const cv::Scalar GREEN(0, 255, 0);
const cv::Scalar RED(0, 0, 255);
const cv::Scalar BLUE(255, 0, 0);

class ImgLogger{
    public:
        ImgLogger(const std::string dst_dir, const std::string name); 
        void save(const cv::Mat & img, const int id) const;
        void save(const cv::Mat & img, const std::string name) const;
        void save(const cv::Mat & img, std::initializer_list<int> v)const;
    private:
        boost::filesystem::path _dir;
};

struct Frame{
    int id;
    cv::Mat rgb;
    cv::Mat gray;
    cv::Mat edge;
    std::vector<cv::Point2f> keyPts;
    std::vector<cv::Vec2f> lines;
    std::vector<std::vector<int>> line_endPt_id_map;
    cv::Mat R_global;
    cv::Mat t_global;

    Frame(int id):id(id), R_global(cv::Mat::eye(2, 2, CV_64F)), t_global(cv::Mat::zeros(2, 1, CV_64F)){}
};

struct Match{
    Frame * pf1;
    Frame * pf2;
    std::vector<std::pair<int, int>> ids;
    Match(Frame* pframe1, Frame * pframe2):pf1(pframe1), pf2(pframe2) {

    }
    cv::Mat R;
    cv::Mat t;
};

cv::Scalar rand_color(); 

//void show_img(cv::Mat & img, std::string & title);
cv::Mat draw_frame(const Frame & f);
void draw_lines(cv::Mat & img, const std::vector<cv::Vec2f> & lines, cv::Scalar color=cv::Scalar(0, 0, 255));
void draw_points(cv::Mat & img, const std::vector<cv::Point2f> & pts);
void draw_points(cv::Mat & img, const std::vector<cv::Point2f> & pts, cv::Scalar color);
float street_dist(const cv::Point2f & p1, const cv::Point2f & p2);
void set_cout_default();
void redirect_cout();
int dist_pt2line(const cv::Vec2f & line, const cv::Point & pt);
void print_pts(const std::vector<cv::Point2f> & pts);
#endif //BASE_H
