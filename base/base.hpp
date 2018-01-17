#ifndef BASE_H
#define BASE_H

#include <iostream>
#include <string>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

const cv::Scalar GREEN(0, 255, 0);
const cv::Scalar RED(0, 0, 255);

struct Frame{
    int id;
    cv::Mat rgb;
    cv::Mat edge;
    std::vector<cv::Point2f> keyPts;
    std::vector<cv::Vec2f> lines;
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
#endif //BASE_H
