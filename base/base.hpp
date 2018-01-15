#ifndef BASE_H
#define BASE_H

#include <iostream>
#include <string>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


struct Frame{
    cv::Mat rgb;
    cv::Mat edge;
    std::vector<cv::Point2f> keyPts;
    std::vector<cv::Vec2f> lines;
};

struct Match{
    Frame * pf1;
    Frame * pf2;
    std::vector<std::pair<int, int>> ids;
    Match(Frame* pframe1, Frame * pframe2):pf1(pframe1), pf2(pframe2) {

    }
};

cv::Scalar rand_color(); 

//void show_img(cv::Mat & img, std::string & title);
cv::Mat draw_frame(const Frame & f);
void draw_lines(cv::Mat & img, const std::vector<cv::Vec2f> & lines, cv::Scalar color=cv::Scalar(0, 0, 255));
void draw_points(cv::Mat & img, const std::vector<cv::Point2f> & pts);
#endif //BASE_H
