#ifndef TRACK_H
#define TRACK_H

#include <memory>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class Tracker{
    private:
        std::vector<cv::Point2f> & _old_pts;
        cv::Mat & _old_gray;
        cv::Mat & _new_gray;
        std::vector<cv::Point2f> _tracked_pts;
        std::vector<uchar> _states;
    public:
        Tracker(std::vector<cv::Point2f> & old_pts, cv::Mat & old_gray, cv::Mat & new_gray): _old_pts(old_pts), _old_gray(old_gray), _new_gray(new_gray), _tracked_pts(old_pts.size()), _states(old_pts.size()){

            CV_Assert(!old_gray.empty());
            CV_Assert(!new_gray.empty());
        }

        const std::vector<cv::Point2f> & get_tracked_pts(){
            return _tracked_pts;
        }
        bool check_status(const int i) {
            return _states[i];
        }

        bool run();
};

#endif //TRACK_H
