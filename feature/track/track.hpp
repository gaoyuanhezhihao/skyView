#ifndef TRACK_H
#define TRACK_H

#include <memory>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "base.hpp"
#include "core.hpp"

class Tracker{
    private:
        const std::vector<cv::Point2f> & _old_pts;
        const cv::Mat & _old_gray;
        const cv::Mat & _new_gray;
        std::vector<cv::Point2f> _tracked_pts;
        std::vector<uchar> _states;
    public:
        Tracker(const std::vector<cv::Point2f> & old_pts, const cv::Mat & old_gray, const cv::Mat & new_gray): _old_pts(old_pts), _old_gray(old_gray), _new_gray(new_gray){

            CV_Assert(!_old_gray.empty());
            CV_Assert(!_new_gray.empty());
            CV_Assert(!_old_pts.empty());
        }

        const std::vector<cv::Point2f> & get_tracked_pts()const{
            return _tracked_pts;
        }
        bool check_status(const int i) const{
            return _states[i];
        }

        const std::vector<cv::Point2f> ref_tracked_pts() const {
            return _tracked_pts;
        }

        const std::vector<uchar> ref_status()const {
            return _states;
        }

        bool run();
        cv::Mat draw(); 
};

std::shared_ptr<NewMatch> match_pts(const Tracker & tracker, NewFrame & prevF, NewFrame & curF);
#endif //TRACK_H
