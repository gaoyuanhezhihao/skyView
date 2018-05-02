#ifndef WHEEL_ODOM_H
#define WHEEL_ODOM_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include "motion.hpp"
#include "core.hpp"

struct Odom_Pack{
    double dx;
    double dy;
    double dth;
    Odom_Pack operator-(const Odom_Pack & other) {
        Odom_Pack rst;
        rst.dx = dx - other.dx;
        rst.dy = dy - other.dy;
        rst.dth = dth - other.dth;
        return rst;
    }
};

Odom_Pack read_wheel_odom(int id);

class WheelOdom{
    public:
        static shared_ptr<Frame_Pose_Interface> predict_pose(Frame_Interface & prev, Frame_Interface & cur);
        static shared_ptr<Frame_Pose_Interface> predict_pose_N(Frame_Interface & cur);
        WheelOdom(int fromID, int toID);
        std::vector<cv::Point> transform(const std::vector<cv::Point> & fromPts);
        std::vector<cv::Point2f> transform(const std::vector<cv::Point2f> & fromPts);
    private:
        std::unique_ptr<TransformMat> _T;
};
#endif //WHEEL_ODOM_H
