#ifndef FRAME_POSE_H
#define FRAME_POSE_H
#include <iostream>
#include <memory>
#include "core.hpp"
#include "motion.hpp"

class Frame_Pose_Interface {
    public:
        //virtual bool init_by_odom(std::shared_ptr<Frame_Pose_Interface>  prev_pose, const Motion_Interface & mot) = 0;
        virtual bool report(ostream & out) = 0;
        //virtual void init(double x = 0, double y = 0, double theta = 0);
        virtual double dx() = 0;
        virtual double dy() = 0;
        virtual double theta() = 0;
        virtual bool is_valid() = 0;
};

class SimpleFramePose:public Frame_Pose_Interface{
    public:
        virtual bool report(ostream & out) override;
        virtual double dx() override;
        virtual double dy() override;
        virtual double theta() override;
        virtual bool is_valid() override;
        //virtual void init(double x = 0, double y = 0, double theta = 0) override;
        SimpleFramePose(double x = 0, double y = 0, double theta = 0);
        //virtual bool init_by_odom(std::shared_ptr<Frame_Pose_Interface>  prev_pose, const Motion_Interface & mot) override;
        SimpleFramePose(std::shared_ptr<Frame_Pose_Interface>  prev_pose, const Motion_Interface & mot);
        virtual ~SimpleFramePose();
    private:
        bool _valid = false;
        double _t[3];/* x, y, theta*/
};

#endif //FRAME_POSE_H
