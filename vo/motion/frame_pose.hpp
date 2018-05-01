#ifndef FRAME_POSE_H
#define FRAME_POSE_H
#include <iostream>
#include <memory>
#include <exception>
#include "core.hpp"
#include "motion.hpp"

class Motion_Interface;

class Frame_Pose_Interface {
    public:
        //virtual bool init_by_odom(std::shared_ptr<Frame_Pose_Interface>  prev_pose, const Motion_Interface & mot) = 0;
        virtual std::string whoami() {return "Frame_Pose_Interface";}
        virtual bool report(ostream & out) = 0;
        virtual string format(){throw std::logic_error("format not implemented in base");}
        //virtual void init(double x = 0, double y = 0, double theta = 0);
        virtual double dx() = 0;
        virtual double dy() = 0;
        virtual double theta() = 0;
        virtual bool is_valid() = 0;
        virtual cv::Point2f pt_im2g(const cv::Point2f & ) {throw std::logic_error("pt_im2g not available");}
        virtual cv::Point2f pt_g2im(const cv::Point2f & ) {throw std::logic_error("pt_g2im not available");}
};

class SimpleFramePose:public Frame_Pose_Interface{
    public:
        virtual std::string whoami() override{return "SimpleFramePose";}
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

class GlobalPose: public Frame_Pose_Interface{
    public:
        virtual std::string whoami() override{return "Frame_Pose_Interface";}
        GlobalPose(double x, double y, double theta); 
        virtual bool report(ostream & out) override;
        virtual double dx() override{return _x;}
        virtual double dy() override{return _y;};
        virtual double theta() override{return _th;};
        virtual bool is_valid() override{return true;};
        virtual cv::Point2f pt_im2g(const cv::Point2f & )override;
        virtual cv::Point2f pt_g2im(const cv::Point2f & )override;
        virtual string format() override;
    private:
        double _x;
        double _y;
        double _th;
        TransformMat _im2g;//from image coordinates to global
        TransformMat _g2im;//from global to image
        //cv::Mat _R_im2g; [> from image coordinates to global<]
        //cv::Mat _t_im2g; 

        //cv::Mat _R_g2im; [> from global to image <]
        //cv::Mat _t_g2im; 
};

#endif //FRAME_POSE_H
