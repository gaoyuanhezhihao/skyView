#ifndef MOTION_H
#define MOTION_H

#include <iostream>
#include <array>
#include <exception>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "match.hpp"

using namespace std;

class Motion_Interface {
    public:
        virtual bool calc_cam_motion() = 0;
        virtual ~Motion_Interface(){}
        virtual void report(ostream & out)const=0;
        virtual std::string format()const{
            throw std::logic_error("Not Implemented !! Motion_Interface::format()");
        }
        virtual double get_dx()const = 0;
        virtual double get_dy()const = 0;
        virtual double get_theta()const = 0;
};

class Ceres_2frame_motion: public Motion_Interface{
    private:
        Matcher_Interface * _pMch;
        cv::Mat R;
        cv::Mat t;
        double _dx=.0;
        double _dy=.0;
        double _theta=.0;
    public:
        Ceres_2frame_motion(Matcher_Interface * pMatcher):_pMch(pMatcher) {}
        bool calc_cam_motion()override; 
        void report(ostream & out) const override;
        virtual double get_dx()const override {return _dx;}
        virtual double get_dy()const override {return _dy;}
        virtual double get_theta()const override {return _theta;}
};

class Ceres_global_motion: public Motion_Interface{
    private:
        Matcher_Interface * _pMch;
        double _dx;
        double _dy=.0;
        double _theta=.0;
    public:
        Ceres_global_motion(Matcher_Interface * pMatcher): _pMch(pMatcher) {}
        bool calc_cam_motion()override; 
        void report(ostream & out) const override;
        virtual double get_dx()const override {return _dx;}
        virtual double get_dy()const override {return _dy;}
        virtual double get_theta()const override {return _theta;}
        virtual string format()const override;
};

template<int rows, int cols>
using Matrix=std::array<std::array<double, cols>, rows>;

class TransformMat{
    public:
        TransformMat(double dx, double dy, double dth);
        //cv::Point transform(cv::Point pt);
        cv::Point operator*(const cv::Point & pt);
        cv::Point2f operator*(const cv::Point2f & pt);
    private:
        Matrix<2,2> _R;
        std::array<double, 2> _t;
};

class MatG2IM{
    public:
        MatG2IM(double x, double y, double th);
        cv::Point2f operator*(const cv::Point2f & pt);
    private:
        Matrix<2, 2> _R;
        std::array<double, 2> _t;
};

shared_ptr<Frame_Pose_Interface> Ceres_solve_im2g(Frame_Pose_Interface & prev_pose, vector<pair<Point2f, Point2f>> & mch_pts);
#endif //MOTION_H
