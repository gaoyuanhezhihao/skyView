#ifndef MOTION_H
#define MOTION_H

#include <iostream>
#include "match.hpp"

using namespace std;

class Motion_Interface {
    public:
        virtual bool calc_cam_motion() = 0;
        virtual ~Motion_Interface(){}
        virtual void report(ostream & out)const=0;
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
};

#endif //MOTION_H
