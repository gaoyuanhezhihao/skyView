#ifndef MATCH_H
#define MATCH_H

#include "core.hpp"
#include "track.hpp"

class Matcher_Interface{
        virtual bool match_pts();
        virtual bool solve_cam_motion() = 0;
};

class SimpleMatcher:public Matcher_Interface{
    private:
        Frame_Interface * _pf1;
        Frame_Interface * _pf2;
        const Tracker & _trk;
    public:
        SimpleMatcher(Frame_Interface * pf1, Frame_Interface * pf2, const Tracker & tracker):_pf1(pf1), _pf2(pf2), _trk(tracker){}
};


#endif //MATCH_H
