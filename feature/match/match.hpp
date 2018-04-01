#ifndef MATCH_H
#define MATCH_H

#include <iostream> 
#include "core.hpp"
#include "track.hpp"

class Matcher_Interface{
    public:
        virtual ~Matcher_Interface(){};
        virtual bool match()=0;
        //virtual bool solve_cam_motion() = 0;
        virtual const vector<pair<int, int>> & ids()const=0;
        virtual const Frame_Interface * pf1()const =0;
        virtual const Frame_Interface * pf2()const =0;
        virtual void log_img() const=0;
        //virtual void report(std::ostream & out)const=0;
};

class SimpleMatcher:public Matcher_Interface{
    private:
        Frame_Interface * _pf1;
        Frame_Interface * _pf2;
        const Tracker & _trk;
        vector<pair<int, int>> _mch_ids;
    public:
        SimpleMatcher(Frame_Interface * pf1, Frame_Interface * pf2, const Tracker & tracker):_pf1(pf1), _pf2(pf2), _trk(tracker){}
        virtual bool match()override;
        //virtual bool solve_cam_motion()override;
        virtual const vector<pair<int, int>> & ids() const override{return _mch_ids;}
        virtual const Frame_Interface * pf1() const override{return _pf1;};
        virtual const Frame_Interface * pf2() const override{return _pf2;};
        virtual void log_img() const override;
};


#endif //MATCH_H
