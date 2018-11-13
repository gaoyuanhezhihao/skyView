#ifndef VO_FRAME_H
#define VO_FRAME_H

#include "core.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
class KeyPoints{
    private:
        vector<Point2f> _pts;
        vector<int> _lmk_ids;
    public:
        KeyPoints(vector<Point2f> && pts):_pts(std::move(pts)), _lmk_ids(pts.size(), -1) {}
};

class KeyLines{
    private:
        vector<vector<int>> _hl_pt_map;
        vector<vector<int>> _vl_pt_map;
};

class Frame_Base{
    public:
        virtual void read_frame()=0;
        virtual void set_line(shared_ptr<)
}

class VO_Frame{
    public:
        virtual read_frame();
};


#endif //VO_FRAME_H
