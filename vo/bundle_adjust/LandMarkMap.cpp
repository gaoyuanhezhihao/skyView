#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "LandMarkMap.hpp"
#include "core.hpp"

vector<Point2f> _land_marks; 

struct FrameRelation{
    int frame_id;
    int pt_id;
    FrameRelation(int frame_id, int idx): frame_id(frame_id), pt_id(idx){}
};

vector<vector<FrameRelation>> _connects;


Point2f get_land_mark(const int id) {
    assert(id < _land_marks.size());
    return _land_marks[id];
}


void connect_land_mark(int frame_id, const int im_pt_idx, const int lmk_pt_idx){
    assert(lmk_pt_idx< _land_marks.size());
    assert(_land_marks.size() == _connects.size());
    _connects[lmk_pt_idx].emplace_back(frame_id, im_pt_idx);
}

int add_landmark(Point2f pt) {
    _land_marks.push_back(pt);
    _connects.push_back(vector<FrameRelation>());
    assert(_land_marks.size() == _connects.size());
    return _land_marks.size()-1;
}
