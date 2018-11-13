#include <vector>
#include <memory>
#include "core.hpp"
#include "frame_map.hpp"

vector<shared_ptr<Frame_Interface>> _frame_map;

void add_frame_map(shared_ptr<Frame_Interface> pf) {
    const int gap = pf->get_id() + 1 - _frame_map.size();
    for(int i = 0; i <= gap; ++i) {
        _frame_map.push_back(nullptr);
    }

    _frame_map[pf->get_id()] = pf;
}

shared_ptr<Frame_Interface> get_frame(const int i) {
    const int len = _frame_map.size();
    //assert(i < len);
    if(i<len) {
        return _frame_map[i];
    }else {
        return nullptr;
    }
}

shared_ptr<Frame_Interface> get_first_prev_frame(Frame_Interface & cur) {
    int i = min(cur.get_id(), int(_frame_map.size()-1));
    for(; i>= 0; --i) {
        if(nullptr != _frame_map[i]) {
            return _frame_map[i];
        }
    }
    return nullptr;
}
