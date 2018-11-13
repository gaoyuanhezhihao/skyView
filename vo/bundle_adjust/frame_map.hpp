#ifndef FRAME_MAP_H
#define FRAME_MAP_H

#include <vector>
#include <memory>
#include "core.hpp"
using namespace std;

shared_ptr<Frame_Interface> get_frame(const int i = 0);
shared_ptr<Frame_Interface> get_first_prev_frame(Frame_Interface & cur);
void add_frame_map(shared_ptr<Frame_Interface> pf);
#endif //FRAME_MAP_H
