#ifndef LANDMARKMAP_H
#define LANDMARKMAP_H


#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include "core.hpp"

using namespace std;
using namespace cv;

struct FrameRelation{
    shared_ptr<Frame_Interface> ptr;
    int pt_id;
    FrameRelation(shared_ptr<Frame_Interface> connect_frame, int idx): ptr(connect_frame), pt_id(idx){}
};


#endif //LANDMARKMAP_H
