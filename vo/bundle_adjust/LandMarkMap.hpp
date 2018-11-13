#ifndef LANDMARKMAP_H
#define LANDMARKMAP_H


#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include "core.hpp"

using namespace std;
using namespace cv;



Point2f get_land_mark(const int id);
void connect_land_mark(int frame_id, const int im_pt_idx, const int lmk_pt_idx);
int add_landmark(Point2f pt);
#endif //LANDMARKMAP_H
