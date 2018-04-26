#ifndef EDGE_FILTER_H
#define EDGE_FILTER_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace cv;
void edge_filter(const Mat & gray, Mat & edge, const int low, const int high);
#endif //EDGE_FILTER_H
