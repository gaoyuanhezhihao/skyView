#ifndef LINE_CROSS_FILTER_H
#define LINE_CROSS_FILTER_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
using namespace std;
using namespace cv;
void filter_by_line_cross(Size img_sz, vector<Vec2f> & h_lines, vector<Vec2f> & v_lines);
double angle_of_vecs(const Point & vec1, const Point & vec2);
#endif //LINE_CROSS_FILTER_H
