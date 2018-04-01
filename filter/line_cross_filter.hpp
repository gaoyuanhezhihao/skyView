#ifndef LINE_CROSS_FILTER_H
#define LINE_CROSS_FILTER_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
using namespace std;
using namespace cv;
void filter_by_line_cross(Size img_sz, vector<Vec2f> & h_lines, vector<Vec2f> & v_lines);
//double angle_of_vecs(const Point & vec1, const Point & vec2);
double angle_of_vecs(const Point2f & vec1, const Point2f & vec2);
double perpendicular_ratio(const Vec2f & l1, const Vec2f & l2);
Point2f vec_of_line(const Vec2f & l);
#endif //LINE_CROSS_FILTER_H
