#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include "base.hpp"

using namespace cv;
using namespace std;

cv::Mat debug_img;

void debug_show_img(cv::Mat img, const vector<Vec2f> & lines, string title) {
    draw_lines(img, lines);
    imshow(title, img);
    waitKey(0);
}

template <class T>
void debug_show(const string fname, const int line_num, const string vname, const T & v) {
    cout << fname << ":" << line_num <<" " << vname << "=" << v << endl;
}

template <class T>
void debug_show(const string fname, const int line_num, const string vname, const vector<T> & vec) {
    cout << fname << ":" << line_num <<" " << vname << "=" << endl;
    for(const auto & v: vec) {
        cout << '\t' << v << ",";
    }
}
