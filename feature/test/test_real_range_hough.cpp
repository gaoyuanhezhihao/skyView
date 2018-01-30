#include <iostream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "Config.hpp"
#include "line.hpp"

using namespace std;
using namespace cv;

#define SHOW(a) cout << #a << "="<< a << '\n'

int main(int argc, const char ** argv) {
    string config_string{"theta_resolution=1.0\nrho_resolution=0.5\n"};
    configs.init_by_string(config_string);

    cv::Mat rgb = cv::imread("./data/floor_c920_640x480/test_samples/rgb_69.jpg");
    cv::Mat gray, edge;
    vector<Vec2f> lines_expect;
    cvtColor(rgb, gray, CV_BGR2GRAY);
    blur(gray, gray, Size(5,5) );
    Canny(gray, edge, 50, 100, 5);
    cv::Mat edge_clone = edge.clone();
    imwrite("/tmp/edge.jpg", edge);


    {
        HoughLines(edge, lines_expect, 0.5, CV_PI/180, 200, 0, 0 );
        cv::Mat img_line_std = rgb.clone();
        draw_lines(img_line_std, lines_expect, GREEN);
        imwrite("/tmp/std_lines.jpg", img_line_std);
    }


    {
        const double theta_width = 5.0*CV_PI/180;
        vector<pair<double, double>> theta_rgs;
        for(Vec2f & l : lines_expect) {
            theta_rgs.emplace_back(l[1]-theta_width, l[1]+theta_width);
        }

        vector<Vec2f> lines;
        range_hough(edge_clone, theta_rgs, 200, lines);
        cv::Mat img_line_rg = rgb.clone();
        draw_lines(img_line_rg, lines, GREEN);
        imwrite("/tmp/lines_rg.jpg", img_line_rg);
    }
}
