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

    cv::Mat edge_im = Mat::zeros(480, 640, CV_8UC1);
    cv::line(edge_im, {100, 0}, {100, 470}, Scalar(255));
    cv::imshow("edge_im", edge_im);
    waitKey(0);
    vector<pair<double, double>> theta_ranges{{0.0, CV_PI}};
    vector<Vec2f> lines;
    range_hough(edge_im, theta_ranges, 100, lines);
    SHOW(lines.size());
    cout << "rho = " << lines[0][0] << '\n';
    cout << "theta = " << lines[0][1] * 180.0/CV_PI << "\n";
}
