#include "gtest/gtest.h"
#include "line.hpp"
#include "Config.hpp"
#include "range_hough.hpp"
#include <string>
#include <algorithm>
using namespace std;
class RangeHoughTest: public ::testing::Test{
    protected:
        virtual void SetUp(){

        }

        virtual void TearDown(){

        }
};
int main(int argc, char **argv) {
      ::testing::InitGoogleTest(&argc, argv);
    string config_string{"theta_resolution=1.0\nrho_resolution=0.5\n"};
    configs.init_by_string(config_string);
        return RUN_ALL_TESTS();


}
Vec2f add_line(cv::Mat & im, Point2f pt1, Point2f pt2) {
    float rho = rho_from_endPoint(pt1, pt2);
    float theta = theta_from_endPoint(pt1, pt2);
    cv::line(im, pt1, pt2, Scalar(255));
    return Vec2f{rho, theta};
}

TEST_F(RangeHoughTest, one_line) {
    cv::Mat edge_im = Mat::zeros(480, 640, CV_8UC1);
    cv::line(edge_im, {100, 0}, {100, 470}, Scalar(255));
    vector<pair<double, double>> theta_ranges{{0.0, CV_PI*179/180}};
    vector<Vec2f> lines;
    EXPECT_EQ(true, range_hough(edge_im, theta_ranges, 100, lines));
    vector<Vec2f> expect{100.0, 0.0};
    EXPECT_EQ(1, lines.size());
    if(lines.size() == 1) {
        EXPECT_LE(abs(lines[0][0]-expect[0][0]), 10-5);
        EXPECT_LE(abs(lines[0][1]-expect[0][1]), 10-5);
    }
}

struct CmpVec2f{
    bool operator()(const Vec2f & l, const Vec2f & r) const {
        if(l[0] == r[0]){
            return l[1] < r[1];
        }else {
            return l[0] < r[0];
        }
    }
};

TEST_F(RangeHoughTest, multi_line) {
    cv::Mat edge_im = Mat::zeros(480, 640, CV_8UC1);
    vector<pair<double, double>> theta_ranges{
        {0.0, CV_PI*2/180}, {CV_PI*89/180, CV_PI*91/180}};

    vector<pair<Point2f, Point2f>> pts{
        {Point2f(100, 0), Point2f(100, 470)},
        {Point2f(0, 100), Point2f(639, 100)}};
    for(pair<Point2f, Point2f> & endp: pts) {
        add_line(edge_im, endp.first, endp.second);
    }

    vector<Vec2f> expect{{100.0, 0.0},
                         {100.0, CV_PI/2}};
    vector<Vec2f> lines;
    range_hough(edge_im, theta_ranges, 100, lines);
    EXPECT_EQ(2, lines.size());
    std::sort(lines.begin(), lines.end(), CmpVec2f());
    if(2 == lines.size()) {
        EXPECT_LE(abs(lines[0][0]-expect[0][0]), 10-5);
        EXPECT_LE(abs(lines[0][1]-expect[0][1]), 10-5);
        EXPECT_LE(abs(lines[1][0]-expect[1][0]), 10-5);
        EXPECT_LE(abs(lines[1][1]-expect[1][1]), 10-5);
    }

}
