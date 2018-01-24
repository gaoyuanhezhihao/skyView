#include "gtest/gtest.h"
#include "line.hpp"
#include "Config.hpp"
#include <string>
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


TEST_F(RangeHoughTest, one_line) {
    cv::Mat edge_im = Mat::zeros(480, 640, CV_8UC1);
    cv::line(edge_im, {100, 0}, {100, 470}, Scalar(255));
    vector<pair<double, double>> theta_ranges{{0.0, 180.0}};
    vector<Vec2f> lines;
    EXPECT_EQ(true, range_hough(edge_im, theta_ranges, 100, lines));
    vector<Vec2f> expect{100.0, 0.0};
    EXPECT_EQ(1, lines.size());
    if(lines.size() == 1) {
        EXPECT_LE(abs(lines[0][0]-expect[0][0]), 10-5);
        EXPECT_LE(abs(lines[0][1]-expect[0][1]), 10-5);
    }
}
