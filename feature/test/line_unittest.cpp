#include "gtest/gtest.h"
#include "line.hpp"
#include <string>
using namespace std;
class LineTest: public ::testing::Test{
    protected:
        virtual void SetUp(){

        }

        virtual void TearDown(){

        }
};
int main(int argc, char **argv) {
      ::testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();

}

TEST_F(LineTest, theta_rho_from_endPoint) {
    cv::Point2f p1(0, 1);
    cv::Point2f p2(1, 0);
    double theta = theta_from_endPoint(p1, p2);
    double rho = rho_from_endPoint(p1, p2);
    double right_theta = 45 * CV_PI/180;
    double right_rho = sin(CV_PI/4);
    EXPECT_LE(abs(theta - right_theta), 10e-3);
    EXPECT_LE(abs(rho - right_rho), 10e-3);
}

TEST_F(LineTest, merge_ranges) {
    vector<pair<double, double>> ranges{{0.1, 1.2}, {1.1, 1.8}};
    merge_ranges(ranges);
    EXPECT_EQ(1, ranges.size());
    EXPECT_EQ(0.1, ranges[0].first);
    EXPECT_EQ(1.8, ranges[0].second);
    //EXPECT_EQ(pair<double, double>(0.1, 1.8), ranges[0]);
}

TEST_F(LineTest, merge_ranes_2) {
    vector<pair<double, double>> ranges{{0.1, 1.2}, {1.1, 1.8}, {1.7, 2.0}, {2.1, 3.1}, {3.0, 4.0}};
    vector<pair<double, double>> expect{{0.1, 2.0}, {2.1, 4.0}};

    merge_ranges(ranges);
    EXPECT_EQ(expect.size(), ranges.size());
    EXPECT_EQ(expect, ranges);
    //EXPECT_EQ(pair<double, double>(0.1, 1.8), ranges[0]);
}
