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

TEST_F(LineTest, theta_from_endPoint) {
    cv::Point2f p1(0, 1);
    cv::Point2f p2(1, 0);
    double theta = theta_from_endPoint(p1, p2);
    double right_ans = 45 * CV_PI/180;
    EXPECT_LE(abs(theta - right_ans), 10e-3);
}
