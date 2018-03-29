#include "gtest/gtest.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <string>
#include <algorithm>
#include "line_cross_filter.hpp"
//#include "Config.hpp"
using namespace std;
class LineFilterUnitTest: public ::testing::Test{
    protected:
        virtual void SetUp(){

        }

        virtual void TearDown(){

        }
};

int main(int argc, char **argv) {
      ::testing::InitGoogleTest(&argc, argv);
    //string config_string{"theta_resolution=1.0\nrho_resolution=0.5\n"};
    //configs.init_by_string(config_string);
        return RUN_ALL_TESTS();


}

TEST_F(LineFilterUnitTest, angle_of_vecs) {
    cv::Point v1(0, 1);
    cv::Point v2(1, 0);
    EXPECT_LE(abs(CV_PI/2 - angle_of_vecs(v1, v2)), 10-5);
}

