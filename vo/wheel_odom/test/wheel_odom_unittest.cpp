#include "gtest/gtest.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <string>
#include <algorithm>
#include <fstream>
#include "wheel_odom.hpp"
#include "config/Config.hpp"
using namespace std;
class WheelOdomUnitTest: public ::testing::Test{
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

TEST_F(WheelOdomUnitTest, read_wheel_odom) {
    configs.init_by_string("src_dir=./\n");
    ofstream of("./0.txt");
    of << 1.1 << "\n" << 2.2 << "\n" << 3.1 <<endl;
    of.close();
    Odom_Pack odm = read_wheel_odom(0);
    EXPECT_EQ(1.1, odm.dx);
    EXPECT_EQ(2.2, odm.dy);
    EXPECT_EQ(3.1, odm.dth);
}
