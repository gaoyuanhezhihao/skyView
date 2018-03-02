#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "stereo.hpp"

using namespace cv;
using namespace std;

int main(int argc, const char ** argv) {
    if( argc != 2) {
        cout <<" Error! not enough param \n Usage: runner config.txt" << endl;
        return -1;
    }
    configs.init(argv[1]);
    cv::Mat sky_view_img = calibrate();
    imwrite("calibrate_sky_view_image.jpg", sky_view_img);
    return 0;
}
