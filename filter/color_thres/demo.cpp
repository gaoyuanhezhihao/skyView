#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <set>
#include <exception>
#include "config/Config.hpp"
#include "base.hpp"

using namespace std;
using namespace cv;

int main( int argc, char** argv ) {

    if( argc != 2) {
        cout <<" Error! not enough param \n Usage: runner config.txt" << endl;
        return -1;

    }
    configs.init(argv[1]);
    const int id_start = get_param("start_id");
    const int id_last = get_param("last_id");
    const string src_dir = get_param("src_dir");
    const string dst_dir = get_param("dst_dir");
    const int t1l = get_param("thres1l");
    const int t2l = get_param("thres2l");
    const int t3l= get_param("thres3l");
    const int t1h = get_param("thres1h");
    const int t2h = get_param("thres2h");
    const int t3h= get_param("thres3h");
    cout << dst_dir << endl;
    ImgLogger thres_log(dst_dir, "thres");

    cv::Mat edge, grad;
    for(int i =id_start; i < id_last; ++i) {
        cout << i << "\n";
        cv::Mat src = imread(src_dir + to_string(i) + ".jpg");
        Mat hsv_image;
        cv::cvtColor(src, hsv_image, cv::COLOR_BGR2HSV);
        cv::Mat output;
        cv::inRange(hsv_image, cv::Scalar(t1l, t2l, t3l), cv::Scalar(t1h, t2h, t3h), output);
        thres_log.save(output, i);
        imshow("thres", output);
        waitKey(10);
    }
    return 0;
}
