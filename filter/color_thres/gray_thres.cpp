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
    const int gray_low = get_param("gray_low");
    const int gray_high = get_param("gray_high");
    cout << dst_dir << endl;
    ImgLogger thres_log(dst_dir, "thres");

    cv::Mat edge;
    for(int i =id_start; i < id_last; ++i) {
        cout << i << "\n";
        cv::Mat src = imread(src_dir + to_string(i) + ".jpg");
        cv::Mat src_gray;
        cvtColor( src, src_gray, CV_BGR2GRAY );
        edge = cv::Mat::zeros(src_gray.rows, src_gray.cols, src_gray.type());
        for(int r = 0; r < src_gray.rows; ++r) {
            for(int c = 0; c < src_gray.cols; ++c) {
                int v = src_gray.at<uchar>(r, c);
                //++cnt[v];
                if(gray_low < v && v < gray_high) {
                    edge.at<uchar>(r, c) = 255;
                }
            }
        }
        imshow("src", src);
        imshow("edge", edge);
        waitKey(10);
        thres_log.save(edge, i);
    }
    return 0;
}

