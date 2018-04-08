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
string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

void sobel(cv::Mat & src, cv::Mat & edge, cv::Mat & grad) {
    static const int thres_value  = get_param("threshold");
    static const string dst_dir = get_param("dst_dir");
    Mat src_gray;
    //Mat grad;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;


    /// Load an image
    if( !src.data ) {
        throw std::out_of_range("imread fail");
    }

    GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );

    /// Convert it to gray
    cvtColor( src, src_gray, CV_BGR2GRAY );

    /// Generate grad_x and grad_y
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

    /// Gradient X
    //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
    Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );

    /// Gradient Y
    //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
    Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_y, abs_grad_y );

    /// Total Gradient (approximate)
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

    //cout << "grad.depth() = " << grad.depth() << \
        //" , grad.channels()=" << grad.channels() << endl;
    //cout << "type=" << type2str(grad.type()) << endl;
    ////std::set<uchar> img_values;
    //vector<int> cnt(256, 0);
    edge = cv::Mat::zeros(grad.rows, grad.cols, grad.type());
    for(int r = 0; r < grad.rows; ++r) {
        for(int c = 0; c < grad.cols; ++c) {
            int v = grad.at<uchar>(r, c);
            //++cnt[v];
            if(v > thres_value) {
                edge.at<uchar>(r, c) = 255;
            }
        }
    }
    //for(int i = 0; i <256; ++i) {
        //if(cnt[i] != 0) {
            //cout << i << ":" << cnt[i] << "\t";
        //}
    //}



    //imwrite(dst_dir + "grad.jpg", grad);
    //imwrite("edge.jpg", edge);
    //imshow("threholded", edge);
    //waitKey(0);
    //imshow( window_name, grad );

    //waitKey(0);
}

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
    ImgLogger edge_log(dst_dir, "edge");
    ImgLogger grad_log(dst_dir, "grad");

    Mat edge, grad;
    for(int i =id_start; i < id_last; ++i) {
        cout << i << "\n";
        Mat src = imread(src_dir + to_string(i) + ".jpg");
        sobel(src,edge, grad);
        edge_log.save(edge, i);
        grad_log.save(grad, i);

        //imwrite(dst_dir + "grad/" + to_string(i) + ".jpg", grad);
        //imwrite(dst_dir + "edge/" + to_string(i) + ".jpg", edge);
    }
    return 0;
}
