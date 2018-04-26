#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "config/Config.hpp"
#include "core.hpp"

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
void edge_filter(const Mat & gray, Mat & edge, const int low, const int high) {
    //cout << type2str(gray.type()) << endl;
    //cout << type2str(edge.type()) << endl;
    CV_Assert(!gray.empty());
    CV_Assert(!edge.empty());

    for(int r = 0; r < edge.rows; ++r) {
        for(int c = 0; c < edge.cols; ++c) {
            int g = gray.at<uchar>(r, c);
            uchar & o = edge.at<uchar>(r, c);
            if(g < low  || high < g) {
                o = 0;
            }
            //if(o > 0 && (g < low || high < g)) {
                    //o = 0;
            //}
        }
    }
}
void edge_black_filter(const Mat & bgr, Mat & edge) {
    static double thres = get_param("black_filter_thres");
    //unsigned char* pixels = (unsigned char*)(bgr.data);
    //for (int i = 0; i < bgr.rows; i++)
    //{
        //for (int j = 0; j < bgr.cols; j++)
        //{
            //double B = pixels[bgr.step * j + i] ;
            //double G = pixels[bgr.step * j + i + 1];
            //double R = pixels[bgr.step * j + i + 2];
        //}
    //}
    for(int r = 0; r < bgr.rows; ++r) {
        for(int c = 0; c < bgr.cols; ++c) {
            double B = bgr.at<Vec3b>(r, c)[0];
            double G = bgr.at<Vec3b>(r, c)[1];
            double R = bgr.at<Vec3b>(r, c)[2];
            // calculate the distance between R, B and G
            double distance = sqrt((R - G) * (R - G) + (R - B) * (R - B) + (G - B) * (G - B));

            // black_ratio = (1 - distance / 411)^4
            double black_ratio = 1 - (distance / 411);
            if(black_ratio < 0){
                black_ratio = 0;
            }
            black_ratio = black_ratio * black_ratio;
            black_ratio = black_ratio * black_ratio;
            if(black_ratio < thres) {
                edge.at<uchar>(r,c) = 0;

            }
            //int gray = 0.3 * R + 0.59 * G + 0.11 * B;
            //int p = (255 - gray) * black_ratio;
            //if(p < thres) {
                //edge.at<uchar>(r,c) = 0;
            //}
        }
    }
}

void edge_neib_filter(const Mat & gray, Mat & edge, const int low, const int high) {
    CV_Assert(!gray.empty());
    CV_Assert(!edge.empty());

    static const vector<Point> neib{{-1, 0}, {1, 0}, {0, 1}, {0,-1}, {0, 0}};
    for(int r = 0; r < edge.rows; ++r) {
        for(int c = 0; c < edge.cols; ++c) {
            uchar & o = edge.at<uchar>(r, c);
            if(o == 0) {
                continue;
            }
            bool found = false;
            for(const Point nb: neib) {
                int nr = r + nb.y;
                int nc = c + nb.x;
                if(0<= nr && nr < gray.rows && 
                        0 <= nc && nc < gray.cols) {
                    uchar g = gray.at<uchar>(nr, nc);
                    if(low < g && g < high) {
                        found = true;
                        break;
                    }
                }
            }
            if(!found) {
                o = 0;
            }
        }
    }
}
void extract_black(Mat* src, Mat* dst, double th_g){
    // loop for each pixel
    // skip some pixels to calculate fast
    int skip_step_ = 1;
    for(int y = 0; y < src->rows; y=y+skip_step_){
        for(int x = 0; x < src->cols; x=x+skip_step_){
            double r = src->data[ y * src->step + x * src->elemSize()];
            double g = src->data[ y * src->step + x * src->elemSize() + 1];
            double b = src->data[ y * src->step + x * src->elemSize() + 2];

            // calculate the distance between R, B and G
            double distance = sqrt((r - g) * (r - g) + (r - b) * (r - b) + (g - b) * (g - b));

            // black_ratio = (1 - distance / 411)^4
            double black_ratio = 1 - (distance / 411);
            if(black_ratio < 0){
                black_ratio = 0;
            }
            black_ratio = black_ratio * black_ratio;
            black_ratio = black_ratio * black_ratio;

            // grayscale 
            int gray = 0.3 * r + 0.59 * g + 0.11 * b;
            int p = (255 - gray) * black_ratio;

            // binarize
            if(p > th_g){
                dst->data[ y / skip_step_ * dst->step + x / skip_step_ * dst->elemSize()] = 255;
            }
        }
    }
    return;
}
void find_black_line(Mat & bgr) {
    Mat black_image = Mat::zeros(Size(bgr.cols , bgr.rows), CV_8U);
    extract_black(&bgr, &black_image, 120);
    Mat filteredx = black_image.clone();
    Sobel(black_image, filteredx, CV_8U, 1, 0);
    imshow("black", black_image);
    waitKey(10);
}

void SimpleFrame::filter_edge(){
    static const int gray_low = get_param("gray_thres_low");
    static const int gray_high = get_param("gray_thres_high");
    //edge_filter(_gray, _edge, gray_low, gray_high);
    find_black_line(_rgb);
    //edge_neib_filter(_gray, _edge, gray_low, gray_high);
    //edge_black_filter(_rgb, _edge);
}
