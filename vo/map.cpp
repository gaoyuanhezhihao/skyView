#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

cv::Rect2f get_min_surround_rect(const vector<Point2f> & pts) {
    CV_Assert(pts.size() == 4);
    float x_min = pts[0].x;
    float x_max = pts[0].x;
    float y_min = pts[0].y;
    float y_max = pts[0].y;

    for(int i = 1; i < 4; ++i) {
        x_min = min(x_min, pts[i].x);
        x_max = max(x_max, pts[i].x);
        y_min = min(y_min, pts[i].y);
        y_max = max(y_max, pts[i].y);
    }
    return cv::Rect(x_min, y_min, x_max-x_min, y_max-y_min);
}

cv::Rect2f get_min_total_rect(const Rect2f & rect1, const Rect2f & rect2) {
    float x_min = min(rect1.tl().x, rect2.tl().x);
    float y_min= min(rect1.tl().y, rect2.tl().y);

    float x_max = max(rect1.br().x, rect2.br().x);
    float y_max = max(rect1.br().y, rect2.br().y);
    return Rect2f(x_min, y_min, x_max-x_min, y_max-y_min);
}



void merge_image(const Mat & im1, const Mat & im2, const Mat & R, const Mat & t) {
    const int w2 = im2.cols;
    const int h2 = im2.rows;

    vector<Point2f> new_endpts;
    new_endpts.push_back(trsf_pt_2D(R, t, {0, 0}));
    new_endpts.push_back(trsf_pt_2D(R, t, {0, h2}));
    new_endpts.push_back(trsf_pt_2D(R, t, {w2, 0}));
    new_endpts.push_back(trsf_pt_2D(R, t, {w2, h2}));

    Rect2f rect2 = get_min_surround_rect(new_endpts);
    Rect2f rect1(0, 0, im1.cols, im1.rows);

    Rect2f rect_c = get_min_total_rect(rect1, rect2);

    cv::Mat dst(rect_c.height, rect_c.width, im1.type());
    cv::Mat t_c1(2, 1, CV_64F);
    t_c1.at<double>(0, 0) = rect_c.tl().x;
    t_c1.at<double>(1, 0) = rect_c.tl().y;
    //back_warp(dst, im1, t_c1);
}
