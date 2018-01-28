#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "base.hpp"
#include "Config.hpp"

using namespace std;
using namespace cv;
cv::Scalar rand_color() {
    static cv::RNG rng(12345);
    return cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
}

cv::Mat draw_frame(const Frame & f) {
    cv::Mat img = f.rgb.clone();
    draw_lines(img, f.lines);
    draw_points(img, f.keyPts);
    return img;
}

void draw_points(cv::Mat & img, const vector<Point2f> & pts) {
    for(const Point2f& pt: pts) {
        cv::circle(img, Point(pt.x, pt.y), 5, rand_color(), 2);
    }
}

void draw_points(cv::Mat & img, const vector<Point2f> & pts, cv::Scalar color) {
    for(const Point2f& pt: pts) {
        cv::circle(img, Point(pt.x, pt.y), 5, color, 2);
    }
}

void draw_lines(cv::Mat & img, const vector<Vec2f> & lines, cv::Scalar color) {
    for( size_t i = 0; i < lines.size(); i++ ) {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line(img, pt1, pt2, color, 1, CV_AA);
    }
}
void draw_linesP(cv::Mat & img, const vector<Vec4i> & lines) {
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line(img, Point(l[0], l[1]), Point(l[2], l[3]), rand_color(), 1, CV_AA);
    }
}

float street_dist(const Point2f & p1, const Point2f & p2) {
    return abs(p1.x - p2.x) + abs(p1.y - p2.y);
}


