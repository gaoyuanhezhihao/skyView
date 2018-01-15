#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <vector>
#include <cmath>
#include "line.hpp"
#include "debug.hpp"

using namespace cv;
using namespace std;
cv::Scalar rand_color() {
    static cv::RNG rng(12345);
    return cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
}
bool intersect(const Vec2f & l1, const Vec2f & l2, Point2f & pt) {
    const double r1 = l1[0];
    const double theta1 = l1[1];
    const double c1 = cos(theta1);
    const double s1 = sin(theta1);

    const double r2 = l2[0];
    const double theta2 = l2[1];
    const double c2 = cos(theta2);
    const double s2 = sin(theta2);

    if(abs(theta1-theta2) < 0.01) {
        /* they are parallel */
        return false;
    }

    /* x */
    double denom = c1*s2-s1*c2;
    double nume = r1 * s2 - r2 * s1;
    pt.x = nume/denom;
    /* y */
    denom = (s1*c2-s2*c1);
    nume = (r1*c2-r2*c1);
    pt.y = nume/denom;
    //cout << "pt=" << pt << endl;
    return true;
}

vector<Point2f> get_inlier_intersects(const vector<Vec2f> & lines, const cv::Size & img_size)  {
    const int sz = lines.size();
    cv::Point2f pt;
    vector<Point2f> inliers;
    for(int i = 0; i < sz; ++i) {
        for(int j = i+1; j < sz; ++ j){
            cv::Mat tmp_img = debug_img.clone();
            //cout << "=====" << lines[i] << "---" << lines[j] << endl;
            //vector<Vec2f> tmp_lines{lines[i], lines[j]};
            //draw_lines(tmp_img, tmp_lines);
            if(intersect(lines[i], lines[j], pt) &&
                    0.0f <= pt.x && pt.x < img_size.width &&
                    0.0f <= pt.y && pt.y < img_size.height) {
                inliers.push_back(pt);
                //vector<Point2f> tmp_pts{pt};
                //draw_points(tmp_img, tmp_pts);
            }
            //cv::imshow("debug img", tmp_img);
            //cv::waitKey(0);
        }
    }
    return inliers;
}


vector<Vec2f> detect_lines(cv::Mat & src) {
    Mat dst, cdst;
    cvtColor(src, dst, CV_BGR2GRAY);
    blur(dst, dst, Size(5,5) );
    Canny(dst, dst, 50, 100, 5);
    cvtColor(dst, cdst, CV_GRAY2BGR);
    cv::imshow("edge", cdst);
    cv::waitKey(1);
    //vector<Vec4i> linesP; 
    //HoughLinesP(dst, linesP, 1, CV_PI/180, 50, 50, 10 ); 
    vector<Vec2f> lines;
    HoughLines(dst, lines, 0.5, CV_PI/180, 200, 0, 0 );
    return lines;
}

void draw_lines(cv::Mat & img, const vector<Vec2f> & lines) {
    for( size_t i = 0; i < lines.size(); i++ ) {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line(img, pt1, pt2, rand_color(), 3, CV_AA);
    }
}
void draw_linesP(cv::Mat & img, const vector<Vec4i> & lines) {
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line(img, Point(l[0], l[1]), Point(l[2], l[3]), rand_color(), 1, LINE_AA);
    }
}

void draw_points(cv::Mat & img, const vector<Point2f> & pts) {
    for(const Point2f& pt: pts) {
        cv::circle(img, Point(pt.x, pt.y), 5, rand_color(), 2);
    }
}
