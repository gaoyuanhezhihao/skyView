#include <memory>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include "base.hpp"
#include "Config.hpp"

using namespace std;
using namespace cv;
namespace FS=boost::filesystem;
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


std::streambuf *coutbuf = nullptr;
std::shared_ptr<std::ofstream> p_out = nullptr;
void redirect_cout() {
    const string dst_dir = configs["result_dir"];
    //std::ofstream out(dst_dir+"cout.txt");
    p_out = std::make_shared<std::ofstream>(dst_dir+"cout.txt");
    coutbuf= std::cout.rdbuf(); //save old buf
    std::cout.rdbuf(p_out->rdbuf()); 
}
void set_cout_default() {
    std::cout.rdbuf(coutbuf);
}


ImgLogger::ImgLogger(const string dst_dir, const std::string name){
    FS::path dst_path(dst_dir);
    FS::path dst_name(name);
    _dir = dst_dir/dst_name;
    FS::remove_all(_dir);
    FS::create_directories(_dir);
}

void ImgLogger::save(const cv::Mat & img, const int id) const {
    save(img, to_string(id));
}
void ImgLogger::save(const cv::Mat & img, const string name) const {
    boost::filesystem::path fname(name+".jpg");
    FS::path p = _dir / fname;
    //imwrite(img, p.string());
    imwrite(p.string(), img);
}
