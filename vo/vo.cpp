#include <iostream>
#include <algorithm>
#include <memory>
#include <stdexcept>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Eigen/Dense"

#include "base.hpp"
#include "Config.hpp"
#include "stereo.hpp"

using namespace cv;
using namespace std;
using namespace Eigen;


void read_frame(const int i, Frame & f) {
    static const string samples_dir = configs["samples"];
    static const string dst_dir = configs["result_dir"];
    static const int cols_sky_im = configs["cols_sky_im"];
    static const int rows_sky_im = configs["rows_sky_im"];

    f.rgb = cv::imread(samples_dir + to_string(i) + ".jpg");
    f.rgb = get_sky_view(f.rgb, cols_sky_im, rows_sky_im);
}

std::shared_ptr<Match> match_keyPoints(Frame & f1, Frame & f2) {
    static auto cmp_pt = [](const Point2f & p1, const Point2f & p2) {
        if(p1.x == p2.x) {
            return p1.y < p2.y;
        }else {
            return p1.x < p2.x;
        }
    };
    cout << "f1.keyPts.size() = " << f1.keyPts.size() << ", f2.keyPts.size()"  << f2.keyPts.size() << endl;
    if(f1.keyPts.size() == 4 && f2.keyPts.size() == 4) {
        sort(f1.keyPts.begin(), f1.keyPts.end(), cmp_pt);
        sort(f2.keyPts.begin(), f2.keyPts.end(), cmp_pt);
        shared_ptr<Match> pm = std::make_shared<Match>(&f1, &f2);
        pm->ids = vector<pair<int, int>> {{0, 0}, {1, 1}, {2, 2}, {3, 3}};
        return pm;
    }else {
        throw std::logic_error("bug");
        /* TODO predict missed key points */
        
    }
    return nullptr;
}

void draw_matches(Match & m, cv::Mat & imgMatches) {
    //cv::imshow("m.pf1->rgb", m.pf1->rgb);
    //cv::imshow("m.pf2->rgb", m.pf2->rgb);
    //waitKey(0);
    cv::hconcat(m.pf1->rgb, m.pf2->rgb, imgMatches);
    const int w1 = m.pf1->rgb.cols;
    //const int h1 = m.pf1->rgb.rows;
    const vector<cv::Point2f> & kp1 = m.pf1->keyPts;
    const vector<cv::Point2f> & kp2 = m.pf2->keyPts;
    for(pair<int, int> & mch: m.ids) {
        Point pt1(kp1[mch.first].x, kp1[mch.first].y);
        Point pt2(kp2[mch.second].x+w1, kp2[mch.second].y);
        cv::circle(imgMatches, pt1, 5, rand_color(), 2);
        cv::circle(imgMatches, pt2, 5, rand_color(), 2);
        cv::line(imgMatches, pt1, pt2, rand_color(), 1, CV_AA);
    }
}
bool get_motion(Match & m) {
    int cnt = m.ids.size();
    CV_Assert(cnt == 4);
    Matrix<double, Eigen::Dynamic, 6> A = MatrixXd::Zero(2*cnt, 6);
    Matrix<double, Eigen::Dynamic, 1> b = MatrixXd::Zero(2*cnt, 1);

    for(int i = 0; i < cnt; ++i) {
        const pair<int, int> mch_pt_ids = m.ids[i];
        const double x1 = m.pf1->keyPts[mch_pt_ids.first].x;
        const double y1 = m.pf1->keyPts[mch_pt_ids.first].y;

        const double x2 = m.pf2->keyPts[mch_pt_ids.second].x;
        const double y2 = m.pf2->keyPts[mch_pt_ids.second].y;

        A(2*i, 0) = x2;
        A(2*i, 1) = y2;
        A(2*i, 2) = 1.0;
        A(2*i+1, 3) = x2;
        A(2*i+1, 4) = y2;
        A(2*i+1, 5) = 1.0;

        b(2*i, 0) = x1;
        b(2*i+1, 0) = y1;
    }
    cout << "A=" <<  A << "\n";
    cout << "b=" <<  b << "\n";
    Matrix<double, 6, 1> x = A.colPivHouseholderQr().solve(b);
    double relative_error = (A*x - b).norm() / b.norm(); 
    cout << "relative error = "<< relative_error << "\n";
    cout << "x=" << x << "\n";
    m.R = Mat::zeros(2, 2, CV_64F);
    m.t = Mat::zeros(2, 1, CV_64F);
    m.R.at<double> (0, 0) = x(0, 0);
    m.R.at<double> (0, 1) = x(1, 0);
    m.R.at<double> (1, 0) = x(3, 0);
    m.R.at<double> (1, 1) = x(4, 0);

    m.t.at<double> (0, 0) = x(2, 0);
    m.t.at<double> (1, 0) = x(5, 0);
    return true;
}

//bool get_square_motion(Match & m) {
    //CV_Assert(m.ids.size() == 4);
    //double x1_sum = 0.0;
    //double y1_sum = 0.0;
    //double x2_sum = 0.0;
    //double y2_sum = 0.0;
    //for(int i = 0; i < 4; ++i) {
        //const pair<int, int> mch_pt_ids = m.ids[i];
        //const double x1 = m.pf1->keyPts[mch_pt_ids.first].x;
        //const double y1 = m.pf1->keyPts[mch_pt_ids.first].y;
        //x1_sum += x1;
        //y1_sum += y1;

        //const double x2 = m.pf2->keyPts[mch_pt_ids.second].x;
        //const double y2 = m.pf2->keyPts[mch_pt_ids.second].y;
        //x2_sum += x2;
        //y2_sum += y2;
    //}
    //cv::Point2f center1(x1_sum/4, y1_sum/4);
    //cv::Point2f center2(x2_sum/4, y2_sum/4);

    //cv::Point2f deta = center2 - center1;
//}
