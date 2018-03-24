#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <array>
#include <numeric>
//#include "vanish.hpp"
#include "Config.hpp"
#include "base.hpp"
using namespace std;
using namespace cv;

const int CLICK_NUM = 1;
array<cv::Point, CLICK_NUM> click_pts;
int click_cnt = 0;
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN ) {
        if(click_cnt < CLICK_NUM) {
            click_pts[click_cnt++] = Point(x, y);
        }else {
            cout << "WARNING no more clicks!" << endl;
        }
    }
}

int hough_thres = 150;
enum SelectResult{
    Retry,
    Success,
    Over
};

struct Sample{
    Mat rgb;
    vector<Vec2f> lines;
    vector<Point> pts;
    vector<int> pt_ids;
    Mat draw() {
        assert(pts.size() == pt_ids.size());
        Mat clone = rgb.clone();
        draw_lines(clone, lines, GREEN);
        
        for(size_t i = 0; i < pts.size(); ++i) {
            circle(clone, pts[i], 5, Scalar(0, 100, 100));
            cv::putText(clone, to_string(pt_ids[i]), pts[i],\
                    FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
        }
        return clone;
    }
    void dump_lines(ofstream & out) const {
        for(const Vec2f & l:lines) {
            out << l[0] << " " << l[1] << '\n';
        }
    }
    void dump_pts(ofstream & out) const {
        for(size_t i = 0; i < pts.size(); ++i) {
            out << pts[i].x << " " << pts[i].y  << " " << pt_ids[i] << "\n";
        }
    }
};

Point show_waitClick(Mat rgb) {
    click_cnt = 0;
    while(click_cnt < 1) {
        imshow("window", rgb);
        waitKey(10);
    }
    return click_pts[0];
}

char show_waitKey(Mat rgb, const vector<Vec2f> & lines, const int select=-1)  {
    draw_lines(rgb, lines, Scalar(0, 100, 100));
    if(-1 != select) {
        draw_lines(rgb, vector<Vec2f> {lines[select]}, GREEN);
    }
    imshow("window", rgb);
    return waitKey(0);
}
vector<Vec2f> adjust_hough(Mat rgb, Mat edge) {
    vector<Vec2f> lines;
    do{
        lines.clear();
        HoughLines(edge, lines, 1, CV_PI / 180, hough_thres, 0, 0);
    }while('y' != show_waitKey(rgb.clone(), lines));
    return lines;
}

struct PtDistCmpor{
    Point pt;
    PtDistCmpor(const Point pt):pt(pt) {}
    bool operator()(const Vec2f & l, const Vec2f & r) const {
        return dist_pt2line(l, pt) < dist_pt2line(r, pt);
    }
};
char select_one_line(Mat rgb, vector<Vec2f> lines, vector<Vec2f> & selected) {
    if(0 == lines.size()) {
        return 'o';
    }
    Point pt = show_waitClick(rgb);
    char order = ' ';
    std::sort(lines.begin(), lines.end(), PtDistCmpor(pt));
    while('o' != order) {
        for(size_t i = 0; i < lines.size(); ++i) {
            order = show_waitKey(rgb.clone(), lines, i);
            if('y' == order) {
                selected.push_back(lines[i]);
                break;
            }
            if('o' == order) { /* over */
                break;
            }
        }
    }

    return order;
}

void select_lines(Sample & rst) {
    cv::Mat edge;
    Canny(rst.rgb, edge, 30, 70, 3);
    vector<Vec2f> lines = adjust_hough(rst.rgb, edge);

    Mat img_lines = rst.rgb.clone();
    draw_lines(img_lines, lines, Scalar(0, 100, 100));
    do{
        draw_lines(img_lines, rst.lines, GREEN);
        imshow("selected lines", img_lines);
        waitKey(10);
    }while('o' != select_one_line(img_lines.clone(), lines, rst.lines));
}

vector<Point> intersect(vector<Vec2f> & l_lines, vector<Vec2f> & r_lines) {
    vector<Point> Intersection;
    int x =0;
    int y = 0;
    for (size_t i = 0; i < l_lines.size(); ++i) {
        for (size_t j = 0; j < r_lines.size(); ++j) {
            float rho_l = l_lines[i][0], theta_l = l_lines[i][1];
            float rho_r = r_lines[j][0], theta_r = r_lines[j][1];
            double denom = (sin(theta_l)*cos(theta_r) - cos(theta_l)*sin(theta_r));
            x = (rho_r*sin(theta_l) - rho_l*sin(theta_r)) / denom;
            y = (rho_l*cos(theta_r) - rho_r*cos(theta_l)) / denom;
            Point pt(x, y);
            Intersection.push_back(pt);
        }
    }
    return Intersection;
}

Point intersect(Vec2f l1, Vec2f l2) {
    float rho_l = l1[0], theta_l = l1[1];
    float rho_r = l2[0], theta_r = l2[1];
    double denom = (sin(theta_l)*cos(theta_r) - cos(theta_l)*sin(theta_r));
    int x = (rho_r*sin(theta_l) - rho_l*sin(theta_r)) / denom;
    int y = (rho_l*cos(theta_r) - rho_r*cos(theta_l)) / denom;
    Point pt(x, y);
    return pt;
}


struct Pt2PtCmpor{
    Point ref;
    Pt2PtCmpor(Point ref):ref(ref){}
    bool operator()(const Point & lp, const Point & rp) const {
        return cv::norm(lp-ref) < cv::norm(rp-ref);
    }
};
char select_one_pt(vector<Point> pts, Mat img, Sample & rst) {
    Point click = show_waitClick(img);
    sort(pts.begin(), pts.end(), Pt2PtCmpor(click));
    char order = ' ';
    while('n' == order) {
        for(size_t i = 0; i < pts.size(); ++i) {
            Mat clone = img.clone();
            circle(clone, pts[i], 5, Scalar(0, 100, 100));
            imshow("select point", clone);
            order = waitKey(0);
            if('n' == order) {
                continue;
            }
            if('o' == order) {
                break;
            }
            assert('0' <= order && order <= '9');
            int id = order-'0';
            rst.pts.push_back(pts[i]);
            rst.pt_ids.push_back(id);
            cout << "insert " << id << "\n";
        }
    }
    assert(rst.pts.size() == rst.pt_ids.size());
    return order;
}

void draw_pts(Mat & img, const vector<Point>& pts, cv::Scalar color=Scalar(0, 100, 100)) {
    for(const Point & pt: pts) {
        circle(img, pt, 5, color);
    }
}

void select_point(Sample & rst) {
    const int sz = rst.lines.size();
    if(0 == sz) {
        cout << "empty lines!!!\n";
    }

    vector<Point> pts;
    Mat img_total_pts = rst.rgb.clone();
    for(int i = 0; i < sz; ++i) {
        for(int j = i+1; j < sz; ++j) {
            pts.emplace_back(intersect(rst.lines[i], rst.lines[j]));
        }
    }
    draw_pts(img_total_pts, pts);

    do{
        draw_pts(img_total_pts, rst.pts, GREEN);
        imshow("selected pts", img_total_pts);
        waitKey(10);
    }while('o' != select_one_pt(pts, img_total_pts.clone(), rst));
}

Mat draw_match(const Sample & prev, const Sample & cur) {
    Mat prev_img = prev.draw();
    Mat cur_img = cur.draw();
    cv::Mat match_img;
    cv::hconcat(prev_img, cur_img, match_img);
    const int prev_cols = prev_img.cols;
    for(size_t i = 0; i < prev.pts.size(); ++i) {
        for(size_t j = 0; j < cur.pts.size(); ++j) {
            if(prev.pt_ids[i] == cur.pt_ids[j]) {
                cv::line(match_img,prev.pts[i], cur.pts[j] + Point(prev_cols),\
                        rand_color(), 1, CV_AA);
            }
        }
    }
    return match_img;
}
Sample process(Mat rgb) {
    static Sample prev_smpl;
    if(!prev_smpl.rgb.empty()) {
        imshow("prev", prev_smpl.draw());
        waitKey(1);
    }
    Sample rst;
    rst.rgb = rgb;
    select_lines(rst);
    select_point(rst);

    imshow("match", draw_match(prev_smpl, rst));
    waitKey(1);
    prev_smpl = rst;
    return rst;
}

int main(int argc, const char ** argv) {
    if(argc != 2) {
        cout << "Error! \nusage example ./bin/vanish_pt ../param/configs\n";
    }
    configs.init(argv[1]);
    const string dst_dir = configs["result_dir"];
    const string src_dir = configs["source_dir"];
    namedWindow("window", 1);
    setMouseCallback("window", CallBackFunc, NULL);
    const int start_id = configs["start_id"];
    const int last_id = configs["last_id"];
    for(int i= start_id; i <= last_id; ++i) {
        Mat rgb = imread(src_dir+to_string(i)+".jpg");
        Sample rst = process(rgb);
        
    }
}
