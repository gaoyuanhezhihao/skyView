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
#include "core.hpp"
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
    Mat draw() const {
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
    void dump_lines(ostream & out) const {
        for(const Vec2f & l:lines) {
            out << l[0] << " " << l[1] << '\n';
        }
    }
    void dump_pts(ostream & out) const {
        for(size_t i = 0; i < pts.size(); ++i) {
            out << pts[i].x << " " << pts[i].y  << " " << pt_ids[i] << "\n";
        }
    }
};

char show_waitClick_Order(Mat rgb, Point & click) {
    cout << "click points/input order...\n";
    click_cnt = 0;
    char order;
    while(click_cnt < 1) {
        imshow("window", rgb);
        order = waitKey(10);
        if('i' == order || 'd' == order || 'o' == order) {
            break;
        }
    }
    click = click_pts[0];
    return order; 
}

Point show_waitClick(Mat rgb) {
    cout << "click points...\n";
    click_cnt = 0;
    while(click_cnt < 1) {
        imshow("window", rgb);
        waitKey(10);
    }
    return click_pts[0];
}

char show_waitKey(Mat rgb, const vector<Vec2f> & lines, const int select=-1)  {
    cout << "order ?:";
    draw_lines(rgb, lines, Scalar(0, 100, 100));
    if(-1 != select) {
        draw_lines(rgb, vector<Vec2f> {lines[select]}, GREEN);
    }
    imshow("window", rgb);
    char order =waitKey(0);
    cout << order <<"\n";
    return order;
}
//vector<Vec2f> adjust_hough(Mat rgb, Mat edge) {
    //vector<Vec2f> lines;
    //char order = '*';
    //do{
        //cout << "adjust hough...\n";
        //lines.clear();
        //HoughLines(edge, lines, 1, CV_PI / 180, hough_thres, 0, 0);
        //order  = show_waitKey(rgb.clone(), lines);
        //cout << "order =" << order << "\n";
        //if('i' == order) {
            //hough_thres += 5;
            //cout << "increased hough thres to:" << hough_thres << '\n';
        //}else if('d' == order) {
            //hough_thres -= 5;
            //cout << "decreased hough thres to:" << hough_thres << '\n';
        //}
    //}while('y' != order);
    //return lines;
//}

struct PtDistCmpor{
    Point pt;
    PtDistCmpor(const Point pt):pt(pt) {}
    bool operator()(const Vec2f & l, const Vec2f & r) const {
        return dist_pt2line(l, pt) < dist_pt2line(r, pt);
    }
};

bool select_one_line(Mat edge, Mat rgb, Vec2f & selected) {
    static double theta_resolution = configs["theta_resolution"];
    static double rho_resolution = configs["rho_resolution"];
    cout << "select one line\n";
    vector<Vec2f> lines;
    Point click;

    char order = ' ';
    Mat img_lines;
    while(true) {
        lines.clear();
        HoughLines(edge, lines, rho_resolution, theta_resolution*CV_PI / 180, hough_thres, 0, 0);
        img_lines = rgb.clone();
        draw_lines(img_lines, lines, Scalar(0, 100, 100));

        order = show_waitClick_Order(img_lines, click);
        if(order == 'i') {
            hough_thres += 5;
            cout << "increased hough thres to:" << hough_thres << '\n';
        }else if(order == 'd'){
            hough_thres -= 5;
            cout << "decreased hough thres to:" << hough_thres << '\n';
        }else if(order == 'o'){
            return false;
        }else {
            break;
        }
    }


    std::sort(lines.begin(), lines.end(), PtDistCmpor(click));
    order = ' ';
    while(true) {
        for(size_t i = 0; i < lines.size(); ++i) {
            order = show_waitKey(img_lines.clone(), lines, i);
            if('y' == order) {
                selected = lines[i];
                return true;
            }
            if('o' == order) { /* over */
                return false;
            }
        }
    }
}

void select_lines(Sample & rst) {
    cv::Mat edge;
    Canny(rst.rgb, edge, 30, 70, 3);

    Mat img_selected= rst.rgb.clone();
    while(true) {
        draw_lines(img_selected, rst.lines, GREEN);
        imshow("selected lines", img_selected);
        waitKey(10);
        Vec2f slc;
        if(select_one_line(edge, rst.rgb.clone(), slc)) {
            rst.lines.push_back(slc);
        }else {
            break;
        }
    }
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
bool select_one_pt(vector<Point> pts, Mat img, Point & rst) {
    //Point click = show_waitClick(img);
    Point click;
    char order = show_waitClick_Order(img, click);
    if('o' == order) {
        return false;
    }
    sort(pts.begin(), pts.end(), Pt2PtCmpor(click));
    while('o' != order && 'y' != order) {
        for(size_t i = 0; i < pts.size(); ++i) {
            Mat clone = img.clone();
            circle(clone, pts[i], 5, GREEN);
            imshow("window", clone);
            order = waitKey(0);
            cout << "order =" << order << "\n";
            if('o' == order) {
                break;
            }
            if('y' == order) {
                rst = pts[i];
                break;
            }
            //assert('0' <= order && order <= '9');
            //int id = order-'0';
            //rst.pts.push_back(pts[i]);
            //rst.pt_ids.push_back(id);
            //cout << "insert " << id << "\n";
        }
    }
    return order == 'y';
}

void draw_pts(Mat & img, const vector<Point>& pts, cv::Scalar color=Scalar(0, 100, 100)) {
    for(const Point & pt: pts) {
        circle(img, pt, 5, color);
    }
}

void select_new_points(vector<Point> candidates, Sample & rst) {
    static int max_id = 0;
    cout << "---select new points---\n candidates size=" << candidates.size() << "\n";
    Mat img_total_pts = rst.rgb.clone();
    draw_pts(img_total_pts, candidates);
    for(; ; ++max_id) {
        cout << max_id << ":\n";
        draw_pts(img_total_pts, rst.pts, GREEN);
        imshow("selected pts", img_total_pts);
        waitKey(10);
        Point select_pt;
        if(select_one_pt(candidates, img_total_pts.clone(), select_pt)) {
            rst.pts.push_back(select_pt);
            rst.pt_ids.push_back(max_id);
        }else {
            break;
        }
    }
    cout << "===End select new points ===\n";
}

void select_points(Sample & prev, Sample & rst) {
    const int sz = rst.lines.size();
    if(0 == sz) {
        cout << "empty lines!!!\n";
    }

    vector<Point> pts;
    Mat img_total_pts = rst.rgb.clone();
    for(int i = 0; i < sz; ++i) {
        for(int j = i+1; j < sz; ++j) {
            Point pt = intersect(rst.lines[i], rst.lines[j]);
            if(0<= pt.x && pt.x < rst.rgb.cols &&\
                    0<= pt.y && pt.y < rst.rgb.rows) {
                pts.emplace_back(pt);
            }
        }
    }
    draw_pts(img_total_pts, pts);
    cout << "select points\n";
    for(int id: prev.pt_ids) {
        //draw_pts(img_total_pts, rst.pts, GREEN);
        imshow("selected pts", rst.draw());
        waitKey(10);
        cout << id << ":\n";
        Point select_pt;
        if(select_one_pt(pts, img_total_pts.clone(), select_pt)) {
            rst.pts.push_back(select_pt);
            rst.pt_ids.push_back(id);
        }
    }
    //const int max_id = *std::max_element(prev.pt_ids.cbegin(), prev.pt_ids.cend());
    select_new_points(pts, rst);
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
Sample process(Mat rgb, int id) {
    static Sample prev_smpl;
    static const string dst_dir = configs["result_dir"];
    static int prev_id = -1;
    if(!prev_smpl.rgb.empty()) {
        imshow("prev", prev_smpl.draw());
        waitKey(1);
    }
    Sample rst;
    rst.rgb = rgb;
    select_lines(rst);
    if(rst.lines.empty()) {
        cout << "! ignore this sample\n";
        return rst;
    }
    select_points(prev_smpl, rst);

    if(!prev_smpl.rgb.empty()) {
        Mat match_img = draw_match(prev_smpl, rst);
        imshow("match", match_img);
        imwrite(dst_dir+to_string(prev_id)+"--"+to_string(id)+".jpg", match_img);
        waitKey(1);
    }
    waitKey(1);
    prev_smpl = rst;
    prev_id = id;
    return rst;
}

int main(int argc, const char ** argv) {
    if(argc != 2) {
        cout << "Error! \nusage example ./bin/vanish_pt ../param/configs\n";
    }
    configs.init(argv[1]);
    const string dst_dir = configs["result_dir"];
    namedWindow("window", 1);
    setMouseCallback("window", CallBackFunc, NULL);
    const int start_id = configs["start_id"];
    const int last_id = configs["last_id"];
    for(int i= start_id; i <= last_id; ++i) {
        cout << "*****" << i << "*****\n";
        NewFrame cur{i};
        cur.read_frame();

        Mat rgb = cur.rgb();
        imshow("edge", cur.edge());
        waitKey(1);
        Sample rst = process(rgb, i);
        ofstream line_rst(dst_dir+to_string(i)+"_line.txt");
        rst.dump_lines(line_rst);

        ofstream pt_rst(dst_dir+to_string(i)+"_pt.txt");
        rst.dump_pts(pt_rst);
    }
}
