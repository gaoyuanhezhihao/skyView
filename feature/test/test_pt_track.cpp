#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <cstdio>
#include <dirent.h>
#include <errno.h>
#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "base.hpp"
#include "Config.hpp"
#include "debug.hpp"
#include "line.hpp"
#include "stereo.hpp"

using namespace cv;
using namespace std;


void preprocess(Frame & f) {
    detect_lines(f);
    f.lines = merge_close_lines(f.lines);
    get_inlier_intersects(f);
}

cv::Mat draw_track_match(const Mat & img1, const vector<Point2f> & pts1, const Mat & img2, const vector<Point2f> & pts2, const vector<pair<int, int>> & good_match) {
    CV_Assert(!img1.empty() && !img2.empty());
    cv::Mat imgMatches;
    cv::hconcat(img1, img2, imgMatches);
    for(const Point2f & pt1 :pts1) {
        circle(imgMatches, pt1, 3, RED, -1, 8);
    }
    for(const pair<int, int> & m: good_match) {
        const Point2f & p1 = pts1[m.first];
        const Point2f p2(pts2[m.second].x + img1.cols, pts2[m.second].y);
        circle(imgMatches, p1, 3, GREEN, -1, 8);
        circle(imgMatches, p2, 3, GREEN, -1, 8);
        cv::line(imgMatches, p1, p2, rand_color(), 1, CV_AA);
    }
    return imgMatches;
}

void read_frame(const int i, Frame & f) {
    static const string samples_dir = configs["samples"];
    static const string dst_dir = configs["result_dir"];
    static const int cols_sky_im = configs["cols_sky_im"];
    static const int rows_sky_im = configs["rows_sky_im"];

    f.rgb = cv::imread(samples_dir + to_string(i) + ".jpg");
    f.rgb = get_sky_view(f.rgb, cols_sky_im, rows_sky_im);
}
void test() {
    static const string samples_dir = configs["samples"];
    const string dst_dir = configs["result_dir"];
    const int id_start = configs["start_id"];
    const int id_last = configs["last_id"];
    

    Frame prevFrame{id_start};
    read_frame(id_start, prevFrame);
    preprocess(prevFrame);
    
    Size subPixWinSize(10,10), winSize(31,31);
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
    for(int i = id_start+1; i<= id_last; ++i) {
        cout << i<< "\n";
        printf("%d\n", i);

        Frame cur{i};
        read_frame(i, cur);
        preprocess(cur);

        //cvtColor(prevFrame.rgb, prevFrameGray, COLOR_BGR2GRAY);
        //cvtColor(cur.rgb, cur_gray, COLOR_BGR2GRAY);
        vector<Point2f> tracked_pts;
        vector<uchar> states{};
        vector<float> err;
        CV_Assert(!prevFrame.gray.empty());
        CV_Assert(!cur.gray.empty());
        calcOpticalFlowPyrLK(prevFrame.edge, cur.edge, prevFrame.keyPts,
                tracked_pts, states, err, winSize, 3,
                termcrit, 0, 0.001);
        vector<pair<int, int>> good_match;
        int k = 0;
        for(int i = 0; i < tracked_pts.size(); ++i) {
            if(! states[i]) {
                continue;
            }
            good_match.push_back({i, k});
            tracked_pts[k++] = tracked_pts[i];
        }
        tracked_pts.resize(k);
        Mat imgMatch = draw_track_match(prevFrame.rgb, prevFrame.keyPts, cur.rgb, tracked_pts, good_match);
        //cv::imshow("tracked Points", imgMatch);
        cv::imwrite(dst_dir+to_string(i-1) + "--" + to_string(i) + ".jpg", imgMatch);
        //waitKey(0);


        Mat cur_detect_img;
        cur.rgb.copyTo(cur_detect_img);
        draw_lines(cur_detect_img, cur.lines);
        draw_points(cur_detect_img, cur.keyPts);
        cv::imwrite(dst_dir + to_string(i) + "_feature.jpg", cur_detect_img);
        //detect_lines(cur);
        //get_inlier_intersects(cur);
        
        prevFrame = cur;
    }
}

int main(int argc, char ** argv) {
    if( argc != 2) {
        cout <<" Error! not enough param \n Usage: runner config.txt" << endl;
        return -1;

    }
    configs.init(argv[1]);
    test();
}

std::vector<string> get_img_pathes(string dir_path) {
    DIR *dp = opendir(dir_path.c_str());
    if(NULL == dp)  {
        cout << "Error(" << errno << ") opening" << dir_path << endl;
        exit(errno);
    }
    struct dirent *dirp = readdir(dp);
    std::vector<string> files_path;
    for( ;NULL != dirp;dirp = readdir(dp)) {
        std::string file_name=dirp->d_name;
        if(file_name == string(".")){
            continue;
        }
        if(file_name == string("..")) {
            continue;
        }
        files_path.push_back(dir_path + string("/")+ file_name);
    }
    return files_path;
}
