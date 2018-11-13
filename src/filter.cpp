#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "config/Config.hpp"
#include <queue>
#include <limits>

//const int INT_MIN=std::numeric_limits<int>::min();
//const int INT_MAX=std::numeric_limits<int>::max();

vector<int> dr{1, -1, 0, 0};
vector<int> dc{0, 0, -1, 1};
const int neib_sz = dr.size();

bool is_noise(cv::Size im_size, const int r_max, const int r_min, const int c_max, const int c_min, const int cnt) {
    static double thres = configs["filter_score_thres"];
    const int r_rg = r_max - r_min;
    const int c_rg = c_max - c_min;
    if(r_rg == 0 || c_rg == 0) {
        return true;
    }

    double score = (double)max(r_rg, c_rg)/(double)min(r_rg, c_rg); 
    score *=  double(cnt) / (im_size.height * im_size.width);
    return score < thres;
}

void dfs(const cv::Mat & edge_img, cv::Mat & rst, const int r, const int c) {
    std::vector<std::pair<int, int>> buf;
    CV_Assert(edge_img.at<uchar>(r, c) == 255);
    CV_Assert(rst.at<uchar>(r, c) == 1);
    CV_Assert(edge_img.size() == rst.size());
    rst.at<uchar>(r, c) = 0;
    buf.push_back({r, c});

    size_t i = 0;
    int r_min = r;
    int r_max = r;

    int c_min = c;
    int c_max = c;

    while(i < buf.size()) {
        const int r = buf[i].first;
        const int c = buf[i].second;
        ++i;
        for(int k = 0; k < neib_sz; ++i) {
            const int rr = r + dr[k];
            const int cc = c + dc[k];
            if(255 == edge_img.at<uchar>(rr, cc) &&
                    1 == rst.at<uchar>(rr, cc)) {
                continue;
            }
            rst.at<uchar>(rr, cc) = 0;
            buf.push_back({rr, cc});
            r_max = max(r_max, rr);
            r_min = min(r_max, rr);
            c_max = max(c_max, cc);
            c_min = min(c_min, cc);
        }
    }

    if(is_noise(rst.size(), r_max, r_min, c_max, c_min, buf.size())){
        return;
    }

}

void filter_noise_edge(const cv::Mat & edge_img, cv::Mat& rst) {

    CV_Assert(edge_img.depth() == CV_8U);
    CV_Assert(edge_img.channels() == 1);
    const int rows = edge_img.rows;
    const int cols = edge_img.cols;
    rst = cv::Mat::ones(rows, cols, CV_8U);
    for(int r = 0; r < rows; ++r) {
        for(int c = 0; c < cols; ++c) {
            if(0 == edge_img.at<uchar>(r, c) ||
                    1 != rst.at<uchar>(r, c)) {
                continue;
            }
        }
    }
}
