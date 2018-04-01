#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <vector>
#include <string>
#include <string.h>
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

extern cv::Mat debug_img;
#define SHOW(a) cout <<__FILENAME__ << ":" << __LINE__ << "  " #a << "="<< a << '\n'
//#define SHOW(a) debug_show(__FILENAME__, __LINE__, #a, a)
void debug_show_img(cv::Mat img, const std::vector<cv::Vec2f> & lines, std::string title);
