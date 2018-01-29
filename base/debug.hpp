#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <string.h>
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

extern cv::Mat debug_img;
#define SHOW(a) cout <<__FILENAME__ << ":" << __LINE__ << "  " #a << "="<< a << '\n'
