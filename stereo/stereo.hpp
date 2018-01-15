#ifndef STEREO_H
#define STEREO_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "Config.hpp"
cv::Mat get_sky_view(cv::Mat & src_img, int cols, int rows);
#endif //STEREO_H
