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

int main(int argc, const char ** argv) {
    if(argc != 2) {
        cout << "Error! \nusage example ./bin/vanish_pt ../param/configs\n";
    }
    configs.init(argv[1]);
    const string dst_dir = configs["result_dir"];
    const int start_id = configs["start_id"];
    const int last_id = configs["last_id"];
    for(int i= start_id; i <= last_id; ++i) {
        cout << "*****" << i << "*****\n";
        NewFrame cur{i};
        cur.read_frame();

        Mat rgb = cur.rgb();
        imwrite(dst_dir + to_string(i)+".jpg", rgb);
    }
}
