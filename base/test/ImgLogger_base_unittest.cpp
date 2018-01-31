#include <fstream>
#include "gtest/gtest.h"
#include "base.hpp"

namespace FS=boost::filesystem;
class ImgLogger_Test: public ::testing::Test{
    protected:
        virtual void SetUp(){

        }

        virtual void TearDown(){

        }
};

int main(int argc, char **argv) {
      ::testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
}

TEST_F(ImgLogger_Test, create_dir_test) {
    ImgLogger logger("/tmp", "track");
    FS::path dst("/tmp/track");
    EXPECT_TRUE(FS::exists(dst));
}
int count_files(FS::path p) {
    FS::directory_iterator itr(p), end;
    int cnt = 0;
    for(; itr != end; ++itr) {
        ++cnt;
    }
    return cnt;
}

TEST_F (ImgLogger_Test, add_file) {
    FS::create_directories(FS::path("/tmp/track"));
    std::fstream f;
    f.open("/tmp/track/1.txt", std::fstream::out);
    f << "hello" << std::endl;
    f.close();
    f.open("/tmp/track/2.txt", std::fstream::out);
    f << "world" << std::endl;
    f.close();
    EXPECT_EQ(2, count_files(FS::path("/tmp/track")));
}
TEST_F(ImgLogger_Test, clear_dir_test) {
    FS::create_directories(FS::path("/tmp/track"));
    std::fstream f;
    f.open("/tmp/track/1.txt", std::fstream::out);
    f << "hello" << std::endl;
    f.close();
    f.open("/tmp/track/2.txt", std::fstream::out);
    f << "world" << std::endl;
    f.close();

    ImgLogger logger("/tmp", "track");
    EXPECT_EQ(0, count_files(FS::path("/tmp/track")));
}
