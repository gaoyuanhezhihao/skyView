#ifndef POSE_H
#define POSE_H
#include "base.hpp"

void read_frame(const int i, Frame & f);
std::shared_ptr<Match> match_keyPoints(Frame & f1, Frame & f2);
void draw_matches(Match & m, cv::Mat & imgMatches);
bool get_motion(Match & m);
#endif //POSE_H
