#include "str_proc.hpp"
using namespace std;
int trim_right(string & sr) {
    auto itr = sr.end();
    itr--;
    while(itr != sr.begin() && std::isspace(*itr)) {
        --itr;
    }
    sr.erase(itr+1, sr.end());
    return itr - sr.end() -1;
}

int trim_left(string & sr) {
    auto itr = sr.begin();
    while(itr != sr.end() && std::isspace(*itr)) {
        ++ itr;
    }
    sr.erase(sr.begin(), itr);
    return itr - sr.begin() ;
}

int trim_both(string & sr) {
    return trim_left(sr) + trim_right(sr);
}
