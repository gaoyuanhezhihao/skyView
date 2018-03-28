#include "Config.hpp"
#include <string>
#include "stdlib.h"
#include <iostream>
#include <fstream>
#include "str_proc.hpp"
#include <stdexcept>
#include <sstream>
using namespace std;


int & operator << (int & dst, universal_type & src){
    dst = atoi(src.raw_data.c_str());
    return dst;
}

double & operator << (double & dst, universal_type & src) {
    dst = atof(src.raw_data.c_str());
    return dst;
}


void read_config_file(const char * file_name) {
}

ostream & operator << (ostream & std_out, universal_type & src) {
    std_out << src.raw_data;
    return std_out;
}

string & operator <<(string & dst, universal_type & src) {
	dst = src.raw_data;
	return dst;
}

void Config_Map::read_one_line(string line) {
    if('#' == line[0]) {
        return ;
    }
    size_t split_id = line.find_first_of('=');
    if(string::npos == split_id) {
        return;
    }
    else {
        string feature_name = line.substr(0, split_id);
        trim_both(feature_name);
        string ftr_v = line.substr(split_id + 1);
        trim_both(ftr_v);
        //feature_value.set_value(ftr_v);
        universal_type ut(feature_name, ftr_v);
        map.insert(pair<string, universal_type> (feature_name, ut));
    }
}

void Config_Map::init_by_string(const string s) {
    std::istringstream iss(s);
    string line;
    while(getline(iss, line)){
        read_one_line(line);
    }
    is_inited = true;
    return;
}

void Config_Map::init(const char * fname) {
    string line;
    //unordered_map<string, universal_type> config_map;
    //universal_type feature_value;
    ifstream config_file(fname);
    if(!config_file) {
        throw std::invalid_argument("failed to read configure file");
    }
    if(config_file.is_open()) {
        while(getline(config_file, line)) {
            read_one_line(line);
        }
    }
    is_inited = true;
    return;
}
universal_type Config_Map::operator [](const string key) {
    if(!is_inited) {
        throw std::logic_error("config map used before init!!!");
    }
    if(map.end() == map.find(key)) {
        string err = string("Config_Map: key error '") + key + "'";
        throw std::out_of_range(err);
    }
    return map[key];
}
universal_type Config_Map::traceable_get(const string key, const char * fname, const int line_no) {
    try{
        return (*this)[key];
    }catch(const std::out_of_range & e) {
        string err_info = string("\nFile Name:")+string(fname)+string("\nLine number:") + to_string(line_no) + "\n" +e.what();
        throw std::out_of_range(err_info);
    }
}

Config_Map configs;
