#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

#include <map>
#include <string>
#include <unordered_map>
using namespace std;

class universal_type{
    private:
        const string key;
        string raw_data;
    public:
        universal_type(const string key, const string data):key(key), raw_data(data) {
            ;
        }
        universal_type(){
            ;
        }
        bool set_value(string  raw_string){
            raw_data = raw_string;
            return true;
        }
        operator double()const{
            return atof(raw_data.c_str());
        }

        operator int() const {
            return atoi(raw_data.c_str());
        }
        operator string() const {
            return raw_data;
        }
        operator bool() const{
            if("true" == raw_data) {
                return true;
            }else if("false" == raw_data) {
                return false;
            }else{
                string err = string("Config file bug: '") + key + \
                             "' is not a bool" + "\n 'true' or 'false'";
                throw std::out_of_range(err);
            }
        }

     friend int & operator << (int & dst, universal_type &);
     friend double & operator << (double & dst, universal_type & src);
     friend ostream & operator << (ostream & std_out, universal_type & src);
	 friend string & operator <<(string & dst, universal_type & src);
};

class Config_Map{
    private:
        std::unordered_map<string, universal_type> map;
        bool is_inited = false;
        void read_one_line(string line);
    public:
        void init(const char * fname);
        void init_by_string(const string s);
        universal_type operator[](const string key); 
        universal_type traceable_get(const string key, const char * fname, const int line_no);
};

double & operator << (double & dst, universal_type & src);
int & operator << (int & dst, universal_type &src);
string & operator <<(string & dst, universal_type & src);

ostream & operator << (ostream & std_out, universal_type & src);

//using CONFIG=std::unordered_map<string, universal_type>;
extern Config_Map configs;
#define get_param(s) configs.traceable_get(s, __FILE__, __LINE__)
#endif // CONFIG_H_INCLUDED
