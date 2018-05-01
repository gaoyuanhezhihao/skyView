#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <vector>
#include <initializer_list>

class LoggerStream{
    public:
        LoggerStream(std::initializer_list<std::ostream &> & handlers):_hdl(handlers) {}
        template<typename T> 
            LoggerStream & operator<<(const T& data) {
                for(std::ostream & hd: _hdl) {
                    hd << data;
                }
            }
    private:
        std::vector<std::ostream& >  _hdl;
};
#endif //LOGGER_H
