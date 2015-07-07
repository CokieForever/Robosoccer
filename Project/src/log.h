#ifndef LOG_H
#define LOG_H

#include <iostream>
#include <string>
#include <sstream>

/**
 * @brief
 *
 */
typedef enum
{
    INFO,       /**< TODO */
    WARNING,    /**< TODO */
    ERROR,      /**< TODO */
    DEBUG       /**< TODO */
} LogLevel;

#define LOG_1_ARGS(message)                             Log_def(message, __PRETTY_FUNCTION__, __LINE__)
#define LOG_2_ARGS(message, level)                      Log_def(message, __PRETTY_FUNCTION__, __LINE__, level)

#define GET_3RD_ARG(arg1, arg2, arg3, ...)              arg3
#define LOG_MACRO_CHOOSER(...)                          GET_3RD_ARG(__VA_ARGS__, LOG_2_ARGS, LOG_1_ARGS, )

#define Log(...)                                        LOG_MACRO_CHOOSER(__VA_ARGS__)(__VA_ARGS__)


void Log_def(std::string message, std::string funcName, int line, LogLevel level = INFO);


/**
 * @brief
 *
 * @param t
 * @return std::string
 */
template<typename T> std::string ToString(T t)
{
    std::ostringstream s;
    s << t;
    return s.str();
}

#endif // LOG_H
