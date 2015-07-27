#include "log.h"
#include <sys/time.h>

using namespace std;

static string levelString(LogLevel level);


/**
 * @brief Prints a message in the log (console).
 *
 * @param message The message to print.
 * @param funcName The string representation of the calling function.
 * @param line The line at which the function was called.
 * @param level The level of the message.
 */
void Log_def(string message, string funcName, int line, LogLevel level)
{
#if !defined(VERBOSE)
    if (level != WARNING && level != ERROR)
        return;
#endif

#if !defined(VERY_VERBOSE)
    if (level == DEBUG)
        return;
#endif

    char buf[10];
    time_t rawtime;
    struct tm* timeinfo;
    struct timeval tp;

    gettimeofday(&tp, NULL);
    rawtime = tp.tv_sec;
    timeinfo = localtime(&rawtime);
    strftime(buf, 10, "%T", timeinfo);

    cout << buf << "." << tp.tv_usec / 1000 << " ";
    cout << levelString(level) << ": ";

#ifdef VERY_VERBOSE
    cout << message << " (" << funcName << " at line " << line << ")" << endl;
#else
    cout << message << endl;

    //Avoids stupid warnings
    line++;
    funcName = "";
#endif
}

/**
 * @brief Get the string representation of a @ref LogLevel "log level".
 *
 * @param level The log level.
 * @return string The string representation.
 */
static string levelString(LogLevel level)
{
    switch (level)
    {
        case INFO:
            return "INFO";

        case WARNING:
            return "WARNING";

        case ERROR:
            return "ERROR";

        case DEBUG:
            return "DEBUG";
    }
}
