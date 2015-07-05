#include "log.h"
#include <sys/time.h>

using namespace std;

static string levelString(LogLevel level);


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
