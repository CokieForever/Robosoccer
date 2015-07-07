#ifndef MUTEXDEBUG_H
#define MUTEXDEBUG_H

#include "log.h"

#define pthread_mutex_lock(m) do { Log("Trying to lock mutex", DEBUG); pthread_mutex_lock(m); Log("Locked mutex", DEBUG); } while (0)
#define pthread_mutex_unlock(m) do { pthread_mutex_unlock(m); Log("Unlocked mutex", DEBUG); } while (0)

#endif // MUTEXDEBUG_H
