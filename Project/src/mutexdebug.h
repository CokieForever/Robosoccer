#ifndef MUTEXDEBUG_H
#define MUTEXDEBUG_H

#include <iostream>

#define pthread_mutex_lock(m) do { std::cout << "Trying to lock at line " << __LINE__ << std::endl; pthread_mutex_lock(m); std::cout << "Locked at line " << __LINE__ << std::endl; } while (0)
#define pthread_mutex_unlock(m) do { pthread_mutex_unlock(m); std::cout << "Unlocked at line " << __LINE__ << std::endl; } while (0)

#endif // MUTEXDEBUG_H
