#include "Mutex.h"

/**
 * Constructor: Initializes the mutual exclusion. 
 */
Mutex::Mutex() {
	pthread_mutex_init(&mutex,NULL);
}

/**
 * Destructor: Destroys the mutual exclusion. 
 */
Mutex::~Mutex() {
	pthread_mutex_destroy(&mutex);
}

/**
 * Acquire the mutual exclusion. 
 */
void Mutex::acquireMutex() {
	pthread_mutex_lock(&mutex);
}

/**
 * Release the mutual exclusion. 
 */
void Mutex::releaseMutex() {
	pthread_mutex_unlock(&mutex);
}

