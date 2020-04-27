/**
 * Simple class for inheritance or aggregation. 
 * It provides a single mutex (mutal exclusion). 
 * Using it as a super class is easier. 
 * Whereas aggregation has to be used in the case
 * when more than one mutex is needed. 
 * Author: Lars Widmer, www.lawi.ch
 */

#ifndef _MUTEX_H_
#define _MUTEX_H_

#include "pthread.h"

/*
#ifdef WIN32
// only for windows!!!
#include "pthread.h"
#endif
*/

class Mutex {

private:
	pthread_mutex_t mutex;

public:
	Mutex();
	virtual ~Mutex();

	void acquireMutex();
	void releaseMutex();

};

#endif // _MUTEX_H_



