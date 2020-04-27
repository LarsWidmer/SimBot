#include "Helper.h"
/** 
 * kbhit(), a keyboard lookahead monitor
 * getch(), a blocking single character input from stdin
 * + various other tool methods
 */

#include <sstream>
#include <fstream>

#ifdef WIN32
// modification for compatibility with windows by Lars Widmer
#include <conio.h>
#include <iostream>
#include <windows.h>
#include <time.h> // for time

#else

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>

#endif

#ifdef WIN32
int Helper::kbhit() {
	return _kbhit();
}

int Helper::getch() {
	return _getch();
}

#else

#define CMIN  1

#ifdef CTIME
#undef CTIME
#endif

#define CTIME 1

/** 
 *  kbhit()  --  a keyboard lookahead monitor
 *
 *  returns the number of characters available to read.
 */
int Helper::kbhit(void) {
  int                   cnt = 0;
  int                   error;
  static struct termios Otty, Ntty;

  tcgetattr(0, &Otty);
  Ntty = Otty;

  Ntty.c_iflag          = 0;       /* input mode        */
  Ntty.c_oflag          = 0;       /* output mode       */
  Ntty.c_lflag         &= ~ICANON; /* raw mode              */
  Ntty.c_cc[VMIN]       = CMIN;    /* minimum chars to wait for */
  Ntty.c_cc[VTIME]      = CTIME;   /* minimum wait time */

	// Lars modification to supress screen echo
  Ntty.c_lflag &= ~ECHO;    /* disable echo         */

  if (0 == (error = tcsetattr(0, TCSANOW, &Ntty))) {
    struct timeval      tv;
    error     += ioctl(0, FIONREAD, &cnt);
    error     += tcsetattr(0, TCSANOW, &Otty);
    tv.tv_sec  = 0;
    tv.tv_usec = 100;   /* insert a minimal delay */
    select(1, NULL, NULL, NULL, &tv);
  }

  return (error == 0 ? cnt : -1 );

}

/** 
 *  getch()  --  a blocking single character input from stdin
 *
 *  Returns a character, or -1 if an input error occurs.
 *
 *  Conditionals allow compiling with or without echoing of
 *  the input characters, and with or without flushing pre-existing
 *  existing  buffered input before blocking.
 *
 */
int Helper::getch(void) {
  char                  ch;
  int                   error;
  static struct termios Otty, Ntty;

  fflush(stdout);
  tcgetattr(0, &Otty);
  Ntty = Otty;

  Ntty.c_iflag  =  0;           /* input mode           */
  Ntty.c_oflag  =  0;           /* output mode          */
  Ntty.c_lflag &= ~ICANON;  /* line settings        */

#if 1
  /* disable echoing the char as it is typed */
  Ntty.c_lflag &= ~ECHO;    /* disable echo         */
#else
  /* enable echoing the char as it is typed */
  Ntty.c_lflag |=  ECHO;        /* enable echo          */
#endif

  Ntty.c_cc[VMIN]  = CMIN;      /* minimum chars to wait for */
  Ntty.c_cc[VTIME] = CTIME;     /* minimum wait time    */

#if 0
/*
 * use this to flush the input buffer before blocking for new input
 */
#define FLAG TCSAFLUSH
#else
/*
 * use this to return a char from the current input buffer, or block if
 * no input is waiting.
 */
#define FLAG TCSANOW

#endif

 if (0 == (error = tcsetattr(0, FLAG, &Ntty))) {
    error  = read(0, &ch, 1 );                /* get char from stdin */
    error += tcsetattr(0, FLAG, &Otty);   /* restore old settings */
  }

  return (error == 1 ? (int) ch : -1 );

} 	
#endif

// ******************************************* LARS CODE ****************************

/**
 * Constructor
 * The class can be used with or without constructor.
 * Both is possible.
 * The echo fetures are only available on a valid instance.
 * This can be done via getInstance or the constructor.
 */
Helper::Helper() {
	echoMax = 3;
	restartuSeconds();
}

Helper* Helper::getInstance() {
	static Helper instance = Helper();
	return &instance;
}

/**
 * Simple platform independent sleep function. 
 * Author: Lars
 */
void Helper::specialSleep(const unsigned & microseconds) {
	#ifdef WIN32
	Sleep((microseconds+400)/1000);
	#else
	if (microseconds < 1000000) {
		usleep(microseconds);
	} else {
		sleep(microseconds / 1000000);
		usleep(microseconds % 1000000);
	}
	#endif
}

/**
 * Static helper method.
 * Returns the given string in upper case.
 */
string Helper::str2UpperCase(const string & str) {
	string out;
	out.clear();
	for (unsigned i=0; i<str.size(); i++) {
		out += toupper(str[i]);
	}
	return out;
}


/**
 * Returns the difference vector of the two given vectors.
 * result = vec1 - vec2
 */
LimbCoordinates Helper::vectorDifference(const LimbCoordinates & vec1, const LimbCoordinates & vec2) {
	LimbCoordinates lc;
	lc.x = vec1.x - vec2.x;
	lc.y = vec1.y - vec2.y;
	lc.z = vec1.z - vec2.z;
	return lc;
}

/**
 * Returns the sum vector of the two given vectors.
 * result = vec1 + vec2
 */
LimbCoordinates Helper::vectorSum(const LimbCoordinates & vec1, const LimbCoordinates & vec2) {
	LimbCoordinates lc;
	lc.x = vec1.x + vec2.x;
	lc.y = vec1.y + vec2.y;
	lc.z = vec1.z + vec2.z;
	return lc;
}

/**
 * Returns the scalar product of the two given vectors.
 */
float Helper::scalarProduct(const LimbCoordinates & vec1, const LimbCoordinates & vec2) {
	return vec1.x*vec2.x + vec1.y*vec2.y + vec1.z*vec2.z; // scalar product
}

/**
 * Returns the length of the given vector.
 */
float Helper::vectorLength(const LimbCoordinates & vec) {
	return sqrt(pow(vec.x,2)+pow(vec.y,2)+pow(vec.z,2));
}

/**
 * Normalizes the given vector to the length 1.
 */
LimbCoordinates Helper::normalize(const LimbCoordinates & vec) {
	float len = vectorLength(vec);
	LimbCoordinates lc;
	lc.x = vec.x / len;
	lc.y = vec.y / len;
	lc.z = vec.z / len;
	return lc;
}

/**
 * Returns the given vector streched by the given factor.
 * result = vec1 * factor
 */
LimbCoordinates Helper::multiply(const LimbCoordinates & vec, float factor) {
	LimbCoordinates lc;
	lc.x = vec.x * factor;
	lc.y = vec.y * factor;
	lc.z = vec.z * factor;
	return lc;
}


/**
 * Find vector product of the two given vectors.
 */
LimbCoordinates Helper::vectorProduct(const LimbCoordinates & vec1, const LimbCoordinates & vec2) {
	LimbCoordinates lc;
	lc.x = vec1.y*vec2.z - vec1.z*vec2.y;
	lc.y = vec1.z*vec2.x - vec1.x*vec2.z;
	lc.z = vec1.x*vec2.y - vec1.y*vec2.x;
	return lc;
}

/**
 * Returns the angle between the two vectors.
 * We assume all components of vec1 always to be positive.
 * Works only for outputs in the range of 0..PI.
 * There is no signum set.
 */
float Helper::angleBetweenVectors(const LimbCoordinates & vec1, const LimbCoordinates & vec2) {
	float len1 = vectorLength(vec1); // sqrt(pow(vec1.x,2)+pow(vec1.y,2)+pow(vec1.z,2));
	float len2 = vectorLength(vec2); // sqrt(pow(vec2.x,2)+pow(vec2.y,2)+pow(vec2.z,2));
	float scaP = scalarProduct(vec1,vec2); // vec1.x*vec2.x + vec1.y*vec2.y + vec1.z*vec2.z; // scalar product
	float ang  = acos(scaP/(len1*len2)); // angle
	return ang;
}

/**
 * Returns the euler angles of the given vector.
 * Depends on the method angleBetweenVectors.
 * The y rotation (roll) of the return value is always zero
 * because there is no roll for a vector.
 */
EulerRotation Helper::vectorAngles(const LimbCoordinates & vec) {
	// basic axes
	LimbCoordinates xAxes;
	xAxes.x = 1;
	xAxes.y = 0;
	xAxes.z = 0;
	LimbCoordinates yAxes;
	yAxes.x = 0;
	yAxes.y = 1;
	yAxes.z = 0;
	LimbCoordinates zAxes;
	zAxes.x = 0;
	zAxes.y = 0;
	zAxes.z = 1;

	// 2 dimensional projections
	LimbCoordinates xyPart;
	xyPart.x = vec.x;
	xyPart.y = vec.y;
	xyPart.z = 0;
	LimbCoordinates yzPart;
	yzPart.x = 0;
	yzPart.y = vec.y;
	yzPart.z = vec.z;
	LimbCoordinates xzPart;
	xzPart.x = vec.x;
	xzPart.y = 0;
	xzPart.z = vec.z;

	// rotation return value
	EulerRotation er;
	int sign;
	int defaultSign = 1;
	LimbCoordinates r;

	r = vectorProduct(yAxes,xyPart);
	sign = defaultSign * static_cast<int>(r.z / Helper::abs(r.z));
	er.zRotation = sign * angleBetweenVectors(yAxes,xyPart); // azimuth = yaw

	sign = defaultSign * static_cast<int>(vec.z / Helper::abs(vec.z));
	er.xRotation = sign * angleBetweenVectors(vec,xyPart); // pitch

	er.yRotation = 0; // roll of a vector is always zero

	return er;
}

/**
 * Returns the pitch and yaw (azimuth) of the given vector.
 * Depends on the method vectorAngles.
 * The y rotation (roll) of the return value is always zero
 * because there is no roll for a vector.
 */
vector<float> Helper::vectorAngles2(const LimbCoordinates & vec) {
	EulerRotation er = vectorAngles(vec);
	vector<float> ret;
	ret.push_back(er.xRotation); // pitch
	ret.push_back(er.zRotation); // yaw
	return ret;
}

/**
 * Sets the row width for inRow, echoBr ...
 */
void Helper::setRowWidth(const int & newEchoMax) {
	echoMax = newEchoMax;
}

/**
 * Converts a integer number to an ascii character.
 * There is no range checking.
 * The intention is only to use numbers between 1 and 24.
 * 1 gets to A.
 */
char Helper::num2Char(int num) {
	char c;
	c = static_cast<int>(num);
	c += 65;
	return c;
}

/**
 * Rotates the point at the given coordinate.
 * This method rotates first around the x, then y and finally around the z axis.
 * The new coordinates are returned.
 */
LimbCoordinates Helper::rotate(float x, float y, float z, float r1, float r2, float r3) {
	float t1,t2,t3,s1,s2,s3;
	LimbCoordinates lc;

	// rotate about x axis (roll)
	t1 = x;
	t2 = cos(r1)*y + sin(r1)*z;
	t3 = cos(r1)*z - sin(r1)*y;
	// rolling a point doesn't make sense at all
	// TODO: I think this can be commented out?

	// rotate about y axis (pitch)
	s1 = cos(r2)*t1 - sin(r2)*t3;
	s2 = t2;
	s3 = sin(r2)*t1 + cos(r2)*t3;

	// rotate about z axis (azimuth)
	lc.x = cos(r3)*s1 + sin(r3)*s2;
	lc.y = cos(r3)*s2 - sin(r3)*s1;
	lc.z = s3;

	return lc;
}

/**
 * Rotates the point at the given coordinate.
 * This method rotates first around the x, then y and finally around the z axis.
 * The new coordinates are returned.
 */
LimbCoordinates Helper::rotate(LimbCoordinates c, float r1, float r2, float r3) {
	return rotate(c.x, c.y, c.z, r1, r2, r3);
}

/**
 * Rotates the point at the given coordinate.
 * This method rotates first around the x, then y and finally around the z axis.
 * The new coordinates are returned.
 */
LimbCoordinates Helper::rotate(LimbCoordinates c, EulerRotation r) {
	return rotate(c.x, c.y, c.z, r.xRotation, r.yRotation, r.zRotation);
}

/**
 * Returns a point (x,y,z) on the unit circle for the given values of pitch an yaw.
 */
vector<float> Helper::unitCirclePoint(vector<float> py, float radius) {
	if (py.size() != 2) {
		cerr << "Wrong number of arguments (2 needed) for unitCirclePoint function: " << py.size() << endl;
	}
	/*
	float x = 0;
	float y = 0;
	float z = 0;
	float r = 1;
	float rp = 0;
	float pitch = py.at(0);
	float yaw = py.at(1);
	z = r * sin(pitch);
	rp = r * cos(pitch);
	y = rp * sin(yaw);
	x = rp * cos(yaw);
	vector<float> c;
	c.push_back(x);
	c.push_back(y);
	c.push_back(z);
	return c;
	*/
	LimbCoordinates lc = rotate(radius,0,0,0,py.at(0),py.at(1));
	vector<float> c;
	c.push_back(lc.x);
	c.push_back(lc.y);
	c.push_back(lc.z);
	return c;
}

/**
 * Returns a point (x,y,z) on the unit circle for the given values of pitch an yaw.
 */
vector<float> Helper::unitCirclePoint(vector<float> py) {
	return unitCirclePoint(py, 1);
}

/**
 * Returns a point (x,y,z) on the unit circle for the given values of pitch an yaw.
 */
vector<float> Helper::unitCirclePoint(EulerRotation er) {
	vector<float> py;
	py.push_back(er.yRotation);
	py.push_back(er.zRotation);
	return unitCirclePoint(py);
}

/**
 * Returns the distance of two pairs of pitch and yaw by interpreting the given values as points on the unit circle.
 */
float Helper::unitCircleDistance(vector<float> py1, vector<float> py2) {
	return vectorLength(vectorDifference(unitCirclePoint(py1),unitCirclePoint(py2)));
}

/**
 * Prints the given vector to standard output.
 */
void Helper::print2DecimalPlaces(const vector<float> & vec) {
	std::vector<float>::const_iterator it;
	for (it=vec.begin(); it!=vec.end(); ++it) {
		float out = (static_cast<int>((*it)*100))/100.0;
		cout << out << "; ";
	}
	cout << endl;
}

/**
 * Sets the row width for inRow, echoBr ...
 * Synonym to setRowWidth.
 */
void Helper::setRowSize(const int & newSize) {
	echoMax = newSize;
}

/**
 * Returns true if the file at the given path exists. 
 */
bool Helper::fileExists(const char *filename) {
  ifstream ifile(filename);
  return ifile;
}

/**
 * Returns a pair of two numbers. 
 * The first one tells the seconds since January 1, 1970. 
 * The second number tells the microseconds since the last full second. 
 */
pair<unsigned long int,unsigned long int> Helper::secAnduSec() {
	long int sec  = 0;
	long int usec = 0;

	#ifdef WIN32
	
	SYSTEMTIME st;    
	union _t {
		FILETIME    ft;
		__int64     tlong;
	} t;
	GetSystemTime(&st);
	SystemTimeToFileTime(&st,&t.ft);
	sec  = t.tlong / 10000;
	usec = t.tlong * 100;

	#else

	struct timeval tp;
	gettimeofday(&tp, NULL);
	sec  = tp.tv_sec;
	usec = tp.tv_usec; 
	
	#endif

	pair<long int, long int> timePair;
	timePair.first = sec;
	timePair.second = usec;
	return timePair;
}


/**
 * Returns the microseconds since the last full second of seconds().
 */
unsigned long int Helper::uSeconds() {
	pair<long int,long int> timePair = secAnduSec();
	return timePair.second;
}

/**
 * Returns the seconds since January 1, 1970. 
 */
unsigned long int Helper::seconds() {
	pair<long int,long int> timePair = secAnduSec();
	return timePair.first;
}

/**
 * Returns the microseconds since the program has been started. 
 * This is comfortable but after running the program for more than an hour an overflow occurs. 
 * So be aware of this! 
 */
unsigned long int Helper::uSecondsSinceStart() {
	pair<long int,long int> timePair = secAnduSec();
	long int diff  = timePair.first  - startSec;
	long int uDiff = timePair.second - startuSec;
	long int together = diff*1000000 + uDiff;
	return together;
}

/**
 * Resets the starting point for uSecondsSinceStart(). 
 * Since the class is a singleton the reset applies for any usage! 
 */
void Helper::restartuSeconds() {
	pair<long int,long int> timePair = secAnduSec();
	startSec  = timePair.first;
	startuSec = timePair.second;
}

