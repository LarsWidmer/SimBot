/**
 * Class with a collection of helping function for various purposes, like: 
 * - Reading from keyboard. 
 * - Vector analysis
 * - Time measuring
 * - Printing numbers
 *  
 * Author getch & kbhit: Floyd L. Davidson <http://www.ptialaska.net/~floyd>
 * Ukpeagvik (Barrow, Alaska)
 * Author: Lars Widmer ETHZ
 */

#ifndef __HELPER_INCLUDED__
#define __HELPER_INCLUDED__

#include "Defines.h"
#include <string>
#include <vector>
#include <iostream>
#include <math.h>
#include <sstream>

#ifndef WIN32  // Just for Linux
#include <stdlib.h>
#endif

using namespace std;

class Helper {

public:
	Helper();

	static int getch();
	static int kbhit();

	static Helper * getInstance();
	static string str2UpperCase(const string &);
	static void specialSleep(const unsigned &);
	static pair<unsigned long int, unsigned long int> secAnduSec();
	unsigned static long int uSeconds();
	unsigned static long int seconds();
	unsigned long int uSecondsSinceStart();
	void restartuSeconds();

	void setRowWidth(const int &);
	static char num2Char(int);

	template <class in_value, class out_value> static void convert(const in_value & ival, out_value & oval);
	template <class T> static string toString(const T &);

	template <class T> string inRow(const T &);
	template <class T> string inRow(const vector<T> &);
	template <class T> string inRow(const T &, const int &);
	template <class T> void echo(const T &);
	template <class T> void echo(const T &, const T &);
	template <class T> void echo(const T &, const T &, const T &);
	template <class T> void echo(const vector<T> &);
	template <class T> void echoBr(const T &);
	template <class T> void echoBr(const T &, const T &);
	template <class T> void echoBr(const T &, const T &, const T &);
	template <class T> void echoBr(const vector<T> &);
	void setRowSize(const int &);
	
	template <class T> static T abs(const T &);
	template <class T> static T getMax (const T &, const T &);

	template <class T> static bool has(const vector<T> &, const T &);
	template <class T> static int  locate(const vector<T> &, const T &);
	template <class T> static vector<T> sort(const vector<T> &);
	template <class T> static vector<T> duplicateElimination(const vector<T> &);
	template <class T> static vector<T> concat(const vector<T> &, const vector<T> &);
	template <class T> static vector<T> merge(const vector<T> &, const vector<T> &);

	static LimbCoordinates vectorDifference(const LimbCoordinates &, const LimbCoordinates &);
	static LimbCoordinates vectorSum(const LimbCoordinates &, const LimbCoordinates &);
	static EulerRotation vectorAngles(const LimbCoordinates &);
	static float vectorLength(const LimbCoordinates &);
	static float scalarProduct(const LimbCoordinates &, const LimbCoordinates &);
	static LimbCoordinates normalize(const LimbCoordinates &);
	static LimbCoordinates multiply(const LimbCoordinates &,float);
	static LimbCoordinates vectorProduct(const LimbCoordinates &, const LimbCoordinates &);
	static float angleBetweenVectors(const LimbCoordinates &, const LimbCoordinates &);

	static vector<float> vectorAngles2(const LimbCoordinates &);

	static LimbCoordinates rotate(float, float, float, float, float, float);
	static LimbCoordinates rotate(LimbCoordinates, float, float, float);
	static LimbCoordinates rotate(LimbCoordinates, EulerRotation);

	static vector<float> unitCirclePoint(vector<float>);
	static vector<float> unitCirclePoint(EulerRotation);
	static vector<float> unitCirclePoint(vector<float>, float);
	static float unitCircleDistance(vector<float>, vector<float>);

	static void print2DecimalPlaces(const vector<float> &);

	static bool fileExists(const char *);

	template <class T> static void checkSize(const vector<T> &, const vector<T> &);
	template <class T> static vector<T> vectorDifference(const vector<T> &, const vector<T> &);
	template <class T> static vector<T> vectorSum(const vector<T> &, const vector<T> &);
	template <class T> static float vectorLength(const vector<T> &);
	template <class T> static vector<double> normalize(const vector<T> &);
	template <class T> static float scalarProduct(const vector<T> &, const vector<T> &);
	template <class T> static float angleBetweenVectors(const vector<T> &, const vector<T> &);
	template <class T> static void print(const vector<T> &);
	template <class T> static string toString(const vector<T> &);
	template <class T> static bool equal(const vector<T> &, const vector<T> &);

private:
	int echoMax;
	unsigned long int startSec;
	unsigned long int startuSec;

};

// Template methods need to be included here
#include "Helper-Templates.cpp2"
// Therefore I put them to a special file

#endif // __HELPER_INCLUDED__



