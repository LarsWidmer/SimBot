/**
 * Header file containing various define statements. 
 */

#ifndef _DEFINES_H_
#define _DEFINES_H_

#include <iostream>

// Defines *****************************************************
#define COMPORT "COM1" // Port to connect to the sensor board
#define NODEID  0xA5 // ID of the sensor interface board

#define DOCAL  0 // 0 means we're not in calibration mode
#define CALSENSOR 0xB2 // if we calibrate: which sensor to calibrate
// #define CALFILENAME "calibration0xB1.txt" // unused: file name for writing calibration data
// #define TESTSENSOR 0xB1 // unused: test mode is disabled
// #define DOTEST 0 // test mode is disabled
#define CONSOLEMODE false // tells whether we are in console mode (for calibration) or not

// multi azimuth version
// used to distinguish between 6 cases of interpreting azimuth
// Every case represents a direction in 3 dimensional space
/*
#define ZPOS 1
#define ZNEG 2
#define YPOS 3
#define YNEG 4
#define XPOS 5
#define XNEG 6
*/

#define FLOORLEVEL -6.0f // used for coordinate axes

// Defines sizes of the limbs
#define UPPER_ARM_X 0.125f
#define UPPER_ARM_Y 0.75f
#define UPPER_ARM_Z 0.125f

#define ARM_X 0.125f
#define ARM_Y 1.0f
#define ARM_Z 0.125f

/*
#define TB1x 1.0f
#define TB1y 7.5f
#define TB1z 1.0f
*/
//#define Ch1x 1.0
//#define Ch1y 2.0
//#define Ch1z 0.75

#define CHEST_X 0.5
#define CHEST_Y 1.0
#define CHEST_Z 0.4
/*
#define Wsx 0.35
#define Wsy 0.35
#define Wsz 0.35
*/

#define HEAD_Z 0.4
#define HEAD_X 0.4
#define HEAD_Y 0.6
#define NECK 0.2

#define JOINT_X 0.2
#define JOINT_Y 0.2
#define JOINT_Z 0.2

// #define Shoulders 1.6f
#define SHOULDER_X 0.75
#define SHOULDER_Y 0.25
#define SHOULDER_Z 0.25

#define PELVIS_X 0.6
#define PELVIS_Y 0.2
#define PELVIS_Z 0.3
/*
#define PELVIS_X 0.6
#define PELVIS_Y 0.2
#define PELVIS_Z 0.3
*/

#define UPPER_LEG_X 0.15
#define UPPER_LEG_Y 1.35
#define UPPER_LEG_Z 0.15

#define LEG_X 0.15
#define LEG_Y 1.4
#define LEG_Z 0.15

#define FOOT_X 0.15
#define FOOT_Y 0.15
#define FOOT_Z 0.3

#define GAP 0.1

#define INHERIT_BEGIN_COORD 1
#define INHERIT_END_COORD 2
#define INHERIT_CENTER_COORD 3

#define TOP_SUSPENDED 0
#define BOTTOM_SUSPENDED 1

// measured at 3.20 volts
// used to convert the retrieved battery level to voltage
#define THREEVOLTS 990.0f

// Filenames where the calibration values are stored
// The files get automatically read but have to be written by hand (text editor)
/*
#define CALFILENAME0XB1 "calibration0xB1.txt"
#define CALFILENAME0XB2 "calibration0xB2.txt"
#define CALFILENAME0XB3 "calibration0xB3.txt"
#define CALFILENAME0XB4 "calibration0xB4.txt"
#define CALFILENAME0XB5 "calibration0xB5.txt"
*/
#define CALFILENAMEPRE "../calibration-data/calibration0x"
#define CALFILENAMEPOST ".txt"

#define CAPTUREFILE "Capture0X.txt"
#define REPLAY_FILEAME "capture0"
#define REPLAYFILE_EXTENSION ".txt"

#define MAX_NUMBER_OF_SENSORS 256
#define NUMBER_OF_SENSORS 10

// Basic Values for the game with the two balls
#define G1 0.1f
#define G2 0.001f
#define G3 0.001f
#define G4 0.99f
// #define G4 0.98f

#define LEFT_ARM 0xB6
#define LEFT_UPPER_ARM 0xB7
#define CHEST 0xB8
#define RIGHT_UPPER_ARM 0xB9
#define RIGHT_ARM 0xBA
/*
#define LEFT_LEG 0xBA
#define LEFT_UPPER_LEG 0xB9
#define PELVIS 0xB8
#define RIGHT_UPPER_LEG 0xB7
#define RIGHT_LEG 0xB6
*/
#define LEFT_LEG 0xB1
#define LEFT_UPPER_LEG 0xB2
#define PELVIS 0xB3
#define RIGHT_UPPER_LEG 0xB4
#define RIGHT_LEG 0xB5
/*
#define LEFT_LEG 0xB6
#define LEFT_UPPER_LEG 0xB7
#define PELVIS 0xB8
#define RIGHT_UPPER_LEG 0xB9
#define RIGHT_LEG 0xBA
*/
#define NO_SENSOR -1

/*
#define TEXTURE_PATTERN 0
#define TEXTURE_NIGHT 1
#define TEXTURE_ICE 2
#define TEXTURE_STONE 3
#define TEXTURE_WOOD 4
#define TEXTURE_BLUE 5
#define TEXTURE_WATER 6
#define TEXTURE_RED 7
*/

#define STUDENT_TEXTURE "water"
#define TEACHER_TEXTURE "greenSkin"
#define STUDENT_WRONG_TEXTURE "red"

#define BALL_TEXTURE_1 "ice"
#define BALL_TEXTURE_2 "stone"

#define TEXTURE_PATH "./resources/"
#define TEXTURE_EXTENSION ".bmp"

// #define FONTFILE "./resources/font1.bmp"
#define FONTFILE "./resources/font_white.bmp"
// #define BACKGROUNDFILE "./resources/heaven.bmp"
// #define BACKGROUNDFILE "resources/greypebbles.bmp"
#define BACKGROUNDFILE "resources/skyport.bmp"

/* delete and set to null */
#define safeDelete(x){if(x){delete x; x = NULL;}}

/* inform the user about a serious error and terminate program */
#define REPORT_ERROR(m){cerr << "ERROR: " << m << endl << "abnormal program termination! " << endl; exit(1); }

/* inform the user about a problem */
#define REPORT_PROBLEM(m){cerr << "PROBLEM: " << m << "; program continues ..." << endl; }

/* inform the user about something */
#define REPORT_INFO(m){cout << "INFO: " << m << endl; }

/* print debug message */
#define REPORT_DEBUG(m){cout << "DEBUG: " << m << endl; }

/* Defines M_PI if not yet defined */
#ifndef M_PI
#define M_PI    3.14159265358979323846f
#endif // M_PI

// Includes *****************************************************

#include <string>
// #include <vector>
// #include <iostream>
// #include <stdlib.h>

// Structs *****************************************************

#ifndef _NODEDATA
#define _NODEDATA
struct NodeData {
	float accX;
	float accY;
	float accZ;
	float magX;
	float magY;
	float magZ;
	float volts;
	bool valid;
};
#endif

#ifndef _LIMBCOORDINATES
#define _LIMBCOORDINATES
struct LimbCoordinates {
	float x;
	float y;
	float z;
};
#endif

#ifndef _LIMBNAMECOORDINATES
#define _LIMBNAMECOORDINATES
struct LimbNameCoordinates {
	std::string name;
	float x;
	float y;
	float z;
};
#endif

#ifndef _EULERROTATION
#define _EULERROTATION
struct EulerRotation {
	float xRotation;
	float yRotation;
	float zRotation;
};
#endif

#ifndef _NODEANDDATA
#define _NODEANDDATA
struct NodeAndData {
	NodeData data;
	int node;
};
#endif

/*
#ifndef _NODEDATA
#define _NODEDATA
typedef struct _NodeData {
	float accX;
	float accY;
	float accZ;
	float magX;
	float magY;
	float magZ;
	float volts;
	bool valid;
} NodeData;
#endif

#ifndef _EULERROTATION
#define _EULERROTATION
typedef struct _EulerRotation {
	float xRotation;
	float yRotation;
	float zRotation;
} EulerRotation;
#endif
*/

// Empty class *****************************************************

/**
 * No code in this class, just the define statements. 
 * Author: Lars;
 */
class Defines {

};

#endif /* _DEFINES_H_*/


