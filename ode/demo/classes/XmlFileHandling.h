/**
 * XmlFileHandling.h
 * Helper class to store various data in an XML structure and write it to a file.
 */

#include <iostream>
#include <vector>
#include <string>
#include "Defines.h"
#include "../tinyxml/tinystr.h"
#include "../tinyxml/tinyxml.h"

#ifndef _XMLFILEHANDLING_NEIGHBOUR_HELPER_H_
#define _XMLFILEHANDLING_NEIGHBOUR_HELPER_H_

typedef std::vector<LimbCoordinates> CoordSet;
struct Neighbour {
	CoordSet positions;
	std::string name;
};
typedef std::vector<Neighbour> Neighbours;

typedef std::vector<std::string> StringVec;

typedef std::vector<int> IntVec;
struct IntStruct {
	IntVec vals;
	StringVec defs;
	std::string name;
	int num;
};
typedef std::vector<IntStruct> ISVec;
struct IntData {
	ISVec dats;
	std::string name;
};

typedef std::vector<float> FloatVec;
struct FloatStruct {
	FloatVec vals;
	StringVec defs;
	std::string name;
	float num;
};
typedef std::vector<FloatStruct> FSVec;
struct FloatData {
	FSVec dats;
	std::string name;
};

class XmlFileHandling {

	public:
		static void storeNeighboursOld(const Neighbours & neighbours, const std::string & filename);
		static Neighbours readNeighboursOld(const std::string & filename);
		static void store(const Neighbours & neighbours, const std::string & filename);
		static Neighbours readNeighbours(const std::string & filename);
		static void store(const FloatData & floatData, const std::string & filename);
		static FloatData readFloatData(const std::string & filename);
		static void store(const IntData & intData, const std::string & filename);
		static IntData readIntData(const std::string & filename);
		static FloatData convert(const Neighbours & neighbours, const std::string & name);
		static Neighbours convert(const FloatData & floatData);
		static FloatData getEmptyFloatData();
		static FloatStruct getEmptyFloatStruct();
		static IntData getEmptyIntData();
		static IntStruct getEmptyIntStruct();
		static void print(const FloatData & floatData);
		static void print(const Neighbours & neighbours);
		static void print(const IntData & intData);

};

#endif // _XMLFILEHANDLING_NEIGHBOUR_HELPER_H_



