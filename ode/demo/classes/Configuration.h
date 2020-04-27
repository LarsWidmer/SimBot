/**
 * Configuration.h
 * Class offers access to configuration data stored in XML files. 
 * The system is flexible and not bound to a single application. 
 * Possible data formats are: 
 * - int
 * - string
 * - vector<int>
 * - vector<string>
 * The values are identified by a string name. 
 * A generic configure application allows to edit the 
 * configuration as it's stored in file. 
 * But Due to the human readable xml file it's often
 * simpler and faster to edit the file using a 
 * standard text editor. 
 */

#include "XmlFileHandling.h"

#include <iostream>
#include <vector>
#include <string>

#include "Mutex.h"

#ifndef _CONFIGURATION_H_
#define _CONFIGURATION_H_

class Configuration : Mutex {

	private:
		IntData data;
		std::string filename;
		std::pair<ISVec::iterator,bool> locate(std::string);

		bool warnEmpty;

		Configuration();
		Configuration(std::string);

	public:
		static Configuration * getInstance(std::string = "newconfig.xml");
		virtual ~Configuration();

		void setEmptyDataWarnings(bool);
		bool getEmptyDataWarnings();
		void enableEmptyDataWarnings();
		void disableEmptyDataWarnings();

		std::string getFilename();

		void clear();
		void clear(std::string);
		void remove(std::string);

		std::vector<std::string> getStrings(std::string);
		std::string getString(std::string);
		std::vector<int> getInts(std::string);
		int getInt(std::string);

		void set(std::string, std::vector<std::string>);
		void set(std::string, std::string);
		void set(std::string, std::vector<int>);
		void set(std::string, int);
		void set(std::string);

		void add(std::string, std::vector<std::string>);
		void add(std::string, std::string);
		void add(std::string, std::vector<int>);
		void add(std::string, int);
		void add(std::string);

		void load(std::string = "");
		void save(std::string = "");
		static void createFile(std::string);
		
		ISVec getISVec();
		void print();

};

#endif // _CONFIGURATION_H_

