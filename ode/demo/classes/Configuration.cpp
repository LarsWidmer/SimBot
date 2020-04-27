#include "Configuration.h"

#include "Helper.h"

using namespace std;

/**
 * Private constructor clears the internal configuration cache. 
 * There are no configuration values available until a file is loaded. 
 */
Configuration::Configuration() {
	// cout << " #  Constructor starting empty configuration" << endl; 
	clear();
	warnEmpty = true;
}

/**
 * Private constructor loads the given file to the internal configuration cache. 
 * If they given file doesn't exist, an empty one gets created. 
 */
Configuration::Configuration(string fname) {
	// cout << " #  Constructor loading configuration " << fname << endl; 
	if (!Helper::fileExists(fname.c_str())) {
		// cout << " #  creating empty file: " << fname << endl;
		createFile(fname);
	}
	load(fname);
}

/**
 * Singleton constructor returns the one and only instance of this class. 
 * Singleton is used not to get strange behaviour because of multiple instances sharing the same file on disk. 
 * Usually there is only one configuration file per project. 
 * If multiple files should be used it's easy to load different files (one after another) with the singleton instance. 
 * Thats why getInstance needs the filename as parameter. 
 */
Configuration * Configuration::getInstance(string fname) {
	// cout << " #  Configuration getting Instance: " << fname << endl;
	static Configuration instance;
	if (instance.getFilename() != fname) {
		// cout << " #  loading another file: " << fname << endl;
		if (!Helper::fileExists(fname.c_str())) {
			// cout << " #  file not found, creating empty file: " << fname << endl;
			instance.createFile(fname);
		}
		instance.load(fname);
	}
	return &instance;
}

/**
 * Empty destructor. 
 */
Configuration::~Configuration() {
	// acquireMutex();
	// releaseMutex();
	// nothing to do
}

/**
 * Sets if a warning is printed to std-error if the field with the given
 * name can't be found or is empty. 
 * This only works with getStrings and getInts. 
 * getString and getInt always issue a warning. 
 */
void Configuration::setEmptyDataWarnings(bool flag) {
	warnEmpty = flag;
}

/**
 * Returns if a warning is printed to std-error if the field with the given
 * name can't be found or is empty. 
 * This only works with getStrings and getInts. 
 * getString and getInt always issue a warning. 
 */
bool Configuration::getEmptyDataWarnings() {
	return warnEmpty;
}

/**
 * Enables that a warning is printed to std-error if the field with the given
 * name can't be found or is empty. 
 * This only works with getStrings and getInts. 
 * getString and getInt always issue a warning. 
 */
void Configuration::enableEmptyDataWarnings() {
	warnEmpty = true;
}

/**
 * Disables that a warning is printed to std-error if the field with the given
 * name can't be found or is empty. 
 * This only works with getStrings and getInts. 
 * getString and getInt always issue a warning. 
 */
void Configuration::disableEmptyDataWarnings() {
	warnEmpty = false;
}

/**
 * Returns the name of the loaded file. 
 */
std::string Configuration::getFilename() {
	string tmp = "";
	acquireMutex();
	tmp = filename;
	releaseMutex();
	return tmp;
}

/**
 * Clears the internal configuration cache. 
 */
void Configuration::clear() {
	// data.name.clear();
	acquireMutex();
	filename = "";
	data.name = "Configuration File";
	data.dats.clear();
	releaseMutex();
}

/**
 * Removes a field from the configuration (in cache). 
 */
void Configuration::remove(string field) {
	acquireMutex();
	pair<ISVec::iterator,bool> p = locate(field);
	if (p.second) {
		data.dats.erase(p.first);
	}
	releaseMutex();
}

/**
 * Clears a field in the configuration. 
 */
void Configuration::clear(string field) {
	acquireMutex();
	pair<ISVec::iterator,bool> p = locate(field);
	if (p.second) {
		p.first->defs.clear();
		p.first->vals.clear();
	}
	releaseMutex();
}

/**
 * Return an iterator to the field with the given name. 
 * If the second part of the return value is false, the field with the given name hasn't been found.
 * If no field has been found, the returned iterator is invalid. 
 */
pair<ISVec::iterator,bool> Configuration::locate(string field) {
	pair<ISVec::iterator,bool> p;
	p.second = false;
	for (ISVec::iterator it=data.dats.begin(); it!=data.dats.end(); ++it) {
		if (it->name == field) {
			p.first = it;
			p.second = true;
		}
	}
	return p;
}

/**
 * Returns the strings saved in the field with the given name. 
 */
vector<string> Configuration::getStrings(string field) {
	vector<string> v;
	v.clear();
	if (warnEmpty) {
		acquireMutex();
		pair<ISVec::iterator,bool> p = locate(field);
		if (p.second) {
			if (p.first->defs.size() < 1) {
				cerr << "Warning: No strings in this field!" << endl;
			}
			v = p.first->defs;
		} else {
			cerr << "Warning: No field with this name!" << endl;
		}
		releaseMutex();
	} else {
		acquireMutex();
		pair<ISVec::iterator,bool> p = locate(field);
		if (p.second) {
			v = p.first->defs;
		}	
		releaseMutex();
	}
	return v;
}

/**
 * Returns the integers saved in the field with the given name. 
 */
vector<int> Configuration::getInts(string field) {
	vector<int> v;
	v.clear();
	if (warnEmpty) {
		acquireMutex();
		pair<ISVec::iterator,bool> p = locate(field);
		if (p.second) {
			if (p.first->vals.size() < 1) {
				cerr << "Warning: No integers in this field!" << endl;
			}
			v = p.first->vals;
		} else {
			cerr << "Warning: No field with this name!" << endl;
		}
		releaseMutex();
	} else {
		acquireMutex();
		pair<ISVec::iterator,bool> p = locate(field);
		if (p.second) {
			v = p.first->vals;
		}
		releaseMutex();
	}
	return v;
}

/**
 * Returns a single strings saved in the field with the given name. 
 * If the field contains more then one string an error is printed. 
 */
string Configuration::getString(string field) {
	string tmp = "";
	vector<string> strs = getStrings(field);
	if (strs.size() == 1) {
		tmp = strs.front();
	} else {
		cerr << "Configuration::getString ERROR: " << strs.size() << " values in this field!" << endl;
	}
	return tmp;
}

/**
 * Returns a single int saved in the field with the given name. 
 * If the field contains more then one int an error is printed. 
 */
int Configuration::getInt(string field) {
	int tmp = -1;
	vector<int> ints = getInts(field);
	if (ints.size() == 1) {
		tmp = ints.front();
	} else {
		cerr << "Configuration::getInt ERROR: " << ints.size() << " values in this field!" << endl;
	}
	return tmp;
}

/**
 * Sets the integer values of the field with the given name. 
 * If the field doesn't yet exist it's added to the configuration and filld with the given values. 
 */
void Configuration::set(string field, vector<string> values) {
	acquireMutex();
	pair<ISVec::iterator,bool> p = locate(field);
	if (p.second) {
		p.first->vals.clear();
		p.first->defs = values;
		p.first->num = 0;
	} else {
		IntStruct	is;
		is.name = field;
		is.vals.clear();
		is.defs = values;
		is.num = 0;
		data.dats.push_back(is);		
	}
	releaseMutex();
}

/**
 * Sets the string values of the field with the given name. 
 * If the field doesn't yet exist it's added to the configuration and filld with the given values. 
 */
void Configuration::set(string field, vector<int> values) {
	acquireMutex();
	pair<ISVec::iterator,bool> p = locate(field);
	if (p.second) {
		p.first->vals = values;
		p.first->defs.clear();
		p.first->num = 0;
	} else {
		IntStruct	is;
		is.name = field;
		is.defs.clear();
		is.vals = values;
		is.num = 0;
		data.dats.push_back(is);		
	}
	releaseMutex();
}

/**
 * Overloads the set function to set without giving actual values. 
 * Clears the field with the given name if it already exists. 
 * If the field doesn't yet exist it's added to the configuration as an empty field
 * with neither integer nor string values. 
 */
void Configuration::set(string field) {
	acquireMutex();
	pair<ISVec::iterator,bool> p = locate(field);
	if (p.second) {
		p.first->vals.clear();
		p.first->defs.clear();
		p.first->num = 0;
	} else {
		IntStruct	is;
		is.name = field;
		is.defs.clear();
		is.vals.clear();
		is.num = 0;
		data.dats.push_back(is);		
	}
	releaseMutex();
}

/**
 * Sets the string value of the field with the given name. 
 * If the field doesn't yet exist it's added to the configuration and filld with the given value. 
 */
void Configuration::set(string field, string value) {
	StringVec sv;
	sv.clear();
	sv.push_back(value);
	set(field,sv);
}

/**
 * Sets the integer value of the field with the given name. 
 * If the field doesn't yet exist it's added to the configuration and filld with the given value. 
 */
void Configuration::set(string field, int value) {
	IntVec iv;
	iv.clear();
	iv.push_back(value);
	set(field, iv);
}

/**
 * Loads the configuration from the file with the given name. 
 * The name may contain a full path if the file is located in another directory. 
 */
void Configuration::load(string fname) {
	if (fname == "") {
		fname = filename;
	}
	acquireMutex();
	filename = fname;
	data = XmlFileHandling::readIntData(fname);
	releaseMutex();
}

/**
 * Saves the configuration to a file with the given name. 
 * The name may contain a full path if the file is located in another directory. 
 */
void Configuration::save(string fname) {
	if (fname == "") {
		fname = filename;
	}
	acquireMutex();
	filename = fname;
	data.name = fname;
	XmlFileHandling::store(data,fname);
	releaseMutex();
}

/**
 * Returns the internal cache data structure. 
 * For most usages it shouldn't be necessary to use this function. 
 */
ISVec Configuration::getISVec() {
	ISVec isv;
	acquireMutex();
	isv = data.dats;
	releaseMutex();
	return isv;
}

/**
 * Prints the configuration data in a simple way. 
 */
void Configuration::print() {
	acquireMutex();
	XmlFileHandling::print(data);
	releaseMutex();
}

/**
 * Adds strings in the second paremeter to the field in the first parameter. 
 * If the field doesn't exist yet it gets created. 
 */
void Configuration::add(string field, vector<string> values) {
	acquireMutex();
	pair<ISVec::iterator,bool> p = locate(field);
	if (p.second) {
		for (vector<string>::iterator it=values.begin(); it!=values.end(); ++it) {
			p.first->defs.push_back(*it);			
		}
	} else {
		set(field,values);
	}
	releaseMutex();
}

/**
 * Adds integer values in the second paremeter to the field in the first parameter. 
 * If the field doesn't exist yet it gets created. 
 */
void Configuration::add(string field, vector<int> values) {
	acquireMutex();
	pair<ISVec::iterator,bool> p = locate(field);
	if (p.second) {
		for (vector<int>::iterator it=values.begin(); it!=values.end(); ++it) {
			p.first->vals.push_back(*it);			
		}
	} else {
		set(field,values);
	}
	releaseMutex();
}

/**
 * Adds the string in the second paremeter to the field in the first parameter. 
 * If the field doesn't exist yet it gets created. 
 */
void Configuration::add(string field, string value) {
	StringVec sv;
	sv.clear();
	sv.push_back(value);
	add(field,sv);
}

/**
 * Adds the integer in the second paremeter to the field in the first parameter. 
 * If the field doesn't exist yet it gets created. 
 */
void Configuration::add(string field, int value) {
	IntVec iv;
	iv.clear();
	iv.push_back(value);
	add(field, iv);
}

/**
 * Creats a field with the given name, if it doesn't yet exist. 
 */
void Configuration::add(string field) {
	pair<ISVec::iterator,bool> p = locate(field);
	if (!p.second) {
		set(field);
	}
}

/**
 * Creates an empty configuration file with the given filename. 
 */
void Configuration::createFile(string name) {
	if (!Helper::fileExists(name.c_str())) {
		IntData tmp;
		XmlFileHandling::store(tmp, name);
	}
}


