/**
 * ConfigureClassification.cpp
 * Tool to edit xml configuration files. 
 * It is able to read, print, modify and save configuration data. 
 * Use this tool to configure the parameters used by supertux, recordgestures, etc. 
 */

#include "../classes/Helper.h"
#include "../classes/Configuration.h"

using namespace std;

Configuration * c;
string fname;
bool changed;

/**
 * Helper function to print an item of the configuration data structure. 
 */
void print(int i) {
	ISVec isv = c->getISVec();
	cout << isv.at(i).name << " = ";
	if (isv.at(i).vals.size() > 0) {
		cout << "Integer(s):";
		Helper::getInstance()->setRowSize(2);
		Helper::getInstance()->echo(isv.at(i).vals);
		// cout << "; ";
	} else {
		cout << "No Integers; ";
	}
	if (isv.at(i).defs.size() > 0) {
		cout << "String(s):";
		Helper::getInstance()->setRowSize(2);
		Helper::getInstance()->echo(isv.at(i).defs);
		// cout << "; ";
	} else {
		cout << "No Strings; ";
	}
	cout << endl;
}

/**
 * Print the whole configuration data structure. 
 */
void print() {
	ISVec isv = c->getISVec();
	if (isv.size() > 0) {
		for (int i=0; i<isv.size(); ++i) {
			print(i);
		}
	} else {
		cout << "(empty data structure)" << endl;
	}
}

/**
 * Prints the data of a given field. 
 */
void print(string str) {
	ISVec isv = c->getISVec();
	for (int i=0; i<isv.size(); ++i) {
		if (isv.at(i).name == str) {
			print(i);
		}
	}
}

/**
 * Empties the keyboard buffer. 
 */
void clearKeyboard() {
	while (Helper::kbhit()) {
		Helper::getch();
	}
}

/**
 * Get a single key from the keyboard. 
 * Return value -1 stands for exit. 
 */
int getKey() {
	clearKeyboard();
	char c = Helper::getch();
	clearKeyboard();
	if (c==27 || c=='q' || c=='x') {
		return -1;
	}
	if (c >= '0' && c <= '9') {
		return c - '0';
	}
	return c;
}

/**
 * Prints a prompt, uses getKey an prints the pressed key. 
 */
int readKey() {
	cout << " >>> ";
	int n = getKey();
	cout << n << endl;
	return n;
}

/**
 * Get to users answer (y/n) after a choice has been presented. 
 */
int getChoice() {
	clearKeyboard();
	cout << " (y/n) >>> " << flush;
	int n = getKey();
	clearKeyboard();
	if (n=='y' || n=='Y' || n=='z' || n=='Z') {
		// cout << endl;
		return true;
	}
	// cout << endl;
	return false;
}

/**
 * Read a string from the user. 
 */
string readStr() {
	cout << " >>> " << flush;
	string in;
  char name[256];
	clearKeyboard();
  cin.getline(name,256);
	clearKeyboard();
	in = name;
	// cout << endl;
	return in;
}

/**
 * Read an integer from the user. 
 * There's currently no explicit error handling. 
 */
int readInt() {
	cout << " >>> " << flush;
	int in = 0;
	clearKeyboard();
	cin >> in;
	clearKeyboard();
	// cout << endl;
	return in;	
}

/**
 * Open and read a new file. 
 */
void open() {
	cout << "Filename?";
	fname = readStr();
	c->load(fname);
	changed = false;
}

/**
 * Save the configuration to the current file. 
 */
void save() {
	c->save(fname);
	changed = false;
}

/**
 * Save the current configuration to a new file. 
 */
void saveAs() {
	fname = readStr();
	c->save(fname);
	changed = false;
}

/**
 * Clear the current configuration. 
 */
void clear() {
	cout << "All data will be purged, do you want to continue?";
	if (getChoice()) {
		c->clear();
		cout << " ... whole file cleared!" << endl;
		changed = true;
	} else {
		cout << " ... skipped" << endl;
	}
}

/**
 * Close the current configuration file. 
 */
void close() {
	if (changed) {
		cout << "Old file changed, do you want to save?";
		if (getChoice()) {
			save();
			cout << " ... saved!" << endl;
		} else {
			cout << " ... not saved!" << endl;
		}
	}
	fname = "unnamed.xml";
	c->clear();
	changed = false;
}

/**
 * Prints the whole configuration data of the current configuration file. 
 */
void show() {
	cout << "Whole configuration data set: " << endl;
	print();
}

/**
 * User interaction to change the data within a field. 
 */
void edit() {
	// print();
	cout << "Which field (name) to edit?";
	string str = readStr();
	print(str);
	bool done = false;
	while (!done) {
		string inS;
		int inI;
		cout << "Editing " << str << " field:" << endl;
		cout << 1 << " Clear field?" << endl;
		cout << 2 << " Add a string?" << endl;
		cout << 3 << " Add an integer?" << endl;
		cout << 4 << " Back to main menu?" << endl;
		// int n = Helper::getch()-'0';
		cout << "Your choice";
		int n = readKey();
		// int n = readInt();
		switch (n) {
			case 1 : 
				c->clear(str);
				changed = true;
				break;
			case 2 : 
				cout << "Enter string value to add?";
				inS = readStr();
				c->add(str,inS);
				changed = true;
				break;
			case 3 : 
				cout << "Enter integer value to add?";
				inI = readInt();
				c->add(str,inI);
				changed = true;
				break;
			case 4 : 
				done = true;
				return;
				break;
			default : 
				done = true;
				return;
				break;
		}
	}
}

/**
 * Removes a field (and its values) from the configuration. 
 */
void erase() {
	cout << "Which field (name) to delete?";
	string name = readStr();
	print(name);
	c->remove(name);
	changed = true;
}

/**
 * Add a field to the configuration. 
 */
void add() {
	cout << "Field name to create?";
	string name = readStr();
	c->add(name);
	changed = true;
}

/**
 * Main menu
 * Offers the choices, reads the user input, calls the according features, loops until exit. 
 */
void menu() {
	bool done = false;
	while (!done) {
		cout << "***** Main Menu: " << fname << " *****";
		if (changed) {
			cout << "*";
		}
		cout << endl;
		cout << 0 << " - exit" << endl;
		cout << 1 << " - open file ..." << endl;
		cout << 2 << " - save file" << endl;
		cout << 3 << " - save file as ..." << endl;
		cout << 4 << " - close file" << endl;
		cout << 5 << " - show all" << endl;
		cout << 6 << " - add field ..." << endl;
		cout << 7 << " - edit field ..." << endl;
		cout << 8 << " - delete field ..." << endl;
		cout << 9 << " - clear whole file" << endl;
		cout << "Your choice" << flush;
		// int in = Helper::getch()-'0';
		// cout << endl;
		int in = readKey();
		switch (in) {
			case -1 : 
				close();
				done = true;
				return;
				break;
			case 0 : 
				close();
				done = true;
				return;
				break;
			case 1 : 
				open();
				break;
			case 2 : 
				save();
				break;
			case 3 : 
				saveAs();
				break;
			case 4 : 
				close();
				break;
			case 5 : 
				print();
				break;
			case 6 : 
				add();
				break;
			case 7 :
				edit(); 
				break;
			case 8 : 
				erase();
				break;
			case 9 : 
				clear();
				break;
			default : 
				break;
		}
	}
}

/**
 * Initialize
 */
void constructor(int argc, char *argv[]) {
	cout << "Start Configuration Editor " << endl;
	fname = "unnamed.xml";
	changed = false;
	c = NULL;
	if (argc > 1) {
		fname = argv[1];
		if (!Helper::fileExists(fname.c_str())) {
			cout << "Creating File: " << fname << endl;
			Configuration::createFile(fname);
		}
		cout << "Editing File: " << fname << endl;
		c = Configuration::getInstance(fname);
	} else {
		cout << "Editing empty configuration" << endl;
		c = Configuration::getInstance();
	}	
}

/**
 * Clean up
 */
void destructor() {
	cout << "End Configuration Editor " << endl;
}

/**
 * Main routine
 */
int main (int argc, char *argv[]) {
	constructor(argc, argv);
	menu();
	destructor();
	return 0;
}



