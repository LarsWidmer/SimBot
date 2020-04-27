#include "XmlFileHandling.h"

/**
 * Neighbours are the data for Phils feature extraction algorithm.
 * Actually it's a list of limb positions according to different limbs.
 * This method stores such neighbours in a generic xml file.
 * There is no purpose string in this file like the name of the feature extraction algorithm.
 */
void XmlFileHandling::storeNeighboursOld(const Neighbours & neighbours, const std::string & filename) {
	TiXmlDocument doc;
	TiXmlDeclaration * decl = new TiXmlDeclaration("1.0", "", "");
	doc.LinkEndChild(decl);
	// typename
	Neighbours::const_iterator iter;
	CoordSet::const_iterator citer;
	TiXmlElement * limbTXE;
	TiXmlElement * coordTXE;
	for (iter=neighbours.begin(); iter!=neighbours.end(); ++iter) {
		limbTXE = new TiXmlElement("Limb");
		std::string tmpName = iter->name;
		limbTXE->SetAttribute("Name",tmpName.c_str());
		// limbTXE->SetAttribute("Name",static_cast<const string>(iter->name));
		for (citer=iter->positions.begin(); citer!=iter->positions.end(); ++citer) {
			coordTXE = new TiXmlElement("Position");
			coordTXE->SetDoubleAttribute("X",citer->x);
			coordTXE->SetDoubleAttribute("Y",citer->y);
			coordTXE->SetDoubleAttribute("Z",citer->z);
			limbTXE->LinkEndChild(coordTXE);
		}
		doc.LinkEndChild(limbTXE);
	}
	doc.SaveFile(filename.c_str());
}

/**
 * Neighbours are the data for Phils feature extraction algorithm.
 * Actually it's a list of limb positions according to different limbs.
 * This method reads such neighbours from a generic xml file.
 * There is no purpose string in this file like the name of the feature extraction algorithm.
 */
Neighbours XmlFileHandling::readNeighboursOld(const std::string & filename) {
	TiXmlDocument doc(filename.c_str());
	if (!doc.LoadFile()) {
		std::cerr << "Error: Couldn't load XML file!" << std::endl;
		exit(1);
	}

	TiXmlHandle hDoc(&doc);
	TiXmlElement* limbIter;

	Neighbours neighbours;
	neighbours.clear();
	Neighbour neighbour;
	LimbCoordinates lc;

	limbIter=hDoc.FirstChild("Limb").Element();
	// should always have a valid root but handle gracefully if it doesn't
	if (!limbIter) {
		std::cerr << "Error: No valid root in XML file!" << std::endl;
		exit(1);
	}

	for(limbIter; limbIter; limbIter=limbIter->NextSiblingElement()) {
		const char *pKey = limbIter->Value();
		const char *pName = limbIter->Attribute("Name");
		if (pName) {
			neighbour.name = pName;
			neighbour.positions.clear();
			if (strcmp(pKey,"Limb") != 0) {
				std::cerr << "Wrong element in XML file! >>>" << pKey << "<<<" << std::endl;
			}
			// std::cout << "Key: " << pKey << " Name: " << pName << std::endl;
			for(TiXmlElement* posIter=limbIter->FirstChildElement(); posIter; posIter=posIter->NextSiblingElement()) {
				posIter->QueryFloatAttribute("X",&lc.x);
				posIter->QueryFloatAttribute("Y",&lc.y);
				posIter->QueryFloatAttribute("Z",&lc.z);
				neighbour.positions.push_back(lc);
				// std::cout << "Coordinates: " << lc.x << "; " << lc.y << "; " << lc.z << std::endl;
			}
			neighbours.push_back(neighbour);
		}
	}

	return neighbours;
}

/**
 * Neighbours are the data for Phils feature extraction algorithm.
 * Actually it's a list of limb positions according to different limbs.
 * FloatData is a more or less generic data structure.
 * It is intended to store stuctured lists of float numbers.
 * There are fields for textual information about the purpose or the data definition.
 * This method converts from Neighbours to FloatData.
 */
FloatData XmlFileHandling::convert(const Neighbours & neighbours, const std::string & name) {
	FloatData fd;
	fd.name = name;
	for (Neighbours::const_iterator nit=neighbours.begin(); nit!=neighbours.end(); ++nit) {
		FloatStruct fs;
		fs.defs.push_back(nit->name);
		fs.name = nit->name;
		fs.num = 0;
		for (CoordSet::const_iterator cit=nit->positions.begin(); cit!=nit->positions.end(); ++cit) {
			fs.vals.push_back(cit->x);
			fs.vals.push_back(cit->y);
			fs.vals.push_back(cit->z);
		}
		fd.dats.push_back(fs);
	}
	return fd;
}

/**
 * Neighbours are the data for Phils feature extraction algorithm.
 * Actually it's a list of limb positions according to different limbs.
 * FloatData is a more or less generic data structure.
 * It is intended to store stuctured lists of float numbers.
 * There are fields for textual information about the purpose or the data definition.
 * This method converts from FloatData to Neighbours.
 */
Neighbours XmlFileHandling::convert(const FloatData & floatData) {
	Neighbours nbs;
	for (FSVec::const_iterator fsit=floatData.dats.begin(); fsit!=floatData.dats.end(); ++fsit) {
		Neighbour nb;
		nb.name = fsit->defs.at(0);
		for (FloatVec::const_iterator fit=fsit->vals.begin(); fit!=fsit->vals.end(); ++fit) {
			LimbCoordinates lc;
			lc.x = *fit;
			++fit;
			if (fit==fsit->vals.end()) {
				std::cerr << "incomplete coordinate data in xml file: missing y value" << std::endl;
			}
			lc.y = *fit;
			++fit;
			if (fit==fsit->vals.end()) {
				std::cerr << "incomplete coordinate data in xml file: missing z value" << std::endl;
			}
			lc.z = *fit;
			nb.positions.push_back(lc);
		}
		nbs.push_back(nb);
	}
	return nbs;
}

/**
 * Neighbours are the data for Phils feature extraction algorithm.
 * Actually it's a list of limb positions according to different limbs.
 * This method stores such neighbours in a generic xml file.
 * There is a purpose string in this file giving the name of the feature extraction algorithm.
 */
void XmlFileHandling::store(const Neighbours & neighbours, const std::string & filename) {
	store(convert(neighbours,"Philip Feature Extraction"),filename);
}

/**
 * Neighbours are the data for Phils feature extraction algorithm.
 * Actually it's a list of limb positions according to different limbs.
 * This method reads such neighbours from a generic xml file.
 * There is a purpose string in this file giving the name of the feature extraction algorithm.
 */
Neighbours XmlFileHandling::readNeighbours(const std::string & filename) {
	FloatData fd = readFloatData(filename);
	std::string name = "Philip Feature Extraction";
	if (name != fd.name) {
		std::cerr << "Wrong purpose string in file! " 	<< std::endl;
		std::cerr << "Expected: " 	<< name 			<< std::endl;
		std::cerr << "Found:    " 	<< fd.name 			<< std::endl;
	}
	return convert(fd);
}

/**
 * Neighbours are the data for Phils feature extraction algorithm.
 * Actually it's a list of limb positions according to different limbs.
 * This method prints the given Neighbours to standard out.
 */
void XmlFileHandling::print(const Neighbours & neighbours) {
	std::cout << "*** START:     Neighbours ***" << std::endl;
	for (Neighbours::const_iterator fsit=neighbours.begin(); fsit!=neighbours.end(); ++fsit) {
		std::cout << "Data-Name:     " << fsit->name << std::endl;
		std::cout << "- Numbers:     " << std::endl;
		for (CoordSet::const_iterator fit=fsit->positions.begin(); fit!=fsit->positions.end(); ++fit) {
			std::cout << " *   X: " << fit->x << std::endl;
			std::cout << "  *  Y: " << fit->y << std::endl;
			std::cout << "   * Z: " << fit->z << std::endl;
		}
	}
	std::cout << "*** END:       Neighbours ***" << std::endl;
}

/**
 * FloatData is a more or less generic data structure.
 * It is intended to store stuctured lists of float numbers.
 * There are fields for textual information about the purpose or the data definition.
 * This method stores FloatData to a generic xml file.
 */
void XmlFileHandling::store(const FloatData & floatData, const std::string & filename) {
	TiXmlDocument doc;
	TiXmlDeclaration * decl = new TiXmlDeclaration("1.0", "", "");
	doc.LinkEndChild(decl);

	TiXmlElement * datTXE;
	TiXmlElement * fraTXE;
	TiXmlElement * defTXE;
	TiXmlElement * valTXE;
	TiXmlElement * strTXE;
	TiXmlElement * floTXE;
	datTXE = new TiXmlElement("File");
	datTXE->SetAttribute("Name",floatData.name.c_str());
	// std::cout << floatData.name << std::endl;
	for (FSVec::const_iterator fsit=floatData.dats.begin(); fsit!=floatData.dats.end(); ++fsit) {
		fraTXE = new TiXmlElement("Data");
		fraTXE->SetAttribute("Name",fsit->name.c_str());
		fraTXE->SetDoubleAttribute("Num",fsit->num);

		defTXE = new TiXmlElement("Definition");
		for (StringVec::const_iterator sit=fsit->defs.begin(); sit!=fsit->defs.end(); ++sit) {
			// std::cout << *sit << std::endl;
			strTXE = new TiXmlElement("String");
			strTXE->SetAttribute("Val",sit->c_str());
			defTXE->LinkEndChild(strTXE);
		}
		fraTXE->LinkEndChild(defTXE);
		valTXE = new TiXmlElement("Numbers");
		for (FloatVec::const_iterator fit=fsit->vals.begin(); fit!=fsit->vals.end(); ++fit) {
			// std::cout << *fit << std::endl;
			floTXE = new TiXmlElement("Float");
			floTXE->SetDoubleAttribute("Num",*fit);
			valTXE->LinkEndChild(floTXE);
		}
		fraTXE->LinkEndChild(valTXE);
		datTXE->LinkEndChild(fraTXE);
	}
	doc.LinkEndChild(datTXE);
	doc.SaveFile(filename.c_str());
}

/**
 * FloatData is a more or less generic data structure.
 * It is intended to store stuctured lists of float numbers.
 * There are fields for textual information about the purpose or the data definition.
 * This method reads FloatData from a generic xml file.
 */
FloatData XmlFileHandling::readFloatData(const std::string & filename) {
	TiXmlDocument doc(filename.c_str());
	if (!doc.LoadFile()) {
		std::cerr << "Error: Couldn't load XML file!" << std::endl;
		exit(1);
	}

	FloatData fd = getEmptyFloatData();
	FloatStruct fs;

	TiXmlHandle hDoc(&doc);
	TiXmlElement * anchor;
	TiXmlElement * datIt;
	TiXmlElement * it;
	TiXmlNode * tnode;
	anchor = hDoc.FirstChild("File").Element();
	if (!anchor) {
		std::cerr << "Error: No valid root in XML file!" << std::endl;
		exit(1);
	}
	fd.name = anchor->Attribute("Name");

	// anchor->FirstChild("Data").Element();
	// for(datIt=anchor->FirstChildElement(); datIt; datIt=datIt->NextSiblingElement()) {
	for(datIt=anchor->FirstChildElement(); datIt; datIt=datIt->NextSiblingElement()) {
		if (strcmp("Data",datIt->Value()) != 0) {
			std::cerr << "Wrong items in Data file part! " << datIt->Value() << std::endl;
		}
		fs = getEmptyFloatStruct();
		fs.name = datIt->Attribute("Name");
		datIt->QueryFloatAttribute("Num",&fs.num);
		// for(it=datIt->FirstChild("Definition")->FirstChildElement(); it; it=it->NextSiblingElement()) {
		tnode = datIt->FirstChild("Definition");
		if (tnode) {
			for(it=tnode->FirstChildElement(); it; it=it->NextSiblingElement()) {
				if (strcmp("String",it->Value()) != 0) {
					std::cerr << "Wrong items in Definiton file part! " << it->Value() << std::endl;
				}
				std::string str = it->Attribute("Val");
				fs.defs.push_back(str);
			}
		}
		// for(it=datIt->FirstChild("Numbers")->FirstChildElement(); it; it=it->NextSiblingElement()) {
		tnode = datIt->FirstChild("Numbers");
		if (tnode) {
			for(it=tnode->FirstChildElement(); it; it=it->NextSiblingElement()) {
				if (strcmp("Float",it->Value()) != 0) {
					std::cerr << "Wrong items in Numbers file part! " << it->Value() << std::endl;
				}
				float fl;
				it->QueryFloatAttribute("Num",&fl);
				fs.vals.push_back(fl);
			}
		}
		fd.dats.push_back(fs);
	}

	return fd;
}

/**
 * FloatData is a more or less generic data structure.
 * It is intended to store stuctured lists of float numbers.
 * There are fields for textual information about the purpose or the data definition.
 * This method returns an empty structure.
 */
FloatData XmlFileHandling::getEmptyFloatData() {
	FloatData fd;
	fd.name.clear();
	fd.dats.clear();
	return fd;
}

/**
 * FloatStruct is part of FloatData
 * It is intended to store stuctured lists of float numbers.
 * There are fields for textual information about the purpose or the data definition.
 * This method returns an empty structure.
 */
FloatStruct XmlFileHandling::getEmptyFloatStruct() {
	FloatStruct fs;
	fs.name.clear();
	fs.defs.clear();
	fs.vals.clear();
	fs.num = 0;
	return fs;
}

/**
 * FloatData is a more or less generic data structure.
 * It is intended to store stuctured lists of float numbers.
 * There are fields for textual information about the purpose or the data definition.
 * This method prints a FloatData item to standard out.
 */
void XmlFileHandling::print(const FloatData & floatData) {
	std::cout << "*** START:     FloatData ***" << std::endl;
	std::cout << "*** Name:      " << floatData.name << std::endl;
	for (FSVec::const_iterator fsit=floatData.dats.begin(); fsit!=floatData.dats.end(); ++fsit) {
		std::cout << "Data-Name:     " << fsit->name << std::endl;
		std::cout << "Data-Number:   " << fsit->num << std::endl;
		std::cout << "- Definitions: " << std::endl;
		for (StringVec::const_iterator sit=fsit->defs.begin(); sit!=fsit->defs.end(); ++sit) {
			std::cout << "    " << *sit << std::endl;
		}
		std::cout << "- Numbers:     " << std::endl;
		for (FloatVec::const_iterator fit=fsit->vals.begin(); fit!=fsit->vals.end(); ++fit) {
			std::cout << "    " << *fit << std::endl;
		}
	}
	std::cout << "*** END:       FloatData ***" << std::endl;
}

/**
 * IntData is a more or less generic data structure.
 * It is intended to store stuctured lists of int numbers.
 * There are fields for textual information about the purpose or the data definition.
 * This method stores IntData to a generic xml file.
 */
void XmlFileHandling::store(const IntData & intData, const std::string & filename) {
	TiXmlDocument doc;
	TiXmlDeclaration * decl = new TiXmlDeclaration("1.0", "", "");
	doc.LinkEndChild(decl);

	TiXmlElement * datTXE;
	TiXmlElement * fraTXE;
	TiXmlElement * defTXE;
	TiXmlElement * valTXE;
	TiXmlElement * strTXE;
	TiXmlElement * intTXE;
	datTXE = new TiXmlElement("File");
	datTXE->SetAttribute("Name",intData.name.c_str());
	// std::cout << floatData.name << std::endl;
	for (ISVec::const_iterator isit=intData.dats.begin(); isit!=intData.dats.end(); ++isit) {
		fraTXE = new TiXmlElement("Data");
		fraTXE->SetAttribute("Name",isit->name.c_str());
		fraTXE->SetAttribute("Num",isit->num);

		defTXE = new TiXmlElement("Definition");
		for (StringVec::const_iterator sit=isit->defs.begin(); sit!=isit->defs.end(); ++sit) {
			// std::cout << *sit << std::endl;
			strTXE = new TiXmlElement("String");
			strTXE->SetAttribute("Val",sit->c_str());
			defTXE->LinkEndChild(strTXE);
		}
		fraTXE->LinkEndChild(defTXE);
		valTXE = new TiXmlElement("Numbers");
		for (IntVec::const_iterator iit=isit->vals.begin(); iit!=isit->vals.end(); ++iit) {
			// std::cout << *fit << std::endl;
			intTXE = new TiXmlElement("Int");
			intTXE->SetAttribute("Num",*iit);
			valTXE->LinkEndChild(intTXE);
		}
		fraTXE->LinkEndChild(valTXE);
		datTXE->LinkEndChild(fraTXE);
	}
	doc.LinkEndChild(datTXE);
	doc.SaveFile(filename.c_str());
}

/**
 * IntData is a more or less generic data structure.
 * It is intended to store stuctured lists of int numbers.
 * There are fields for textual information about the purpose or the data definition.
 * This method reads IntData from a generic xml file.
 */
IntData XmlFileHandling::readIntData(const std::string & filename) {
	TiXmlDocument doc(filename.c_str());
	if (!doc.LoadFile()) {
		std::cerr << "Error: Couldn't load XML file!" << std::endl;
		exit(1);
	}
	IntData id = getEmptyIntData();
	IntStruct is;
	TiXmlHandle hDoc(&doc);
	TiXmlElement * anchor;
	TiXmlElement * datIt;
	TiXmlElement * it;
	TiXmlNode * tnode;
	anchor = hDoc.FirstChild("File").Element();
	if (!anchor) {
		std::cerr << "Error: No valid root in XML file!" << std::endl;
		exit(1);
	}
	id.name = anchor->Attribute("Name");
	for(datIt=anchor->FirstChildElement(); datIt; datIt=datIt->NextSiblingElement()) {
		if (strcmp("Data",datIt->Value()) != 0) {
			std::cerr << "Wrong items in Data file part! " << datIt->Value() << std::endl;
		}
		is = getEmptyIntStruct();
		is.name = datIt->Attribute("Name");
		datIt->QueryIntAttribute("Num",&is.num);
		tnode = datIt->FirstChild("Definition");
		if (tnode) {
			for(it=tnode->FirstChildElement(); it; it=it->NextSiblingElement()) {
				if (strcmp("String",it->Value()) != 0) {
					std::cerr << "Wrong items in Definiton file part! " << it->Value() << std::endl;
				}
				std::string str = it->Attribute("Val");
				is.defs.push_back(str);
			}
		}
		tnode = datIt->FirstChild("Numbers");
		if (tnode) {
			for(it=tnode->FirstChildElement(); it; it=it->NextSiblingElement()) {
				if (strcmp("Int",it->Value()) != 0) {
					std::cerr << "Wrong items in Numbers file part! " << it->Value() << std::endl;
				}
				int in;
				it->QueryIntAttribute("Num",&in);
				is.vals.push_back(in);
			}
		}
		id.dats.push_back(is);
	}
	return id;
}

/**
 * FloatData is a more or less generic data structure.
 * It is intended to store stuctured lists of float numbers.
 * There are fields for textual information about the purpose or the data definition.
 * This method prints a FloatData item to standard out.
 */
void XmlFileHandling::print(const IntData & intData) {
	std::cout << "*** START:     IntData ***" << std::endl;
	std::cout << "*** Name:      " << intData.name << std::endl;
	for (ISVec::const_iterator isit=intData.dats.begin(); isit!=intData.dats.end(); ++isit) {
		std::cout << "Data-Name:     " << isit->name << std::endl;
		std::cout << "Data-Number:   " << isit->num << std::endl;
		std::cout << "- Definitions: " << std::endl;
		for (StringVec::const_iterator sit=isit->defs.begin(); sit!=isit->defs.end(); ++sit) {
			std::cout << "    " << *sit << std::endl;
		}
		std::cout << "- Numbers:     " << std::endl;
		for (IntVec::const_iterator iit=isit->vals.begin(); iit!=isit->vals.end(); ++iit) {
			std::cout << "    " << *iit << std::endl;
		}
	}
	std::cout << "*** END:       IntData ***" << std::endl;
}

/**
 * IntData is a more or less generic data structure.
 * It is intended to store stuctured lists of int numbers.
 * There are fields for textual information about the purpose or the data definition.
 * This method return an empty structure.
 */
IntData XmlFileHandling::getEmptyIntData() {
	IntData id;
	id.name.clear();
	id.dats.clear();
	return id;
}

/**
 * IntStruct is part of IntData
 * It is intended to store stuctured lists of int numbers.
 * There are fields for textual information about the purpose or the data definition.
 * This method return an empty structure.
 */
IntStruct XmlFileHandling::getEmptyIntStruct() {
	IntStruct is;
	is.name.clear();
	is.defs.clear();
	is.vals.clear();
	is.num = 0;
	return is;
}

