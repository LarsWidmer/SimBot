#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <math.h>
#include <ode/ode.h>
#include <cmath>
#include <drawstuff/drawstuff.h>
#include "classes/Configuration.h"
#include "classes/Helper.h"

#define DENSITY 0.5

#define POS_X 0
#define POS_Y 0
#define POS_Z 12

#define NUM 100					// max number of objects
#define GPB 3					// maximum number of geometries per body
#define MAX_CONTACTS 8          // maximum number of contact points per body
#define MAX_FEEDBACKNUM 20
#define GRAVITY         REAL(0.5)
#define USE_GEOM_OFFSET 1

#define WIDTH 0.3
#define DEPTH 0.2
#define RADIUS 0.09
#define FOOT 0.8
#define SHIN 0.84
#define LEG 0.66
#define DISTANCE 0.55
#define BODY 1.0
#define PUMP 0.7

#define SHOW_STARTUP false

#define FLIGHT_HEIGHT 5.0

#define XML_FILE "config.xml"
#define SCRIPT_FILE "motors.txt"

#define VELOCITY Configuration::getInstance(XML_FILE)->getInt("Velocity")/100.0
#define MAX_FORCE Configuration::getInstance(XML_FILE)->getInt("MaxiumumForce")/100.0
#define FRICTION_FORCE static_cast<double>(Configuration::getInstance(XML_FILE)->getInt("FrictionForce"))/1000.0
#define STARTUP_FRAMES Configuration::getInstance(XML_FILE)->getInt("StartUp")

// typedef std::pair<dBodyID,dGeomID> BodyGeo;
struct BodyGeo {
	dBodyID body;
	std::vector<dGeomID> geos;
};

typedef std::vector<BodyGeo> BodyGeos;

struct LegVars {
	dBodyID foot, toe, ankle, shin, knee, leg, hip;
	dJointID ankleHinge, kneeHinge, hipHinge;
	dReal finalHeight;
	std::vector<dGeomID> lowFootParts;
};

struct Command {
	std::vector<std::string> instr;
	std::vector<double> param;
};
typedef std::vector<Command> Script;
struct LegScript {
	Script ankle;
	Script knee;
	Script hip;
};

struct BodyScript {
	LegScript left;
	LegScript right;
};
struct StoredCondition {
	std::string name;
	bool flag;
};
typedef std::vector<StoredCondition> StoredConditions;

struct Label {
	std::string name;
	unsigned line;
};
typedef std::vector<Label> LabelTable;

static std::vector<unsigned> scriptCounters;

static std::vector<char> digitsChars;

static unsigned frame;

static dReal oldDiff; // Used in "PID" control loop at startup

static LegVars leftLeg, rightLeg;
static dBodyID body;
static dGeomID pump, bottleFront, bottleBack;

static bool ready;
static int control;
static dReal correctionVelocity;
static float viewAngle; 

static BodyScript flow;
static StoredConditions storedConditions;

static std::vector<std::string> predefinedConditions;

static bool balanceBody;

// dynamics and collision objects
static dWorldID world;
static dSpaceID space;
static std::vector<dJointID> joints;
static BodyGeos bodyGeos;
static dJointGroupID contactgroup;
static dJointGroupID jointgroup;

static LegScript getSideScripts(int i) {
	if (i == 0) {
		return flow.left;
	}
	return flow.right;
}

static Script getJointScript(int i, LegScript ls) {
	switch (i) {
		case 0 : return ls.ankle;
		case 1 : return ls.knee;
		case 2 : return ls.hip;
	}
	return ls.ankle;
}

static void resetScriptCounters() {
	scriptCounters.clear();
	for (int i=0; i<6; ++i) {
		scriptCounters.push_back(0);
	}
}

static void setScriptCounter(unsigned side, unsigned joint, unsigned value) {
	unsigned index = side*3+joint;
	scriptCounters.at(index) = value;
	if (scriptCounters.at(index) < 0) {
		scriptCounters.at(index) = 0;
	}
	if (getJointScript(joint,getSideScripts(side)).size() <= scriptCounters.at(index)) {
		scriptCounters.at(index) = 0;
	}
}

static void stepScriptCounter(unsigned side, unsigned joint) {
	unsigned index = side*3+joint;
	++scriptCounters.at(index);
	if (getJointScript(joint,getSideScripts(side)).size() <= scriptCounters.at(index)) {
		scriptCounters.at(index) = 0;
	}
}

static void stepScriptCounters() {
	for (unsigned side=0; side<2; ++side) {
		for (unsigned joint=0; joint<3; ++joint) {
			stepScriptCounter(side, joint);
		}
	}	
}

static void clearLeg(LegScript ls) {
	ls.ankle.clear();
	ls.knee.clear();
	ls.hip.clear();
}

static void clearFlow() {
	clearLeg(flow.left);
	clearLeg(flow.right);
}

static bool has(std::vector<char> v, char c) {
	for (std::vector<char>::iterator it=v.begin(); it!=v.end(); ++it) {
		if (c == *it) {
			return true;
		}
	}
	return false;
}

static bool containsOnly(std::string s, std::vector<char> vc) {
	for (std::string::iterator it=s.begin(); it!=s.end(); ++it) {
		if (!has(vc,*it)) {
			return false;
		}
	}
	return true;
}

static std::string intToString(int i) {
    std::ostringstream temp;
    temp << i;
    return temp.str();
}

static double stringToDouble(std::string str) {
	std::istringstream stm;
	stm.str(str);
	double d;
	stm >> d;
	return d;
}

static std::vector<char> getDigits() {
	if (digitsChars.size() < 9) {
		std::vector<char> vc;
		vc.clear();
		for (int i=0; i<=9; ++i) {
			vc.push_back(intToString(i).at(0));
		}
		digitsChars = vc;
	}
	return digitsChars;
}

static bool isFloat(std::string s) {
	std::vector<char> vc = getDigits();
	vc.push_back('.');
	vc.push_back('-');
	return containsOnly(s, vc);
}

static bool isInteger(std::string s) {
	std::vector<char> vc = getDigits();
	vc.push_back('-');
	return containsOnly(s, vc);
}

static bool isNumber(std::string s) {
	return isInteger(s);
}

static LegScript loadScript(std::string filename) {
	LegScript leg;
	clearLeg(leg);
	std::ifstream file(filename.c_str());
	std::string line;
	int limb = 0;
	Script scr;
	scr.clear();
	std::string recording = "";
	while (getline(file, line)) {
		bool done = false;
		bool firstWord  = true;
		std::string part;
		std::istringstream iss(line);
		Command com;
		com.instr.clear();
		com.param.clear();
		while (iss >> part && !done) {
			if (isFloat(part)) {
				// std::cout << "parameter: " << part << std::endl;
				com.param.push_back(stringToDouble(part));
			} else {
				// std::cout << "command: " << part << std::endl;
				if (part == "//") { // found comment
					done = true; 
				} else if (firstWord) {
					// std::cout << " ==> " << std::flush;					
					if (part == "Ankle:" || part == "Knee:" || part == "Hip:") {
						// std::cout << "new joint script " << std::flush;					
						if (recording == "Ankle:") {
							// std::cout << "ankle " << std::flush;					
							leg.ankle = scr;					
						} else if (recording =="Knee:") {
							// std::cout << "knee " << std::flush;					
							leg.knee = scr;							
						} else if (recording =="Hip:") {
							// std::cout << "hip " << std::flush;					
							leg.hip = scr;
						}
						recording = part;
						scr.clear();
						// std::cout << std::endl;	//  Just Testcode
					} else {
						com.instr.push_back(part);
						// std::cout << "instruction code saved" << std::endl;											
					}
				} else {
					com.instr.push_back(part);
					// std::cout << "word not on first position: " << part << " saved!" << std::endl;
				}
			}
			if (firstWord) {
				firstWord  = false;
			}
		}
		if (com.instr.size() > 0) {
			scr.push_back(com);
			// std::cout << "full command added" << std::endl;											
		}
	}
	if (recording == "Ankle:") {
		leg.ankle = scr;					
	} else if (recording == "Knee:") {
		leg.knee = scr;							
	} else if (recording == "Hip:") {
		leg.hip = scr;
	}
	file.close();
	return leg;
}

static void showScript(Script scr) {
	for (Script::iterator it=scr.begin(); it!=scr.end(); ++it) {
		if (it->instr.size() < 1) {
			std::cout << "No Command!!!" << std::endl;			
		}
		for (std::vector<std::string>::iterator it2=it->instr.begin(); it2!=it->instr.end(); ++it2) {
			std::cout << *it2 << " ";
		}
		for (std::vector<double>::iterator it2=it->param.begin(); it2!=it->param.end(); ++it2) {
			std::cout << *it2 << " ";
		}
		std::cout << std::endl;
	}
}

static void showLegScripts(LegScript leg) {
	std::cout << "*** Ankle:" << std::endl;
	showScript(leg.ankle);
	std::cout << "*** Knee:" << std::endl;
	showScript(leg.knee);
	std::cout << "*** Hip:" << std::endl;
	showScript(leg.hip);
	std::cout << "*** " << std::endl;
}

static void showFlow() {
	std::cout << "Full Body Motor Script" << std::endl;
	std::cout << "Left Leg: " << std::endl;
	showLegScripts(flow.left);
	std::cout << "Right Leg: " << std::endl;
	showLegScripts(flow.right);
}

static bool hasLeft(std::string s) {
	return (s.substr(0,5) == "left_");	
}

static bool hasRight(std::string s) {
	return (s.substr(0,6) == "right_");
}

static std::string onlyCommand(std::string s, bool left, bool right) {
	if (left) {
		return s.substr(5);
	}
	if (right) {
		return s.substr(6);
	}
	return s;
}

static std::string onlyCommand(std::string s) {
	return onlyCommand(s,hasLeft(s),hasRight(s));
}

static std::vector<Script> explodeScript(Script scr) {
	std::vector<Script> leftRight;
	Script left;
	Script right;
	left.clear();
	right.clear();
	for (Script::iterator it=scr.begin(); it!=scr.end(); ++it) {
		bool hl = hasLeft(it->instr.front());
		bool hr = hasRight(it->instr.front());
		Command c = *it;
		c.instr.front() = onlyCommand(it->instr.front(),hl,hr);
		// std::cout << ">>> " << c.instr << std::endl;
		bool both = !hl && !hr;
		int mult = 1;
		if (c.instr.front()=="Extend" || c.instr.front()=="Flex" || c.instr.front()=="Block" || c.instr.front()=="Relax" || c.instr.front()=="Wait") {
			mult = c.param.front();
		}
		if (hl || both) {
			for (int i=0; i<mult; ++i) {
				left.push_back(c);
			}
		}
		if (hr || both) {
			for (int i=0; i<mult; ++i) {
				right.push_back(c);
			}
		}
	}
	leftRight.push_back(left);
	leftRight.push_back(right);
	return leftRight;
}

static void explodeToFlow(LegScript leg) {
	std::vector<Script> leftRight;
	leftRight = explodeScript(leg.ankle);
	flow.left.ankle = leftRight.at(0);
	flow.right.ankle = leftRight.at(1);
	leftRight = explodeScript(leg.knee);
	flow.left.knee = leftRight.at(0);
	flow.right.knee = leftRight.at(1);
	leftRight = explodeScript(leg.hip);
	flow.left.hip = leftRight.at(0);
	flow.right.hip = leftRight.at(1);	
}

static LabelTable labelTable(Script scr) {
	// std::cout << "Label ... " << std::endl;	
	LabelTable table;
	table.clear();
	unsigned i = 0;
	for (Script::iterator it=scr.begin(); it!=scr.end(); ++it) {
		if (it->instr.front()=="Label") {
			// std::cout << "Label: " << it->instr.front() << " " << it->instr.at(1) << " = " << i << std::endl;
			Label l;
			l.name = it->instr.at(1);
			l.line = i;
			table.push_back(l);
		}
		++i;
	}
	return table;
}

static Script resolveTargets(Script scr, LabelTable table) {
	for (Script::iterator it=scr.begin(); it!=scr.end(); ++it) {
		if (it->instr.front()=="Loop") {
			// std::cout << "Loop: " << it->instr.front() << " " << it->instr.at(1) << std::endl;
			for (LabelTable::iterator it2=table.begin(); it2!=table.end(); ++it2) {
				if (it->instr.at(1) == it2->name) {
					// std::cout << " ==> " << it->instr.front() << " " << it->instr.at(1) << " = " << it2->line << std::endl;
					it->param.push_back(it2->line);
				}
			}
		}		
	}
	return scr;
}

static Script linkLoopsScript(Script scr) {
	return resolveTargets(scr,labelTable(scr));
}

static void linkLoops() {
	flow.left.ankle  = linkLoopsScript(flow.left.ankle);
	flow.left.knee   = linkLoopsScript(flow.left.knee);
	flow.left.hip    = linkLoopsScript(flow.left.hip);
	flow.right.ankle = linkLoopsScript(flow.right.ankle);
	flow.right.knee  = linkLoopsScript(flow.right.knee);
	flow.right.hip   = linkLoopsScript(flow.right.hip);
}

bool equalsCondition(StoredCondition a, StoredCondition b) {
	return (a.name == b.name);
}

bool hasCondition(StoredConditions scs, StoredCondition sc) {
	for (StoredConditions::iterator it=scs.begin(); it!=scs.end(); ++it) {
		if (equalsCondition(*it,sc)) {
			return true;
		}
	}
	return false;
}

bool hasCondition(StoredCondition sc) {
	return hasCondition(storedConditions, sc);
}

StoredConditions setCondition(StoredConditions scs, std::string name, bool value=true) {
	for (StoredConditions::iterator it=scs.begin(); it!=scs.end(); ++it) {
		if (it->name == name) {
			it->flag = value;
		}		
	}
	return scs;
}

void setCondition(std::string name, bool value=true) {
	for (StoredConditions::iterator it=storedConditions.begin(); it!=storedConditions.end(); ++it) {
		if (it->name == name) {
			it->flag = value;
		}		
	}
}

StoredConditions resetCondition(StoredConditions scs, std::string name) {
	return setCondition(scs,name,false);
}

void resetCondition(std::string name) {
	setCondition(name,false);
}

StoredConditions duplicateConditionElimination(StoredConditions scs) {
	StoredConditions ret;
	ret.clear();
	for (StoredConditions::iterator it=scs.begin(); it!=scs.end(); ++it) {
		if (!hasCondition(ret,*it)) {
			ret.push_back(*it);
		}
	}
	return ret;
}

void duplicateConditionElimination() {
	storedConditions = duplicateConditionElimination(storedConditions);
}

void printStoredConditions() {
	std::cout << "Stored Conditions: " << std::endl;
	for (StoredConditions::iterator it=storedConditions.begin(); it!=storedConditions.end(); ++it) {
		std::cout << "Name: " << it->name << "; Flag: " << it->flag << std::endl;
	}
}

void printStoredConditions(StoredConditions scs) {
	printStoredConditions(storedConditions);
}

StoredConditions getConditionsFromScript(Script s) {
	StoredConditions scs;
	scs.clear();
	for (Script::iterator it=s.begin(); it!=s.end(); ++it) {
		if (it->instr.size() >= 2 && (it->instr.at(0) == "Reset" || it->instr.at(0) == "Set")) {
			StoredCondition sc;
			sc.name = it->instr.at(1);
			sc.flag = false;
			scs.push_back(sc);
		}
	}
	return scs;
}

void addStoredConditions(StoredConditions scs) {
	storedConditions = Helper::concat(storedConditions,scs);
}

bool getUserDefinedCondition(std::string n) {
	for (StoredConditions::iterator it=storedConditions.begin(); it!=storedConditions.end(); ++it) {
		if (it->name == n) {
			return it->flag;
		}
	}
	std::cerr << "Condition not found: " << n << std::endl;
	return false;
}

void initStoredConditions(LegScript leg) {
	addStoredConditions(getConditionsFromScript(leg.hip));
	addStoredConditions(getConditionsFromScript(leg.knee));
	addStoredConditions(getConditionsFromScript(leg.ankle));
}	
	
void initStoredConditions() {
	initStoredConditions(flow.left);
	initStoredConditions(flow.right);
}	
	
static void init() {
    // create world
    world = dWorldCreate();
    space = dHashSpaceCreate(0);
    dWorldSetGravity(world, 0, 0, -1);

    dWorldSetCFM(world,1e-5);

	dWorldSetERP(world, 0.2);

    dCreatePlane(space,0,0,1,0);
    contactgroup = dJointGroupCreate(0);
    jointgroup = dJointGroupCreate(0);

	dWorldSetAutoDisableFlag(world,0);
	dWorldSetAutoDisableAverageSamplesCount(world, 10);
	dWorldSetLinearDamping(world, 0.0000001); // 0.00001
	dWorldSetAngularDamping(world, 0.005);
	dWorldSetMaxAngularSpeed(world, 20);

	dWorldSetContactMaxCorrectingVel (world,0.1);
	dWorldSetContactSurfaceLayer (world,0.001);

	joints.clear();
	bodyGeos.clear();	
	
	frame = 0;
	control = 3;
	viewAngle = 0.0;
	ready = false;

	digitsChars.clear();

	Configuration::getInstance(XML_FILE)->load();
	balanceBody = Configuration::getInstance(XML_FILE)->getInt("balanceBody") == 1;
	LegScript leg = loadScript(SCRIPT_FILE);
	
	clearFlow();
	explodeToFlow(leg);
	// waitPadding();

	storedConditions.clear();
	initStoredConditions();
	duplicateConditionElimination();
	printStoredConditions(); // TODO remove this debugging line??? 

	linkLoops();
	resetScriptCounters();

	predefinedConditions.clear();

	std::cout << "Script lengths: " 
			<< flow.left.ankle.size()  << "; " << flow.left.knee.size()  << "; " << flow.left.hip.size()  << "; "  
			<< flow.right.ankle.size() << "; " << flow.right.knee.size() << "; " << flow.right.hip.size() 
			<< std::endl;
}

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.
static void nearCallback (void *data, dGeomID o1, dGeomID o2) {
	int i;
	// exit without doing anything if the two bodies are connected by a joint
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);
	if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;

	dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
	for (i=0; i<MAX_CONTACTS; i++) {
		contact[i].surface.mode = dContactBounce | dContactSoftCFM;
		contact[i].surface.mu = dInfinity;
		contact[i].surface.mu2 = 0;
		contact[i].surface.bounce = 0.1;
		contact[i].surface.bounce_vel = 0.1;
		contact[i].surface.soft_cfm = 0.01;
	}

	if (int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,sizeof(dContact))) {
		dMatrix3 RI;
		dRSetIdentity (RI);
		const dReal ss[3] = {0.02,0.02,0.02};
		for (i=0; i<numc; i++) {
			dJointID c = dJointCreateContact (world,contactgroup,contact+i);
			dJointAttach (c,b1,b2);
		}
	}
}

// start simulation - set viewpoint
static void start() {
    static float xyz[3] = {2.0f,-2.0f,1.7600f};
    static float hpr[3] = {140.000f,-17.0000f,0.0000f};
    dsSetViewpoint(xyz,hpr);
}

static std::string getSide(int i) {
	switch (i) {
		case 0 : return "Left";
		case 1 : return "Right";
	}
	return "";
}

static std::string getJoint(int i) {
	switch (i) {
		case 0 : return "Ankle";
		case 1 : return "Knee";
		case 2 : return "Hip";
	}
	return "";
}

static LegVars getLeg(const unsigned side) {
	// side 0 ==> left; side 1 ==> right;
	if (side <= 0) { // more open
		return leftLeg;
	}
	return rightLeg;
}

static LegVars getLeg(const std::string side) {
	if (side == "left" || side == "Left" || side == "LEFT") {
		return leftLeg;
	}
	if (side == "right" || side == "Right" || side == "RIGHT") {
		return rightLeg;
	}
	std::cerr << "Unusable side information!" << std::endl;
	return leftLeg;
}

static void addForce(dReal x, dReal y, dReal z) {
	for (BodyGeos::iterator it=bodyGeos.begin(); it!=bodyGeos.end(); ++it) {
		dBodyAddForce(it->body, x,  y, z);
	}
}

static void setPredefinedConditions() {
	predefinedConditions.clear();
    predefinedConditions.push_back("LeftFootGround");
    predefinedConditions.push_back("RightFootGround");
    predefinedConditions.push_back("CenterOnLeftFoot");
    predefinedConditions.push_back("CenterOnRightFoot");
    predefinedConditions.push_back("LeftHipOnFoot");
    predefinedConditions.push_back("RightHipOnFoot");
    predefinedConditions.push_back("LeftKneeExtended");
    predefinedConditions.push_back("RightKneeExtended");
    predefinedConditions.push_back("LeftKneeFlexed");
    predefinedConditions.push_back("RightKneeFlexed");
    // predefinedConditions.push_back("LeaningForward");
    // predefinedConditions.push_back("LeaningBackward");
    // predefinedConditions.push_back("LeaningLeft");
    // predefinedConditions.push_back("LeaningRight");
    // predefinedConditions.push_back("LeftAnkleExtended");
    // predefinedConditions.push_back("RightAnkleExtended");
    // predefinedConditions.push_back("LeftAnkleFlexed");
    // predefinedConditions.push_back("RightAnkleFlexed");
}

static bool isPredefinedCondition(std::string n) {
	if (predefinedConditions.size() == 0) {
		setPredefinedConditions();
	}
	return (Helper::has(predefinedConditions,n));
}

static bool onGround(LegVars lv) {
	dReal thereshold = DEPTH/4.0+0.01;
	for (std::vector<dGeomID>::iterator it=lv.lowFootParts.begin(); it!=lv.lowFootParts.end(); ++it) {
		dGeomID trans = dGeomTransformGetGeom(*it);
		const dReal * relPos = dGeomGetPosition(trans);
		dVector3 absPos;
		dBodyGetRelPointPos(lv.foot, relPos[0], relPos[1], relPos[2], absPos);
		// std::cout << "geo postion X: " << absPos[0] << "; Y: " << absPos[1] << "; Z: " << absPos[2] << std::endl;
		if (absPos[2] < thereshold) {
			return true;
		}
	}
	return false;
}

static std::vector<dReal> removeLast(const std::vector<dReal> a) {
	std::vector<dReal> ret;
	ret.clear();
	for (int i=0; i<a.size()-1; ++i) {
		ret.push_back(a.at(i));
	}
	return ret;
}

static dReal pointDistance(const dReal a, const dReal b) {
	return sqrt(pow(a,2)+pow(b,2));
}

static dReal pointDistance(const std::vector<dReal> a, const std::vector<dReal> b) {
	if (a.size() != b.size()) {
		std::cerr << "distance error: vectors not the same size" << std::endl;
		return 0;
	}
	std::vector<dReal> c;
	c.clear();
	for (unsigned i=0; i<a.size(); ++i) {
		c.push_back(a.at(i)-b.at(i));
	}
	dReal sum = 0;
	for (std::vector<dReal>::iterator it=c.begin(); it!=c.end(); ++it) {
		sum += pow(*it,2);
	}
	return sqrt(sum);
}

static std::vector<dReal> array2vec(const dReal* a, unsigned n) {
	std::vector<dReal> r;
	r.clear();
	for (unsigned i=0; i<n; ++i) {
		r.push_back(a[i]);
	}
	return r;
}

static bool kneeFlexed(LegVars lv) {
	dReal dist = pointDistance(SHIN/2.0+DEPTH/2.0,LEG/2.0+DEPTH/2.0);
	dReal curr = pointDistance(array2vec(dBodyGetPosition(lv.shin),3),array2vec(dBodyGetPosition(lv.leg),3));
	return curr <= dist+0.0001;
}

static bool kneeExtended(LegVars lv) {
	dReal curr = pointDistance(array2vec(dBodyGetPosition(lv.shin),3),array2vec(dBodyGetPosition(lv.leg),3));
	return curr >= SHIN/2.0+LEG/2.0+DEPTH-0.0001;
}

static bool centerOnFoot(LegVars lv) {
	dReal diff = pointDistance(removeLast(array2vec(dBodyGetPosition(body),3)),removeLast(array2vec(dBodyGetPosition(lv.foot),3)));
	dReal thereshold = DEPTH;
	// std::cout << "Center Diff: " << diff << std::endl;
	return (thereshold > diff);
}

static bool hipOnFoot(LegVars lv) {
	dReal diff = pointDistance(removeLast(array2vec(dBodyGetPosition(lv.hip),3)),removeLast(array2vec(dBodyGetPosition(lv.foot),3)));
	dReal thereshold = DEPTH;
	// std::cout << "Center Diff: " << diff << std::endl;
	return (thereshold > diff);
}

/*
static bool leaningLeft() {
	dReal bodyX = dBodyGetPosition(body)[0];
	dReal footX = dBodyGetPosition(leftLeg.foot)[0];
	dReal thereshold = DEPTH;
	return (footX-thereshold < bodyX);
}

static bool leaningRight() {
	dReal bodyX = dBodyGetPosition(body)[0];
	dReal footX = dBodyGetPosition(rightLeg.foot)[0];
	dReal thereshold = DEPTH;

	return (footX+thereshold > bodyX);
}
*/

static bool getPreDefinedCondition(std::string cond) {
	if (cond == "LeftFootGround") 
		return onGround(leftLeg);
	if (cond == "RightFootGround")
		return onGround(rightLeg);
	if (cond == "CenterOnLeftFoot")
		return centerOnFoot(leftLeg);
	if (cond == "CenterOnRightFoot")
		return centerOnFoot(rightLeg);
	if (cond == "LeftKneeExtended")
		return kneeExtended(leftLeg);
	if (cond == "RightKneeExtended")
		return kneeExtended(rightLeg);
	if (cond == "LeftKneeFlexed")
		return kneeFlexed(leftLeg);
	if (cond == "RightKneeFlexed")
		return kneeFlexed(rightLeg);
	if (cond == "LeftHipOnFoot")
		return hipOnFoot(leftLeg);
	if (cond == "RightHipOnFoot")
		return hipOnFoot(rightLeg);
	std::cerr << "No predefined condition " << cond << " found!" << std::endl;
	return false;
}

static void testPreDefinedConditions() {
/*
	std::cout << "testPreDefinedConditions: " << " ********************** " << std::endl;
	if (predefinedConditions.size() == 0) {
		setPredefinedConditions();
	}
	for (std::vector<std::string>::iterator it=predefinedConditions.begin(); it!=predefinedConditions.end(); ++it) {
		if (getPreDefinedCondition(*it)) {
			std::cout << "Condition: " << *it << " is fullfilled! " << std::endl;
		}
	}	
	// */
	/*
	if (getPreDefinedCondition("CenterOnLeftFoot")) {
		std::cout << "Condition: is fullfilled! " << std::endl;
	} else {
		std::cout << "Condition: ***************************** " << std::endl;
	}
	// */
}

static bool condition(std::string cond) {	
	if (isPredefinedCondition(cond)) {
		return getPreDefinedCondition(cond);
	}
	return getUserDefinedCondition(cond);
}

/**
 * Interprets the next command from the script. 
 * And returns the according velocity and force. 
 */
static std::vector<double> getFmaxVel(int side, int joint) {
	std::vector<double> ret;
	ret.clear();
	double f = FRICTION_FORCE;
	double v = 0;
	bool skip = true;	
	while (skip) {
		skip = false;
		unsigned index = side*3+joint;
		unsigned pc = scriptCounters.at(index);
		unsigned size = scriptCounters.size();
		Script scr = getJointScript(joint,getSideScripts(side));
		if (pc < scr.size()) {
			Command c = scr.at(pc);
			if (c.instr.front() == "Relax") {
				f = FRICTION_FORCE;
				v = 0;
			} else if (c.instr.front() == "Wait") {
				f = FRICTION_FORCE;
				v = 0;
			} else if (c.instr.front() == "Extend") {
				f = MAX_FORCE;
				if (joint==0 || joint==2) {
					v = VELOCITY;
				} else {
					v = -VELOCITY;
				}
			} else if (c.instr.front() == "Flex") {
				f = MAX_FORCE;
				if (joint==0 || joint==2) {
					v = -VELOCITY;
				} else {
					v = VELOCITY;
				}
			} else if (c.instr.front() == "Block") {
				f = MAX_FORCE;
				v = 0;				
			} else if (c.instr.front() == "Loop") {
				skip = true;
				if (c.instr.size() < 4) {
					// unconditional Loop
					setScriptCounter(side, joint, c.param.front());
				} else {
					// std::cout << "Loop with 4 params: " 
					//		<< c.instr.at(0) << "; " << c.instr.at(1) << "; " << c.instr.at(2) 
					//		<< "; " << c.instr.at(3) << "; " << std::endl;
					if (c.instr.at(2) == "Until") {
						bool cond = false;
						if (c.instr.at(3) == "Not" && c.instr.size() > 4) {
							cond = !condition(c.instr.at(4));
						} else {
							cond = condition(c.instr.at(3));
						}
						// conditional Loop
						if (cond) {
							// std::cout << "total condition true" << std::endl;
							stepScriptCounter(side, joint);
						} else {
							// std::cout << "total condition false" << std::endl;
							setScriptCounter(side, joint, c.param.front());
						}
					} else {
						std::cerr << "Missing Until after " << c.instr.front() << std::endl;
						stepScriptCounter(side, joint);
					}
				}
			} else if (c.instr.front() == "Label") {
				// ignore labels
				skip = true;
				stepScriptCounter(side, joint);
			} else if (c.instr.front() == "Set") {
				skip = true;
				if (c.instr.size() > 1) {
					setCondition(c.instr.at(1));
				} else {
					std::cerr << "Missing parameter for command " << c.instr.front() << std::endl;
				}
				stepScriptCounter(side, joint);
			} else if (c.instr.front() == "Reset") {
				skip = true;
				if (c.instr.size() > 1) {
					resetCondition(c.instr.at(1));
				} else {
					std::cerr << "Missing parameter for command " << c.instr.front() << std::endl;
				}
				stepScriptCounter(side, joint);
			} else if (c.instr.front() == "Nop" || c.instr.front() == "NOP") {
				// Nothing happens ...
				// skip = true;
				// stepScriptCounter(side, joint);
			} else if (c.instr.front() == "Balance") {
				skip = true;
				stepScriptCounter(side, joint);
				if (c.param.front() == 1) {
					balanceBody = true;
				} else {
					balanceBody = false;
				}
			} else if (c.instr.front() == "Push") {
				addForce(c.param.at(0),c.param.at(1),c.param.at(2));
			} else if (c.instr.front() == "Print") {
				if (c.instr.size() < 2 && c.param.size() < 1) {
					std::cout << "Frame: " << frame;
					std::cout << "; Side: " << side << "; Joint: " << joint;
					std::cout << "; Index: " << index << "; PC: " << pc << std::endl;
				} else {
					for (std::vector<std::string>::iterator it=c.instr.begin(); it!=c.instr.end(); ++it) {
						std::cout << *it << "; ";
					}
					for (std::vector<double>::iterator it=c.param.begin(); it!=c.param.end(); ++it) {
						std::cout << *it << "; ";
					}
					std::cout << std::endl;
				}
				skip = true;
				stepScriptCounter(side, joint);
			}
		}
	}
	ret.push_back(f);
	ret.push_back(v);
	return ret;
}

static dJointID getJointID(int side, int joint) {
	LegVars lv;
	if (side == 0) {
		lv = leftLeg;
	} else {
		lv = rightLeg;
	}
	switch (joint) {
		case 0 : return lv.ankleHinge;
		case 1 : return lv.kneeHinge;
		case 2 : return lv.hipHinge;
	}
	std::cerr << "Joint ID not found!" << std::endl;
	return lv.hipHinge;
}

static void setHipHinges(dReal vel) {
	if (vel > 300) {
		vel = 300;
	}
	if (vel < -300) {
		vel = -300;
	}
	dJointSetHingeParam(leftLeg.hipHinge,  dParamFMax, Configuration::getInstance(XML_FILE)->getInt("AutoMaxiumumForce")/100.0); // 10.0
	dJointSetHingeParam(leftLeg.hipHinge,  dParamVel,  vel);
	dJointSetHingeParam(rightLeg.hipHinge, dParamFMax, Configuration::getInstance(XML_FILE)->getInt("AutoMaxiumumForce")/100.0); // 10.0
	dJointSetHingeParam(rightLeg.hipHinge, dParamVel,  vel);
}

static void resetHipHinges() {
	dJointSetHingeParam(leftLeg.hipHinge,  dParamFMax, FRICTION_FORCE);
	dJointSetHingeParam(leftLeg.hipHinge,  dParamVel,  0);
	dJointSetHingeParam(rightLeg.hipHinge, dParamFMax, FRICTION_FORCE);
	dJointSetHingeParam(rightLeg.hipHinge, dParamVel,  0);
}

static void waitStartUp() {
	if (frame > STARTUP_FRAMES) {
		std::cout << "Ready & Go" << std::endl;
		ready = true;
		frame = 0;
		resetHipHinges();
	}
}

static void moveJoint(std::vector<double> currFV, dJointID jointID) {
	// std::vector<int> currFV = currentFmaxVel(name);
	if (currFV.size() == 2) {
		// std::cout << "Joint 1: " << jointID << std::endl;
		// std::cout << "Joint 2: " << rightLeg.hipHinge << std::endl;
		dReal fmax = currFV.at(0);
		dReal vel  = currFV.at(1);
		// std::cout << name << ": " << fmax << "; " << vel << std::endl;
		dJointSetHingeParam(jointID, dParamFMax, fmax);
		dJointSetHingeParam(jointID, dParamVel,  vel);
	}
}

static void moveJoints() {
	for (int side=0; side<2; ++side) {
		for (int joint=0; joint<3; ++joint) {
			moveJoint(getFmaxVel(side,joint),getJointID(side,joint));
		}
	}
}

/*
static void balance(dBodyID i1, dBodyID i2, dBodyID o) {
	const dReal * hl = dBodyGetPosition( leftLeg.hip);
	const dReal * al = dBodyGetPosition( leftLeg.ankle);
	dReal diffl = al[1]-0.03-hl[1];
	dReal factor = 0.2; // Configuration::getInstance(XML_FILE)->getInt("AutoVelocityFactor");
	dReal fl = diffl * diffl * diffl * factor;
	dReal fr = diffr * diffr * diffr * factor;
	if (fl > 30) {
		fl = 30;
	}
	if (fl < -30) {
		fl = -30;
	}
	if (fr > 30) {
		fr = 30;
	}
	if (fr < -30) {
		fr = -30;
	}
	dBodyAddForce(  leftLeg.hip, 0, fl, 0);
	dBodyAddForce( rightLeg.hip, 0, fr, 0);
}
*/

static void pullUp() {
	// std::cout << frame << std::endl;
	// dJointSetHingeParam( leftLeg.hipHinge,   dParamFMax, 0);
	// dJointSetHingeParam( leftLeg.hipHinge,   dParamVel,  0);
	// dJointSetHingeParam(rightLeg.hipHinge,   dParamFMax, 0);
	// dJointSetHingeParam(rightLeg.hipHinge,   dParamVel,  0);
	if (frame < 100) {
		const dReal * hl = dBodyGetPosition(leftLeg.hip);
		dReal diffl = hl[1]+0.12727;
		dReal fl;
		fl  = diffl * -0.03;
		fl += diffl * diffl * diffl * -0.2;
		// std::cout << "hip left: " << hl[1] << "; diff: " << diffl << "; " << std::endl;
		if (fl > 100) {
			fl = 100;
		}
		if (fl < -100) {
			fl = -100;
		}
		dBodyAddForce(  leftLeg.hip, 0, fl, 0);
		dBodyAddForce( rightLeg.hip, 0, fl, 0);
		// std::cout << "A" << std::endl;
		oldDiff = diffl;
	} else if (frame < 240) {
		const dReal * hl = dBodyGetPosition(leftLeg.hip);
		dReal diffl = hl[1]+0.12727;
		dReal speed = diffl - oldDiff;
		dReal fl;
		fl = diffl * diffl * diffl * -0.07 - speed * (1/diffl);
		// std::cout << "hip left: " << hl[1] << "; diff: " << diffl << "; " << frame << std::endl;
		if (fl > 100) {
			fl = 100;
		}
		if (fl < -100) {
			fl = -100;
		}
		dBodyAddForce(  leftLeg.hip, 0, fl, 0);
		dBodyAddForce( rightLeg.hip, 0, fl, 0);
		// std::cout << "B" << std::endl;
		oldDiff = diffl;
	} 
	if (frame < 244) {
		// Balancing
		dGeomID bft = dGeomTransformGetGeom(bottleFront);
		dGeomID bbt = dGeomTransformGetGeom(bottleBack);	
		const dReal * bf = dGeomGetPosition(bft);
		const dReal * bb = dGeomGetPosition(bbt);
		dVector3 resF, resB;
		dBodyGetRelPointPos (body, bf[0], bf[1], bf[2], resF);
		dBodyGetRelPointPos (body, bb[0], bb[1], bb[2], resB);
		// std::cout << "Front: " << resF[2] << "; Back: " << resB[2] << "; Diff: " << resB[2]-resF[2] << std::endl;		
		dReal diffb = resB[2]-resF[2]-0.0828; // 0.23118;
		dReal fb = diffb * diffb * diffb * -1000;
		if (fb > 50) {
			fb = 50;
		}
		if (fb < -50) {
			fb = -50;
		}
		setHipHinges(fb);
	}
	if (frame == 245) {
		dJointSetHingeParam( leftLeg.hipHinge,   dParamFMax, FRICTION_FORCE*10);
		dJointSetHingeParam( leftLeg.hipHinge,   dParamVel,  0);
		dJointSetHingeParam(rightLeg.hipHinge,   dParamFMax, FRICTION_FORCE*10);
		dJointSetHingeParam(rightLeg.hipHinge,   dParamVel,  0);
	}
	if (frame == 249) {
		dJointSetHingeParam( leftLeg.hipHinge,   dParamFMax, FRICTION_FORCE);
		dJointSetHingeParam( leftLeg.hipHinge,   dParamVel,  0);
		dJointSetHingeParam(rightLeg.hipHinge,   dParamFMax, FRICTION_FORCE);
		dJointSetHingeParam(rightLeg.hipHinge,   dParamVel,  0);
	}
}

static void setMotors() {
	if (!ready) {
		waitStartUp();
		pullUp();
	}
	if (ready) {
		moveJoints();
		stepScriptCounters();
	}
	++frame;
	if (frame < 0) {
		frame = 1;
	}
}

static void controlLoop() {
	/*
	if (balanceBody) {
		const dReal * fBottP = dBodyGetPosition(bottleFront);
		const dReal * bBottP = dBodyGetPosition(bottleBack);
		dReal diff = 10000*(bBottP[2]-fBottP[2]);
		int intDiff = diff;
		diff = intDiff / 10000.0;
		dReal absDiff = diff;
		if (absDiff < 0) {
			absDiff *= -1;
		}
		if (absDiff > 0.01 & control>1) {
			// switch on upper body correction
			control = 1;
			correctionVelocity = diff*Configuration::getInstance(XML_FILE)->getInt("AutoVelocityFactor"); // 50.0
			setHipHinges(correctionVelocity);
			// std::cout << "Correction: " << correctionVelocity << std::endl;
		} else if (absDiff < 0.001 & control==1) {
			// switch off upper body correction
			control = 2;
			setHipHinges(0);
			// std::cout << "Blocking Hips: " << absDiff << std::endl;
		} else if (control==1) {
			// catch if we're doing correction in the wrong direction ==> reset correction
			if (correctionVelocity < 0 && diff > 0 || correctionVelocity > 0 && diff < 0) {
				correctionVelocity = diff*Configuration::getInstance(XML_FILE)->getInt("AutoVelocityFactor");
				setHipHinges(correctionVelocity);
			}
			if (absDiff > 0.05) {
				correctionVelocity = diff*2*Configuration::getInstance(XML_FILE)->getInt("AutoVelocityFactor");
				setHipHinges(correctionVelocity);
				// std::cout << "Super *** Correction: " << correctionVelocity << std::endl;
			}
		}
	}
	*/
}

static bool isMotorCapsule(dBodyID body, LegVars leg) {
	if (body == leg.ankle || body == leg.knee || body == leg.hip) {
		return true;
	}
	return false;
}

static bool isMotorCapsule(dBodyID body) {
	return (isMotorCapsule(body, rightLeg) || isMotorCapsule(body, leftLeg));
}

static dJointID getJoint(dBodyID body, LegVars leg) {
	if (body == leg.ankle) {
		return leg.ankleHinge;
	}
	if (body == leg.knee) {
		return leg.kneeHinge;
	}
	if (body == leg.hip) {
		return leg.hipHinge;
	}
	return NULL;
}

static dJointID getJoint(dBodyID body) {
	if (isMotorCapsule(body, leftLeg)) {
		return getJoint(body, leftLeg);
	}
	if (isMotorCapsule(body, rightLeg)) {
		return getJoint(body, rightLeg);
	}
	return NULL;
}

static void rotateCamera() {
	if (Configuration::getInstance(XML_FILE)->getInt("followWithCamera") != 0) {
		const dReal * pos = dBodyGetPosition(body);
		float xyz[3];
		float hpr[3];
		int z = Configuration::getInstance(XML_FILE)->getInt("CameraZ");
		float radius = Configuration::getInstance(XML_FILE)->getInt("CameraRadius");
		xyz[0] = pos[0]+radius;
		xyz[1] = pos[1]-radius;
		xyz[2] = z/100;
		hpr[0] = 140;
		hpr[1] = 0;
		hpr[2] = 0;
		dsSetViewpoint(xyz, hpr);
	} else if (Configuration::getInstance(XML_FILE)->getInt("RotationSpeed") != 0) {
		/**
		 * @brief Sets the viewpoint
		 * @ingroup drawstuff
		 * @param xyz camera position.
		 * @param hpr contains heading, pitch and roll numbers in degrees. heading=0
		 * points along the x axis, pitch=0 is looking towards the horizon, and
		 * roll 0 is "unrotated".
		 */
		float xyz[3];
		float hpr[3];
		int z = Configuration::getInstance(XML_FILE)->getInt("CameraZ");
		float radius = Configuration::getInstance(XML_FILE)->getInt("CameraRadius");
		float x = sin(viewAngle * M_PI / 180.0) * radius;
		float y = cos(viewAngle * M_PI / 180.0) * radius;
		xyz[0] = x;
		xyz[1] = y;
		xyz[2] = z/100;
		hpr[0] = 270-viewAngle;
		hpr[1] = 0;
		hpr[2] = 0;
		dsSetViewpoint(xyz, hpr);	
		viewAngle += 0.001 * Configuration::getInstance(XML_FILE)->getInt("RotationSpeed");
		if (viewAngle > 360) {
			viewAngle = 0;
		}
	}
}

// draw a geom
void drawGeom(dGeomID g, const dReal *pos, const dReal *R, bool show_aabb=false)
{
  if (!g) return;
  if (!pos) pos = dGeomGetPosition (g);
  if (!R) R = dGeomGetRotation (g);

  int type = dGeomGetClass (g);
  if (type == dBoxClass) {
    dVector3 sides;
    dGeomBoxGetLengths (g,sides);
    dsDrawBox (pos,R,sides);
  }
  else if (type == dSphereClass) {
    dsDrawSphere (pos,R,dGeomSphereGetRadius(g));
  }
  else if (type == dCapsuleClass) {
    dReal radius,length;
    dGeomCapsuleGetParams (g,&radius,&length);
    dsDrawCapsule (pos,R,length,radius);
  }
  else if (type == dGeomTransformClass) {
    dGeomID g2 = dGeomTransformGetGeom (g);
    const dReal *pos2 = dGeomGetPosition (g2);
    const dReal *R2 = dGeomGetRotation (g2);
    dVector3 actual_pos;
    dMatrix3 actual_R;
    dMULTIPLY0_331 (actual_pos,R,pos2);
    actual_pos[0] += pos[0];
    actual_pos[1] += pos[1];
    actual_pos[2] += pos[2];
    dMULTIPLY0_333 (actual_R,R,R2);
    drawGeom(g2,actual_pos,actual_R,0);
  }
  else if (type == dCylinderClass) { // not fully implemented in ODE ==> use with care!
    dReal radius,length;
    dGeomCylinderGetParams(g,&radius,&length);
    dsDrawCylinder (pos,R,length,radius);
  }
  if (show_aabb) {
    // draw the bounding box for this geom
    dReal aabb[6];
    dGeomGetAABB (g,aabb);
    dVector3 bbpos;
    for (int i=0; i<3; i++) bbpos[i] = 0.5*(aabb[i*2] + aabb[i*2+1]);
    dVector3 bbsides;
    for (int j=0; j<3; j++) bbsides[j] = aabb[j*2+1] - aabb[j*2];
    dMatrix3 RI;
    dRSetIdentity (RI);
    dsSetColorAlpha (1,0,0,0.5);
    dsDrawBox (bbpos,RI,bbsides);
  }
}

static void drawAll() {
    const dReal *pos;
    const dReal *R;
	for (BodyGeos::iterator it1=bodyGeos.begin(); it1!=bodyGeos.end(); ++it1) {
		if (!dBodyIsEnabled(it1->body)) {
			dsSetColor(1,0.8,0);
		} else {
			dsSetColor(1,1,0);
		}
		if (isMotorCapsule(it1->body)) {
			dReal vel = dJointGetHingeParam(getJoint(it1->body), dParamVel);
			if (vel > 0.0001) {
				dsSetColor(0,1,0);
			} else if (vel < -0.0001) {
				dsSetColor(1,0,0);
			} else {
				dReal fmax = dJointGetHingeParam(getJoint(it1->body), dParamFMax);
				if (fmax > FRICTION_FORCE+0.00001) { 
					dsSetColor(0,0,0);
				}
			}
		}
		for (std::vector<dGeomID>::iterator it2=it1->geos.begin(); it2!=it1->geos.end(); ++it2) {
			pos = dGeomGetPosition(*it2);
			R = dGeomGetRotation(*it2);
			drawGeom(*it2,pos,R);
		}
	}
}

// simulation loop
static void simLoop(int pause) {
	dsSetTexture (DS_WOOD);
    // find collisions and add contact joints
    dSpaceCollide(space,0,&nearCallback);
    // step the simulation
    dWorldStep(world,0.01);
    // dWorldQuickStep(world,0.01);  
    // remove all contact joints
    dJointGroupEmpty(contactgroup);
    // redraw sphere at new location
    if (SHOW_STARTUP || ready) {
    	drawAll();
		rotateCamera();
	}
	setMotors();
	controlLoop();
	testPreDefinedConditions(); // TODO remove this debugging line
}

static dBodyID addSphere(dReal x,dReal y,dReal height,dReal radius) {
	dMass m;
	dBodyID body;
	dGeomID geo;
	body = dBodyCreate(world);
	geo = dCreateSphere(space,radius);
	
	BodyGeo bg;
	bg.body = body;
	bg.geos.clear();
	bg.geos.push_back(geo);
	bodyGeos.push_back(bg);

	dMassSetSphere(&m,DENSITY,radius);
	dBodySetMass(body,&m);
	dGeomSetBody(geo,body);

	// set initial position
	dBodySetPosition(body,x,y,height); //+radius
	return body;
}

static dBodyID addSphere(dReal x,dReal y,dReal height) {
	dReal radius = dRandReal()*0.5+0.1;
	return addSphere(x,y,height,radius);
}

static dBodyID addSphere(dReal height) {
	return addSphere(POS_X,POS_Y,height);
}

static dBodyID addSphere() {
	return addSphere(POS_X,POS_Y,POS_Z);
}

static dBodyID addBox(dReal x,dReal y,dReal z,dReal a,dReal b,dReal c,dReal density) {
	dMass m;
	dBodyID body;
	dGeomID geo;
	body = dBodyCreate(world);
	geo = dCreateBox(space,a,b,c);

	BodyGeo bg;
	bg.body = body;
	bg.geos.clear();
	bg.geos.push_back(geo);
	bodyGeos.push_back(bg);

	dMassSetBox (&m,density,a,b,c);
	dBodySetMass(body,&m);
	dGeomSetBody(geo,body);
	dBodySetPosition(body,x,y,z);
	return body;
}

static dBodyID addBox(dReal x,dReal y,dReal z,dReal a,dReal b,dReal c) {
	return addBox(x,y,z,a,b,c,DENSITY);
}

static dBodyID addCylinder(dReal x, dReal y, dReal z, dReal radius, dReal width) {
	dMass m;
	dBodyID body;
	dGeomID geo;
	body = dBodyCreate(world);
	geo = dCreateCylinder(space,radius,width);
	BodyGeo bg;
	bg.body = body;
	bg.geos.clear();
	bg.geos.push_back(geo);
	bodyGeos.push_back(bg);

	dMassSetCylinder(&m,DENSITY,3,radius,width);
	dBodySetMass(body,&m);
	dGeomSetBody(geo,body);
	dBodySetPosition(body,x,y,z);
	return body;
}

static dBodyID addCapsule(dReal x, dReal y, dReal z, dReal radius, dReal width, dReal density) {
	dMass m;
	dBodyID body;
	dGeomID geo;
	body = dBodyCreate(world);
	geo = dCreateCapsule(space,radius,width);
	BodyGeo bg;
	bg.body = body;
	bg.geos.clear();
	bg.geos.push_back(geo);
	bodyGeos.push_back(bg);

	dMassSetCapsule(&m,density,3,radius,width);
	dBodySetMass(body,&m);
	dGeomSetBody(geo,body);
	dBodySetPosition(body,x,y,z);

	return body;
}

static dBodyID addCapsule(dReal x, dReal y, dReal z, dReal radius, dReal width) {
	return addCapsule(x, y, z, radius, width, DENSITY);
}

static dBodyID addCapsule(dReal x, dReal y, dReal z, dReal radius, dReal width, dReal ax, dReal ay, dReal az, dReal angle, dReal density) {
	dMass m;
	dBodyID body;
	dGeomID geo;
	body = dBodyCreate(world);
	geo = dCreateCapsule(space,radius,width);

	dMassSetCapsule(&m,density,3,radius,width);
	dBodySetMass(body,&m);
	dGeomSetBody(geo,body);
	dBodySetPosition(body,x,y,z);

	dMatrix3 Rtx;
	dRFromAxisAndAngle (Rtx,ax,ay,az,angle);
	dGeomSetRotation (geo,Rtx);
	dMassRotate (&m,Rtx);	

	BodyGeo bg;
	bg.body = body;
	bg.geos.clear();
	bg.geos.push_back(geo);
	bodyGeos.push_back(bg);

	return body;
}

static dBodyID addCapsule(dReal x, dReal y, dReal z, dReal radius, dReal width, dReal ax, dReal ay, dReal az, dReal angle) {
	return addCapsule(x, y, z, radius, width, ax, ay, az, angle, DENSITY);
}

static dBodyID addCapsuleAt(dReal x,dReal y,dReal z) {
    dReal a1, a2, a3, a4;
    dMatrix3 hrot;
    a1 = M_PI/3.0*2.0;
    a2 = M_PI/3.0*2.0;
    a3 = M_PI/3.0*2.0;
    a4 = M_PI/3.0*2.0;
 	dBodyID c = addCapsule(x,y,z,RADIUS-0.001,WIDTH-RADIUS); // ,0,3.1415/2.0.0,3.1415/2.0); //3.1415/2.0
	dRFromAxisAndAngle(hrot, a1, a2, a3, a4);
	dBodySetRotation(c, hrot);
	// TODO dMassRotate(&c->getMass,hrot);
	return c;
}

static dBodyID addCylinderAt(dReal x,dReal y,dReal z) {
    dReal a1, a2, a3, a4;
    dMatrix3 hrot;
    a1 = M_PI/3.0*2.0;
    a2 = M_PI/3.0*2.0;
    a3 = M_PI/3.0*2.0;
    a4 = M_PI/3.0*2.0;
 	dBodyID c = addCylinder(x,y,z,0.1,0.4); // ,0,3.1415/2.0,3.1415/2.0); //3.1415/2.0
	dRFromAxisAndAngle(hrot, a1, a2, a3, a4);
	dBodySetRotation(c, hrot);
	// TODO dMassRotate(&c->getMass,hrot);
	return c;
}

static LegVars buildFoot(dReal pos) {
	LegVars ret;
	std::vector<dGeomID> lowFootParts;
	lowFootParts.clear();
	
	dBodyID body = dBodyCreate(world);
	std::vector<dGeomID> geos;
	geos.clear();
	unsigned gpb = 11;
	for (unsigned k=0; k<gpb; k++) {
		dGeomID gid = dCreateGeomTransform(space);
		dGeomTransformSetCleanup(gid,1);		
		geos.push_back(gid);		
	}

	dBodySetPosition(body,pos,-FOOT/6.0,0.08);
	int i;
	dBodySetData (body,(void*)(size_t)i);

	std::vector<dGeomID> g2;			// encapsulated geometries
	g2.clear();

	std::vector<std::vector<dReal> > dpos;			// delta-positions for encapsulated geometries
	dpos.clear();

	// start accumulating masses for the encapsulated geometries
	dMass m2;
	dMass m;
	dMassSetZero(&m);

	dReal radius = DEPTH/4.0;
	dReal toe = FOOT/7*2-2*radius;
	dReal length = FOOT-toe-4*radius;
	dReal left = WIDTH/2-radius;
	dReal right = -left;
	dReal center = 0;

	std::vector<dReal> coord;
	coord.clear();
	// left capsule of foot
	coord.push_back(0);
	coord.push_back(left);
	coord.push_back(0);
	dpos.push_back(coord);
	coord.clear();
	// center box of foot
	coord.push_back(0);
	coord.push_back(center);
	coord.push_back(0);
	dpos.push_back(coord);
	coord.clear();
	// right capsule of foot
	coord.push_back(0);
	coord.push_back(right);
	coord.push_back(0);
	dpos.push_back(coord);
	coord.clear();
	// left toe
	coord.push_back(0.015);
	coord.push_back(left);
	coord.push_back(length/2+toe/2+2*radius);
	dpos.push_back(coord);
	coord.clear();
	// center toe
	coord.push_back(0.015);
	coord.push_back(center);
	coord.push_back(length/2+toe/2+2*radius);
	dpos.push_back(coord);
	coord.clear();
	// right toe
	coord.push_back(0.015);
	coord.push_back(right);
	coord.push_back(length/2+toe/2+2*radius);
	dpos.push_back(coord);
	coord.clear();
	// foot plate
	coord.push_back((DEPTH/3.0*2.0-radius)/2.0);
	coord.push_back(center);
	coord.push_back(length/10.0);
	dpos.push_back(coord);
	coord.clear();
	// top plate below ankle
	coord.push_back(DEPTH/2.0);
	coord.push_back(center);
	coord.push_back(-length/20.0);
	dpos.push_back(coord);
	coord.clear();
	// capsule at the back of the foot
	coord.push_back(0);
	coord.push_back(center);
	coord.push_back(-length/2-0.001);
	dpos.push_back(coord);
	coord.clear();
	// capsule next to the toes
	coord.push_back(0);
	coord.push_back(center);
	coord.push_back(length/2+0.001);
	dpos.push_back(coord);
	coord.clear();
	// toe plate
	coord.push_back(DEPTH/6);
	coord.push_back(center);
	coord.push_back(length/2.2);
	dpos.push_back(coord);
	coord.clear();

	unsigned elem = 0;

	// left capsule of foot
	g2.push_back(dCreateCapsule(0,radius,length));
	dMassSetCapsule (&m2,DENSITY,3,radius,length);
	dGeomTransformSetGeom (geos.at(elem),g2.at(elem)); 
	dGeomSetPosition(g2.at(elem),dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassTranslate(&m2,dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassAdd (&m,&m2);	
	lowFootParts.push_back(geos.at(elem));
	++elem;

	// center box of foot
	g2.push_back(dCreateBox(0,0.1,0.2,length));
	dMassSetBox(&m2,DENSITY,0.1,0.2,length);
	dGeomTransformSetGeom (geos.at(elem),g2.at(elem)); 
	dGeomSetPosition(g2.at(elem),dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassTranslate(&m2,dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassAdd (&m,&m2);	
	lowFootParts.push_back(geos.at(elem));
	++elem;

	// right capsule of foot
	g2.push_back(dCreateCapsule(0,radius,length));
	dMassSetCapsule (&m2,DENSITY,3,radius,length);
	dGeomTransformSetGeom (geos.at(elem),g2.at(elem)); 
	dGeomSetPosition(g2.at(elem),dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassTranslate(&m2,dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassAdd (&m,&m2);	
	lowFootParts.push_back(geos.at(elem));
	++elem;

    dReal a1t = 0; // M_PI/3.0*2.0;
    dReal a2t = M_PI/13.0;
    dReal a3t = M_PI/13.0;
    dReal a4t = M_PI/13.0;
    dMatrix3 hrott;
	dRFromAxisAndAngle(hrott, a1t, a2t, a3t, a4t);

	// left toe
	g2.push_back(dCreateCapsule(0,radius,toe));
	dMassSetCapsule(&m2,DENSITY,3,radius,toe);
	dGeomTransformSetGeom(geos.at(elem),g2.at(elem)); 
	dGeomSetRotation(g2.at(elem), hrott);
	dGeomSetPosition(g2.at(elem),dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassTranslate(&m2,dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassAdd(&m,&m2);	
	lowFootParts.push_back(geos.at(elem));
	++elem;

	// center toe
	g2.push_back(dCreateCapsule(0,radius,toe));
	dMassSetCapsule(&m2,DENSITY,3,radius,toe);
	dGeomTransformSetGeom(geos.at(elem),g2.at(elem)); 
	dGeomSetRotation(g2.at(elem), hrott);
	dGeomSetPosition(g2.at(elem),dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassTranslate(&m2,dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassAdd(&m,&m2);	
	lowFootParts.push_back(geos.at(elem));
	++elem;

	// right toe
	g2.push_back(dCreateCapsule(0,radius,toe));
	dMassSetCapsule(&m2,DENSITY,3,radius,toe);
	dGeomTransformSetGeom(geos.at(elem),g2.at(elem)); 
	dGeomSetRotation(g2.at(elem), hrott);
	dGeomSetPosition(g2.at(elem),dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassTranslate(&m2,dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassAdd(&m,&m2);	
	lowFootParts.push_back(geos.at(elem));
	++elem;

	// foot plate
	g2.push_back(dCreateBox(0,DEPTH/3.0*2.0-radius,WIDTH,length+toe-radius/3.0*4.0));
	dMassSetBox(&m2,DENSITY,DEPTH/3.0*2.0-radius,WIDTH,length+2.0*radius+toe-radius/3.0*4.0);
	dGeomTransformSetGeom (geos.at(elem),g2.at(elem)); 
	dGeomSetPosition(g2.at(elem),dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassTranslate(&m2,dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassAdd (&m,&m2);	
	++elem;

	// top plate below ankle
	g2.push_back(dCreateBox(0,DEPTH/2.0,WIDTH/4.0*3.0,FOOT/2.4));
	dMassSetBox(&m2,DENSITY,DEPTH/2.0,WIDTH/4.0*3.0,FOOT/2.4);
	dGeomTransformSetGeom (geos.at(elem),g2.at(elem)); 
	dGeomSetPosition(g2.at(elem),dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassTranslate(&m2,dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassAdd (&m,&m2);	
	++elem;

    dReal a1e = M_PI/3.0*2.0;
    dReal a2e = -M_PI/3.0*2.0;
    dReal a3e = M_PI/3.0*2.0;
    dReal a4e = M_PI/3.0*2.0;
    dMatrix3 hrote;
	dRFromAxisAndAngle(hrote, a1e, a2e, a3e, a4e);
	
	// capsule at the back of the foot
	g2.push_back(dCreateCapsule(0,radius,WIDTH-2.0*radius));
	dMassSetCapsule(&m2,DENSITY,3,radius,WIDTH-2.0*radius);
	dGeomTransformSetGeom(geos.at(elem),g2.at(elem)); 
	dGeomSetRotation(g2.at(elem), hrote);
	dGeomSetPosition(g2.at(elem),dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassTranslate(&m2,dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassAdd(&m,&m2);	
	lowFootParts.push_back(geos.at(elem));
	++elem;

	// capsule next to the toes
	g2.push_back(dCreateCapsule(0,radius,WIDTH-2.0*radius));
	dMassSetCapsule(&m2,DENSITY,3,radius,WIDTH-2.0*radius);
	dGeomTransformSetGeom(geos.at(elem),g2.at(elem)); 
	dGeomSetRotation(g2.at(elem), hrote);
	dGeomSetPosition(g2.at(elem),dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassTranslate(&m2,dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassAdd(&m,&m2);	
	lowFootParts.push_back(geos.at(elem));
	++elem;

	// toe plate
	g2.push_back(dCreateBox(0,DEPTH/4.0,WIDTH*9.0/10.0,FOOT/5.0*3.0));
	dMassSetBox(&m2,DENSITY,DEPTH/4.0,WIDTH*9.0/10.0,FOOT/5.0*3.0);
	dGeomTransformSetGeom(geos.at(elem),g2.at(elem));
	dGeomSetPosition(g2.at(elem),dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassTranslate(&m2,dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassAdd(&m,&m2);
	++elem;

	// move all encapsulated objects so that the center of mass is (0,0,0)
	for (unsigned k=0; k<gpb; k++) {
		dGeomSetPosition (g2[k],
				dpos.at(k).at(0)-m.c[0],
				dpos.at(k).at(1)-m.c[1],
				dpos.at(k).at(2)-m.c[2]);
	}
	dMassTranslate (&m,-m.c[0],-m.c[1],-m.c[2]);
	
	for (std::vector<dGeomID>::iterator it=geos.begin(); it!=geos.end(); ++it) { // k=0; k < gpb; k++) {
		if (*it) dGeomSetBody (*it,body);
	}
	dBodySetMass (body,&m);

    dReal a1b = M_PI/3.0*2.0;
    dReal a2b = -M_PI/3.0*2.0;
    dReal a3b = M_PI/3.0*2.0;
    dReal a4b = M_PI/3.0*2.0;
    dMatrix3 hrotb;
	dRFromAxisAndAngle(hrotb, a1b, a2b, a3b, a4b);
	dBodySetRotation(body, hrotb);

	BodyGeo bg;
	bg.body = body;
	bg.geos = geos;
	bodyGeos.push_back(bg);

	ret.lowFootParts = lowFootParts;
	ret.foot = body;
	return ret;
}

static dJointID hinge(dBodyID a, dBodyID b, dReal x, dReal y, dReal z) {
	dJointID hingeID = dJointCreateHinge (world,0);
	dJointAttach(hingeID,a,b);
	dJointSetHingeAnchor(hingeID,x,y,z);
	dJointSetHingeAxis(hingeID,1,0,0);
	dJointEnable(hingeID);
	joints.push_back(hingeID);
	return hingeID;
}

static LegVars buildLeg(dReal pos, dReal startHeight) {
	LegVars lv;
	dReal height = startHeight;
	dReal hingeHeight = startHeight;
	height += DEPTH/2.0;
	hingeHeight = height;
	LegVars tmp = buildFoot(pos);
	lv.foot = tmp.foot;
	lv.lowFootParts = tmp.lowFootParts;
	hingeHeight = hingeHeight + DEPTH;
	height = hingeHeight + DEPTH/2.0 + SHIN/2.0;
	lv.shin = addBox(pos,0,height,WIDTH,DEPTH,SHIN); // shin
	lv.ankleHinge = hinge(lv.foot,lv.shin,pos,0,hingeHeight); // ankle hinge
	dJointSetHingeParam(lv.ankleHinge, dParamLoStop, -M_PI/4.0); // ankle angle low stop
	dJointSetHingeParam(lv.ankleHinge, dParamHiStop,  M_PI/3.0); // ankle angle high stop
	lv.ankle = addCapsuleAt(pos,0,hingeHeight); // ankle
	hinge(lv.foot,lv.ankle,pos,0,hingeHeight); // ankle hinge just connecting the capsule
	height = height + DEPTH/2.0 + SHIN/2.0;
	hingeHeight = height;
	lv.leg = addBox(pos,DEPTH/2.0+LEG/2.0,height,WIDTH,LEG,DEPTH); // leg
	lv.kneeHinge = hinge(lv.shin,lv.leg,pos,0,hingeHeight); // knee hinge
	dJointSetHingeParam(lv.kneeHinge, dParamLoStop, -M_PI/2.0); // knee angle low stop
	dJointSetHingeParam(lv.kneeHinge, dParamHiStop,  0); // knee angle high stop
	lv.knee = addCapsuleAt(pos,0,hingeHeight); // knee
	hinge(lv.shin,lv.knee,pos,0,hingeHeight); // knee hinge just connecting the capsule
	lv.hip  = addCapsuleAt( pos, DEPTH+LEG, hingeHeight);
	lv.hipHinge = hinge(lv.leg, lv.hip, (DISTANCE+WIDTH)/2.0, DEPTH+LEG, hingeHeight); // hip hinge just connecting the capsule
	lv.finalHeight = height;
	return lv;
}

static LegVars buildLeg(dReal pos) {
	buildLeg(pos, 0);
}

static dReal buildBody() {
	dBodyID compBody = dBodyCreate(world);
	std::vector<dGeomID> geos;
	geos.clear();

	dReal radius = DISTANCE/2.0-0.02;
	unsigned gpb = 4;
	
	for (unsigned k=0; k<gpb; k++) {
		dGeomID gid = dCreateGeomTransform(space);
		dGeomTransformSetCleanup(gid,1);		
		geos.push_back(gid);		
	}

	const dReal * posis = dBodyGetPosition(leftLeg.hip);
	dBodySetPosition(compBody,0,posis[1],posis[2]-0.17);
	int i;
	dBodySetData (compBody,(void*)(size_t)i);

	std::vector<dGeomID> g2;				// encapsulated geometries
	g2.clear();
	std::vector<std::vector<dReal> > dpos;	// delta-positions for encapsulated geometries
	dpos.clear();

	std::vector<dReal> coord;
	coord.clear();
	// connection bar between legs
	coord.push_back(0);
	coord.push_back(0);
	coord.push_back(0);
	dpos.push_back(coord);
	coord.clear();
	// front bottle
	coord.push_back(0);
	coord.push_back(radius+DEPTH/2.0);
	coord.push_back(BODY/3.0);
	dpos.push_back(coord);
	coord.clear();
	// back bottle
	coord.push_back(0);
	coord.push_back(-radius-DEPTH/2.0);
	coord.push_back(BODY/3.0);
	dpos.push_back(coord);
	coord.clear();
	// battery
	coord.push_back(0);
	coord.push_back(0);
	coord.push_back(-PUMP/2.0-DEPTH/2.0);
	dpos.push_back(coord);
	coord.clear();
	
	// start accumulating masses for the encapsulated geometries
	dMass m2;
	dMass m;
	dMassSetZero(&m);

	unsigned elem = 0;
	// connection bar between legs
	g2.push_back(dCreateBox(0, DISTANCE+2.0*WIDTH,DEPTH,DEPTH));
	dMassSetBox(&m2,DENSITY,   DISTANCE+2.0*WIDTH,DEPTH,DEPTH);
	dGeomTransformSetGeom(geos.at(elem),g2.at(elem)); 
	dGeomSetPosition(g2.at(elem),dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassTranslate(&m2,dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassAdd (&m,&m2);	

	// front bottle
	++elem;
	bottleFront = dCreateCapsule(0,radius,BODY);
	g2.push_back(bottleFront);
	dMassSetCapsule (&m2,DENSITY*0.01,3, radius,BODY);
	dGeomTransformSetGeom(geos.at(elem),g2.at(elem)); 
	dGeomSetPosition(g2.at(elem),dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassTranslate(&m2,dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	bottleFront = geos.at(elem);
	dMassAdd (&m,&m2);	

	// back bottle
	++elem;
	bottleBack = dCreateCapsule(0,radius,BODY);
	g2.push_back(bottleBack);
	dMassSetCapsule(&m2,DENSITY*0.01,3, radius,BODY);
	dGeomTransformSetGeom(geos.at(elem),g2.at(elem)); 
	dGeomSetPosition(g2.at(elem),dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassTranslate(&m2,dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	bottleBack = geos.at(elem);
	dMassAdd(&m,&m2);	

	// battery & pump
	++elem;
	pump = dCreateBox(0, DISTANCE-RADIUS-0.01, DEPTH, PUMP);
	g2.push_back(pump);
	dMassSetBox(&m2,DENSITY*5, DISTANCE-RADIUS-0.01, DEPTH, PUMP);
	dGeomTransformSetGeom(geos.at(elem),g2.at(elem)); 
	dGeomSetPosition(g2.at(elem),dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	dMassTranslate(&m2,dpos.at(elem).at(0),dpos.at(elem).at(1),dpos.at(elem).at(2));
	pump = geos.at(elem);
	dMassAdd (&m,&m2);	

	// move all encapsulated objects so that the center of mass is (0,0,0)
	for (unsigned k=0; k<dpos.size(); k++) {
		dGeomSetPosition (g2[k],
				dpos.at(k).at(0)-m.c[0],
				dpos.at(k).at(1)-m.c[1],
				dpos.at(k).at(2)-m.c[2]);
	}
	dMassTranslate (&m,-m.c[0],-m.c[1],-m.c[2]);
	
	for (std::vector<dGeomID>::iterator it=geos.begin(); it!=geos.end(); ++it) { // k=0; k < gpb; k++) {
		if (*it) dGeomSetBody (*it,compBody);
	}
	dBodySetMass (compBody,&m);

	BodyGeo bg;
	bg.body = compBody;
	bg.geos = geos;
	bodyGeos.push_back(bg);

	leftLeg.hipHinge = hinge( leftLeg.leg, compBody, (DISTANCE+WIDTH)/2.0, RADIUS*2+LEG, posis[2]);
	dJointSetHingeParam(leftLeg.hipHinge, dParamLoStop, 0);
	dJointSetHingeParam(leftLeg.hipHinge, dParamHiStop, M_PI-0.05);

	rightLeg.hipHinge = hinge(rightLeg.leg, compBody,-(DISTANCE+WIDTH)/2.0, RADIUS*2+LEG, posis[2]);
	dJointSetHingeParam(rightLeg.hipHinge, dParamLoStop, 0);
	dJointSetHingeParam(rightLeg.hipHinge, dParamHiStop, M_PI-0.05);

	body = compBody;
	return DEPTH + leftLeg.finalHeight;
}

static void resetSimulation() {
	std::cout << std::endl << "RESTART " << std::endl;
	init();
	dReal height = 0;
	leftLeg  = buildLeg( (DISTANCE+WIDTH)/2.0,height);
	rightLeg = buildLeg(-(DISTANCE+WIDTH)/2.0,height);
	height = buildBody();
	// buildBottles(height);
	// buildPump(height);
}

static void command (int cmd) {
	// std::cout << "Got the following command: " << cmd << std::endl;
	dReal x,y,z;
	x = y = z = 0;
	switch (cmd) {
		case 'r': 
		case 'R':
			resetSimulation();
			break;
		case 's': 
		case 'S':
			addSphere();
			break;
		case 'b': 
		case 'B':
			x = dRandReal();
			x -= 0.5;
			x /= 100;
			x += POS_X;
			y = dRandReal();
			y -= 0.5;
			y /= 100;
			y += POS_Y;
			z = dRandReal();
			z -= 0.5;
			z += POS_Z;
			addBox(x,y,z,dRandReal(),dRandReal(),dRandReal());
			break;
		case 'c': 
		case 'C':
			x = dRandReal();
			x -= 0.5;
			x /= 100;
			x += POS_X;
			y = dRandReal();
			y -= 0.5;
			y /= 100;
			y += POS_Y;
			z = dRandReal();
			z -= 0.5;
			z += POS_Z;
			addCylinder(x,y,z,dRandReal(),dRandReal());
			break;
		case 'a': 
		case 'A':
			x = dRandReal();
			x -= 0.5;
			x *= 20;
			x += POS_X;
			y = dRandReal();
			y -= 0.5;
			y *= 20;
			y += POS_Y;
			z = dRandReal();
			y *= 10;
			z += POS_Z;
			dReal ax; 
			ax = dRandReal()*2.0-1.0;
			dReal bx;
			bx = dRandReal()*2.0-1.0;
			dReal cx;
			cx = dRandReal()*2.0-1.0;
			dReal dx;
			dx = dRandReal()*10.0-5.0;
			// std::cout << ax << "; " << bx << "; " << cx << "; " << dx << "; " << std::endl;
			//  -0.431073; 0.602785; -0.551941; -3.66994;
			// addCapsule(x,y,z,dRandReal()/2.0,dRandReal(),ax,bx,cx,dx);
			addCapsule(x,y,z,dRandReal()/2.0,dRandReal(),ax,bx,cx,dx);
			break;
		case 'd': 
		case 'D': 
			x = dRandReal();
			y = dRandReal();
			x -= 0.5;
			y -= 0.5;
			x /= 100;
			y /= 100;
			x += POS_X;
			y += POS_Y;
			z = POS_Z;
			addSphere(x,y,z);
			break;
	}
}

int main (int argc, char **argv) {
	std::cout << "Program start" << std::endl;
	
	Configuration::getInstance(XML_FILE)->disableEmptyDataWarnings();
    
    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.stop = 0;
    fn.path_to_textures = "../../drawstuff/textures";
 	fn.command = &command;

    dInitODE();

	resetSimulation();

    // run simulation
    dsSimulationLoop(argc,argv,640,480,&fn);
    // dsSimulationLoop(argc,argv,640,480,&fn);
    // dsSimulationLoop(argc,argv,1024,768,&fn);
    // clean up
    dJointGroupDestroy(contactgroup);
    dJointGroupDestroy(jointgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    
	std::cout << std::endl << "Program end" << std::endl << std::endl;
    return 0;
}


