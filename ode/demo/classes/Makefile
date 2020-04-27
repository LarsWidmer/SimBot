CPP  = g++
RES  =

EXEC = 

SRC = $(wildcard ./*.cpp)

TINYXMLEXEC = ../tinyxml/xmltest
TINYXMLHDR = $(wildcard tinyxml/*.h)
TINYXMLCPP = $(filter-out $(TINYXMLEXEC:=.cpp),$(wildcard tinyxml/*.cpp))
HEADER = $(wildcard classes/*.h) classes/Helper-Templates.cpp2
TINYXMLOBJ = $(TINYXMLCPP:.cpp=.o)

OBJ = $(addsuffix .o,$(basename ${SRC})) $(RES)
EXECOBJ = $(EXEC:=.o)
EXECSRC = $(EXEC:=.cpp)
EXECDIRS = $(sort $(dir ${EXEC}))
DIRS = $(EXECDIRS) classes/ tinyxml/
MYBIN = $(notdir ${EXEC})
BIN = $(MYBIN) $(notdir ${TINYXMLEXEC})

LIBS =
INCS =
CXXFLAGS =
CXXSDLFLAGS = `sdl-config --cflags`
CXXLINKFLAGS = -lpthread
CXXLINKSDLFLAGS = -lpthread `sdl-config --cflags --libs`
RM = rm
GLUTFLAGS = -lglut -lGL -lGLU

default: clearscreen all
	@echo ... successfully completed!

clearscreen:
	reset

test :
	clear
	@echo ----------------------------------------
	@echo Test target: 
	@echo ----------------------------------------
	@echo Found the following header files in classes folder:
	@echo $(HEADER)
	@echo ----------------------------------------
	@echo Found the following cpp files in classes folder:
	@echo $(SRC)
	@echo ----------------------------------------
	@echo Expecting the following cpp files for the executables:
	@echo $(EXECSRC)
	@echo ----------------------------------------
	@echo ... Test done!
	@echo ----------------------------------------

hello :
	@echo forwarding request ... 

all : $(OBJ)

clean : 
	-@${RM} */*.o */*.gch */*.exe */*.*~ ${EXECOBJ} ${OBJ} ${EXEC} ${TINYXMLOBJ} ${EXECLINK} 2> /dev/null
	clear
	@echo ... done! 
	@echo All executables, object files, gch files and backup files have been removed. 

./%.o : $(HEADER) ./%.cpp 
	${CPP} -c ${CXXFLAGS} ${INCS} $(addsuffix .cpp,$(basename $@)) -o $@

$(TINYXMLEXEC) : % : %.o $(TINYXMLOBJ) 
	${CPP} $^ ${CXXFLAGS} ${INCS} -o $@	

../tinyxml/%.o : ../tinyxml/%.cpp $(TINYXMLHDR)  
	${CPP} -c ${CXXFLAGS} ${INCS} $< -o $@

xmltest : tinyxml

tinyxml : tinyxml/

tinyxml/ : $(TINYXMLEXEC)


