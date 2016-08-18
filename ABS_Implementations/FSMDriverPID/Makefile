# @file Makefile
#
# @author Guilherme N. Ramos (gnramos@unb.br)
#
# Builds the controller specified by DRIVER argument.

#########
# Setup #
#########

# Default driver
DRIVER = FSMDriverPID


# Compiler & flags
CC       = g++
CXXFLAGS = -Wall -std=c++11
EXTFLAGS = -D __DRIVER_CLASS__=$(DRIVER) -D __DRIVER_INCLUDE__='"$(DRIVER).h"'
# Uncomment the following line for a verbose client
# CXXFLAGS = -Wall -g -D __UDP_CLIENT_VERBOSE__

# Target [folder(s)]
BIN_DIR = bin
OBJ_DIR = obj
TARGET  = $(BIN_DIR)/FSMDriverPID

# Include directories
CLIENT_INC_DIR = ../../include/client
DRIVER_INC_DIR = include
FSM_INC_DIR    = ../../include/FSM
FSM3_INC_DIR   = ../../include/FSMDriver3

# Source
CLIENT_SRC_DIR = ../../src/client
CLIENT_SRC     = WrapperBaseDriver.cpp SimpleParser.cpp CarState.cpp CarControl.cpp
CLIENT_MAIN    = $(CLIENT_SRC_DIR)/client.cpp
DRIVER_SRC_DIR = src
DRIVER_SRC     = $(notdir $(wildcard $(DRIVER_SRC_DIR)/*.cpp))
FSM_SRC_DIR    = ../../src/FSM
FSM_SRC        = $(notdir $(wildcard $(FSM_SRC_DIR)/*.cpp))
FSM3_SRC       = ../../src/FSMDriver3/FSMDriver3.cpp




# DRIVER_SUP_SRC_DIR = src/$(DRIVER_SUP)
# DRIVER_SUP_SRC = $(DRIVER_SUP_SRC_DIR)/$(DRIVER_SUP).cpp

# Info
DOXYGEN    := $(shell doxygen --version 2>/dev/null)
DOXYFILE    = Doxyfile
DOC_OUTPUT ?= .

############
# Organize #
############
CLIENT_OBJ = $(patsubst %.cpp,$(OBJ_DIR)/%.o,$(CLIENT_SRC))
DRIVER_OBJ = $(patsubst %.cpp,$(OBJ_DIR)/%.o,$(DRIVER_SRC))
FSM_OBJ    = $(patsubst %.cpp,$(OBJ_DIR)/%.o,$(FSM_SRC))
FSM3_OBJ   = $(OBJ_DIR)/FSMDriver3.o

HEADERS    = $(addprefix -I,$(CLIENT_INC_DIR) $(FSM_INC_DIR) $(DRIVER_INC_DIR) $(FSM3_INC_DIR))
OBJECTS    = $(CLIENT_OBJ) $(FSM_OBJ) $(FSM3_OBJ) $(DRIVER_OBJ)

FLAGS      = $(CXXFLAGS) $(EXTFLAGS)


###########
# Targets #
###########
all: $(TARGET)

dirs:
	@mkdir -p $(OBJ_DIR)
	@mkdir -p $(BIN_DIR)

$(OBJ_DIR)/%.o: $(CLIENT_SRC_DIR)/%.cpp
	$(CC) -c -o $@ $< $(CXXFLAGS) -I $(CLIENT_INC_DIR)

$(OBJ_DIR)/%.o: $(FSM_SRC_DIR)/%.cpp
	$(CC) -c -o $@ $< $(CXXFLAGS) -I $(FSM_INC_DIR) -I $(CLIENT_INC_DIR)

$(OBJ_DIR)/FSMDriver3.o: $(FSM3_SRC)
	$(CC) -c -o $@ $< $(CXXFLAGS) -I $(FSM_INC_DIR) -I $(CLIENT_INC_DIR) -I $(FSM3_INC_DIR)

#$(DRIVER_OBJ): $(DRIVER_SRC_DIR)/$(DRIVER_SRC) $(FSM_OBJ)
$(OBJ_DIR)/%.o: $(DRIVER_SRC_DIR)/%.cpp $(FSM_OBJ)
	$(CC) -c -o $@ $< $(CXXFLAGS) $(HEADERS)

$(TARGET): dirs $(CLIENT_OBJ) $(DRIVER_OBJ) $(FSM3_OBJ) $ $(BIN_DIR)
	$(CC) $(FLAGS) -o $(TARGET) $(CLIENT_MAIN) $(HEADERS) $(OBJECTS)

info: $(DOXYFILE)
ifdef DOXYGEN
	( cat $(DOXYFILE) ; echo "OUTPUT_DIRECTORY=$(DOC_OUTPUT)" ) | doxygen -
else
	$(info "Doxygen not installed (http://www.doxygen.org).")
	exit 1
endif

clean:
	@rm -rf $(OBJ_DIR)/*.o $(BIN_DIR)/* $(DOC_OUTPUT)/html
