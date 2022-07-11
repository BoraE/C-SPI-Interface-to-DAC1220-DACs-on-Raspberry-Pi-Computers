# Copyright (C) 2022 Bora Eryilmaz

TARGETS = \
test_dac1220

# Write generated artifacts into these directories
BUILD_DIR = build
DOC_DIR = html

# Source files to compile and build
SRCS = $(wildcard *.cpp)
EXES = $(addprefix $(BUILD_DIR)/,$(TARGETS))
ALLOBJS = $(addprefix $(BUILD_DIR)/,$(SRCS:.cpp=.o))
OBJS = $(filter-out $(patsubst %,%.o,$(EXES)),$(ALLOBJS))

# Compiler
CPP = g++

# Compiler flags
CPP_FLAGS = -I. -g -Wall -std=c++14 -Wno-psabi -MMD -MP

# Linker flags
LD_FLAGS = -l bcm2835

all: $(EXES)

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR)
	@echo Compiling $@
	$(CPP) -c $(CPP_FLAGS) $< -o $@
	@echo

$(EXES): % : %.o $(OBJS) Makefile
	@echo Building $@
	$(CPP) $(OBJS) $< $(LD_FLAGS) -o $@
	@echo

$(BUILD_DIR):
	mkdir $@

.PHONY: clean doc tags test
clean:
	rm -fR $(BUILD_DIR) $(DOC_DIR)
	rm -f TAGS

doc:
	doxygen doxygen_config.txt

tags:
	ctags --c++-kinds=+px --recurse -e . ~/Downloads/bcm2835-1.71/src/ /usr/include/

test:
	sudo ./build/test_dac1220

-include $(wildcard $(BUILD_DIR)/*.d)
