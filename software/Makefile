CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O2

LIB_SRC := leg_ik.cpp gait.cpp
LIB_OBJ := $(LIB_SRC:.cpp=.o)
TEST_BIN := tests/test_main

.PHONY: all test clean

all: $(TEST_BIN)

%.o: %.cpp %.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

tests/test_main.o: tests/test_main.cpp leg_ik.hpp gait.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TEST_BIN): $(LIB_OBJ) tests/test_main.o
	$(CXX) $(CXXFLAGS) $^ -o $@

test: $(TEST_BIN)
	./$(TEST_BIN)

clean:
	rm -f $(LIB_OBJ) tests/test_main.o $(TEST_BIN)
