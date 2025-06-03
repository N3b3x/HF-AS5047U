# HF-AS5047U Makefile
# Build library and run unit tests

include config.mk

SRC := $(wildcard src/*.cpp)
OBJ := $(SRC:.cpp=.o)
LIB := libas5047u.a

TEST_SRC := tests/test_as5047u.cpp
TEST_OBJ := $(TEST_SRC:.cpp=.o)
TEST_BIN := test

.PHONY: all lib test clean

all: lib

lib: $(LIB)

$(LIB): $(OBJ)
	$(AR) rcs $@ $^

%.o: %.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(TEST_BIN): $(OBJ) $(TEST_OBJ)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

test: $(TEST_BIN)
	./$(TEST_BIN)

clean:
	rm -f $(OBJ) $(TEST_OBJ) $(LIB) $(TEST_BIN)
