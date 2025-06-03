# Default build configuration for HF-AS5047U
CXX ?= g++
AR  ?= ar
CPPFLAGS ?= -I./src -I.
CXXFLAGS ?= -std=c++20 -Wall -Wextra -pedantic -O2
LDFLAGS ?=
