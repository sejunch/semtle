CXX      ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra
LDFLAGS  := $(shell pkg-config --libs sdl2)
CPPFLAGS := $(shell pkg-config --cflags sdl2)

semtle: semtle.cpp
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $< -o $@ $(LDFLAGS)

run: semtle
	./semtle

clean:
	rm -f semtle

.PHONY: run clean
