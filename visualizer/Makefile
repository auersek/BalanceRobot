CXX = g++
SRC = src/grid.cpp	# changed from grid.cpp to
INC = -Iinclude
CFLAGS = -std=c++17 $(shell sdl2-config --cflags)
LIBS = $(shell sdl2-config --libs)

all: visualizer

visualizer: $(SRC)
	$(CXX) $(CFLAGS) $(INC) $(SRC) -o $@ $(LIBS)

clean:
	rm -f visualizer
