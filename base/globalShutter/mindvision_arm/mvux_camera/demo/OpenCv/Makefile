CXX ?= g++
CXXFLAGS = -Wall -std=c++11 $(DEBUG)
CPPFLAGS += .... # pre-processor flags, for include paths etc.
CFLAGS =`pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`  -L../../lib/gcc-4.4.3/ -L/usr/local/cuda-6.5/lib
#SHARELIBDIR= -L../../../lib/gcc-4.4.3/
INCLUDE= -I../../include

all: main
main:  
	$(CXX) $(CFLAGS) $(CXXFLAGS) -o main  main.cpp manifoldGPIO.cpp $(LIBS)   $(INCLUDE)   -lMVSDK -lrt
clean:
	rm -f *.o 
	rm -f main 
