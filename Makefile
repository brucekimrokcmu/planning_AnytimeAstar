all: main

FValueCompare o: FValueCompare.hpp
	g++ -c FValueCompare.hpp

GraphHelper.o: GraphHelper.cpp GraphHelper.hpp
	g++ -c GraphHelper.cpp GraphHelper.hpp

main: main.cpp FValueCompare.o GraphHelper.o
	g++ main.cpp FValueCompare.o GraphHelper.o -o main

clean:
	rm -f *.o *~ main
    int posex = goalNode.GetPoseX();