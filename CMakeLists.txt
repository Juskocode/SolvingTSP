cmake_minimum_required(VERSION 3.23)
project(tsp)

set(CMAKE_CXX_STANDARD 23)

add_executable(SolvingTSP main.cpp src/DataStructs/Graph.h src/DataStructs/minHeap.h
        src/DataStructs/Graph.cpp
        src/DataManagement/Parser.h
        src/DataManagement/Parser.cpp)
