cmake_minimum_required(VERSION 3.30)
project(_idea)

set(CMAKE_CXX_STANDARD 14)

add_executable(_idea
    editor.xml
    hw3_2sem.iml
    modules.xml
    workspace.xml
        ../graph.cpp
        ../GraphManager.cpp
        ../main.cpp
        ../PathFinder.cpp)
