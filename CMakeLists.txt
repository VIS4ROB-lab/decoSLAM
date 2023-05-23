cmake_minimum_required(VERSION 3.16)
project(DecoSLAM)

include(FetchContent)

# Required language standard.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++2a -O3") # -fconcepts -fopenmp -Wall -Wextra -Wnon-virtual-dtor -pedantic")

find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(opengv REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cppzmq REQUIRED)

add_library(roscpp INTERFACE)
set_target_properties(roscpp PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${roscpp_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${roscpp_LIBRARIES}")

add_library(std_msgs INTERFACE)
set_target_properties(std_msgs PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${std_msgs_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${std_msgs_LIBRARIES}")

add_library(visualization_msgs INTERFACE)
set_target_properties(visualization_msgs PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${visualization_msgs_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${visualization_msgs_LIBRARIES}")

# Hyper libraries
FetchContent_Declare(HyperVariables GIT_REPOSITORY https://github.com/VIS4ROB-lab/HyperVariables.git GIT_TAG e46e67af46078a1163a7903d48bd98fe34cf4b62)
FetchContent_MakeAvailable(HyperVariables)
FetchContent_Declare(HyperSensors GIT_REPOSITORY https://github.com/VIS4ROB-lab/HyperSensors.git GIT_TAG a24e9e1a7043618623a75afb403ab24fe6aa14c0)
FetchContent_MakeAvailable(HyperSensors)


# Summarize the source files
file(GLOB_RECURSE SOURCE_FILES src/global.cpp
        src/map/*
        src/optimization/*
        src/storage/*
        src/system/*
        src/utils/*
        src/visualization/*)

add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/thirdparty/cereal-1.3.2/include)
target_link_libraries(${PROJECT_NAME} PUBLIC
        roscpp
        std_msgs
        visualization_msgs
        ${OpenCV_LIBS}
        ceres
        yaml-cpp
        HyperVariables
        HyperSensors
        opengv
        Eigen3::Eigen
        zmq)

add_executable(main src/main.cpp)
target_link_libraries(main ${PROJECT_NAME})

add_executable(combine_poses evaluation/main.cpp)
target_link_libraries(combine_poses ${PROJECT_NAME})

# Tests
enable_testing()
find_package(GTest)
add_subdirectory(test)
