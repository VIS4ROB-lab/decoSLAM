# Project name.
project(DecoTests)

# Include directories.
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/thirdparty/cereal-1.3.2/include)

# Collect all files.
file(GLOB_RECURSE FILES *.hpp *.tpp *.cpp)

# Create the executable.
add_executable(${PROJECT_NAME} ${FILES})
target_link_libraries(${PROJECT_NAME} ${CMAKE_PROJECT_NAME} glog gtest)
