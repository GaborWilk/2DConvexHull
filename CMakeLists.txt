cmake_minimum_required(VERSION 3.8)

project(convex_hull_of_intersections LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -Wextra")

set(SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/ConvexHull.cpp
    ${PROJECT_SOURCE_DIR}/src/main.cpp
    ${PROJECT_SOURCE_DIR}/src/Shape.cpp)

set(TEST_FILES
    ${PROJECT_SOURCE_DIR}/src/ConvexHull.cpp
    ${PROJECT_SOURCE_DIR}/src/Shape.cpp)

set(HEADER_FILES
    ${PROJECT_SOURCE_DIR}/include/ConvexHull.h
    ${PROJECT_SOURCE_DIR}/include/Shape.h)

set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
include_directories (${INCLUDE_DIR})

ENABLE_TESTING()
ADD_SUBDIRECTORY(test)

add_executable(convex_hull_of_intersections ${SOURCE_FILES} ${HEADER_FILES})
