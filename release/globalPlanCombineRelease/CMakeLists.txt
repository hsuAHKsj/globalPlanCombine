cmake_minimum_required(VERSION 3.0.2)
project(mainChannelsLib)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
    message(STATUS "optional:-std=c++11")
endif(CMAKE_COMPILER_IS_GNUCXX)

include_directories("./include")
include_directories("D:/eigen-3.4.0")
include_directories("C:/Users/Administrator/Desktop/huangyanwei/json-develop/include")
include_directories("/usr/include/eigen3")
include_directories("/home/huang/Desktop/json-develop/include")

link_directories("./../../lib")
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

add_executable(
  testGlobalPlannerInterface
  test/testGlobalPlannerInterface.cpp
)
target_link_libraries(testGlobalPlannerInterface
  CmpAgu.a
)