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
link_directories("./lib")

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

set(LIBRARY_OUTPUT_PATH ../lib) 

ADD_LIBRARY(CmpAgu STATIC
  src/fileOperation.cpp
  src/AdjMatrix.cpp
  src/globalPath.cpp
  src/utilityFun.cpp
  src/refVector.cpp
  src/globalPlanInterface.cpp
  src/rs.cpp
)

# add_executable(
#   readMainChannels
#   test/readMainChannels.cpp
#   src/fileOperation.cpp
#   src/AdjMatrix.cpp
#   src/globalpath.cpp
#   src/utilityFun.cpp
#   src/refVector.cpp
# )

# add_executable(
#   testProjection
#   test/testProjection.cpp
# )

# add_executable(
#   testCheckPointInCapsule
#   test/testCheckPointInCapsule.cpp
#   src/fileOperation.cpp
# )

# add_executable(
#   testIntersection
#   test/testIntersection.cpp
#   src/fileOperation.cpp
# )

# add_executable(
#   testIsPointInsideSegmentment
#   test/testIsPointInsideSegmentment.cpp
#   src/AdjMatrix.cpp
# )

# add_executable(
#   testBuildAdjanceMatrix
#   test/testBuildAdjanceMatrix.cpp
#   src/AdjMatrix.cpp
# )

# add_executable(
#   test_Dijstra
#   test/test_Dijstra.cpp
#   src/AdjMatrix.cpp
# )

# add_executable(
#   testPath2RefVector
#   test/testPath2RefVector.cpp
#   src/refVector.cpp
# )

# add_executable(
#   test_rs
#   test/test_pp.cpp
#   src/rs.cpp
# )

# add_executable(
#   testRead
#   test/testRead.cpp
# )


# add_executable(
#   testGlobalPlannerInterface
#   test/testGlobalPlannerInterface.cpp
# )
# target_link_libraries(testGlobalPlannerInterface
#   CmpAgu.a
# )