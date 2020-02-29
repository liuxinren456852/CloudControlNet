
cmake_minimum_required(VERSION 2.8)

project(cloudcontrolnet)

SET(CMAKE_CXX_FLAGS "-O3 -std=c++11 -w -fopenmp -pthread")
SET(CMAKE_C_FLAGS "-O3 -std=c++11 -fopenmp -pthread")

#SET(CMAKE_BUILD_TYPE "Debug") 
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#GLOG 
find_package(glog REQUIRED)

#PCL
FIND_PACKAGE(PCL REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})
LIST(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
message("PCL [OK]")

#PROJ4
SET( PROJ4_ROOT /usr/local/include/proj )
INCLUDE_DIRECTORIES(${PROJ4_INCLUDE_DIR}) 

# ceres
FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})

# g2o 
find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )

# head
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src SRC_LIST)
message("SRC_LIST is " ${SRC_LIST})

SET(DEP_LIBS ${DEP_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${G2O_LIBRARIES} /usr/local/lib/libproj.so glog::glog)

add_executable(ccn ${PROJECT_SOURCE_DIR}/test/ccn_main.cpp ${SRC_LIST})
target_link_libraries(ccn ${DEP_LIBS})
