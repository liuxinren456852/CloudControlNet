
cmake_minimum_required(VERSION 2.8)

project(cloudcontrolnet)

SET(CMAKE_CXX_FLAGS "-O3 -std=c++11 -w -fopenmp -pthread")
SET(CMAKE_C_FLAGS "-O3 -std=c++11 -fopenmp -pthread")

#SET(CMAKE_BUILD_TYPE "Debug") 
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

#PCL
#Boost, vtk, flann, eigen are involved in PCL
FIND_PACKAGE(PCL REQUIRED)
IF(PCL_FOUND) 
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})
LIST(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
message("PCL [OK]")
ENDIF(PCL_FOUND)

#libLas
FIND_PACKAGE(libLAS REQUIRED)
IF(libLAS_FOUND) 
MESSAGE("LibLAS [OK]") 
INCLUDE_DIRECTORIES(${LIBLAS_INCLUDE_DIR})
LINK_DIRECTORIES(${LIBLAS_LIBS})
LINK_LIBRARIES("/usr/local/lib/liblas.so.2.4.0")
ENDIF(libLAS_FOUND)

#PROJ4
FIND_PACKAGE(PROJ4 REQUIRED)
IF(PROJ4_FOUND)
    include_directories(${PROJ4_INCLUDE_DIR})
	message("PROJ4 [OK]")
ENDIF()

#SET( PROJ4_ROOT /usr/local/include/proj )
#INCLUDE_DIRECTORIES(${PROJ4_INCLUDE_DIR}) 

# ceres
# glog and gflag are involved in ceres
FIND_PACKAGE(Ceres REQUIRED)
IF(Ceres_FOUND)
MESSAGE("CERES [OK]") 
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
ENDIF(Ceres_FOUND)

# g2o 
find_package( G2O REQUIRED )
IF(G2O_FOUND)
MESSAGE("G2O [OK]") 
include_directories( ${G2O_INCLUDE_DIR} )
ENDIF(G2O_FOUND)

# head
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src SRC_LIST)
message("SRC_LIST is " ${SRC_LIST})

SET(DEP_LIBS ${DEP_LIBS} ${PCL_LIBRARIES} ${LibLAS_LIBRARIES} ${CERES_LIBRARIES} )

SET(DEP_LIBS ${DEP_LIBS} glog::glog /usr/local/lib/libproj.so g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension)

add_executable(ccn ${PROJECT_SOURCE_DIR}/test/ccn_main.cpp ${SRC_LIST})
target_link_libraries(ccn ${DEP_LIBS})

add_executable(test_reg ${PROJECT_SOURCE_DIR}/test/test_reg.cpp ${SRC_LIST})
target_link_libraries(test_reg ${DEP_LIBS})




