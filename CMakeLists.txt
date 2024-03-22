PROJECT(ULYSSES)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

ADD_SUBDIRECTORY(src bin)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/ann_1.1.2/include)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
find_package( G2O )
include_directories( ${G2O_INCLUDE_DIRS} ) 
set( G2O_LIBS 
    g2o_core g2o_types_sba g2o_solver_csparse g2o_stuff g2o_csparse_extension 
)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set( CMAKE_EXPORT_COMPILE_COMMANDS ON ) 
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${PROJECT_SOURCE_DIR}/ann_1.1.2/lib)
add_definitions(${PCL_DEFINITIONS})

SET(SRC_LIST main.cpp)
#MESSAGE(STATUS "This is BINARY dir " ${SUTUNGPO_BINARY_DIR})
#MESSAGE(STATUS "This is SOURCE dir " ${SUTUNGPO_SOURCE_DIR})
ADD_EXECUTABLE(Ulysses ${SRC_LIST})
target_link_libraries(Ulysses ${PCL_LIBRARIES} ulysses ANN ${G2O_LIBS})

#INSTALL(TARGETS myrun mylib mystaticlib
#		RUNTIME DESTINATION bin
#		LIBRARY DESTINATION lib
#		ARCHIVE DESTINATION libstatic)

# how to install
#cmake -DCMAKE_INSTALL_PREFIX=/usr 

