SET(SRC_LIST data_reading.cpp plane_feature_extraction.cpp pose_estimation.cpp plane_feature_matching.cpp plane_map_update.cpp)

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
find_package( G2O )
include_directories( ${G2O_INCLUDE_DIRS} ) 
set( G2O_LIBS 
    g2o_core g2o_types_sba g2o_solver_csparse g2o_stuff g2o_csparse_extension 
)

SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/ann_1.1.2/include ${Sophus_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_directories(${PROJECT_SOURCE_DIR}/ann_1.1.2/lib)

ADD_LIBRARY(ulysses SHARED ${SRC_LIST})
target_link_libraries(ulysses ${PCL_LIBRARIES} ${OpenCV_LIBS} ANN ${Sophus_LIBRARIES} ${G2O_LIBS})
