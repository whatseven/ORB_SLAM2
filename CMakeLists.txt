cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

FILE(GLOB SOURCE_FILE src/*.c src/*.cc include/*.h)

find_package(OpenCV 4 REQUIRED)
find_package(Pangolin CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(g2o CONFIG REQUIRED)

set(DBOW2_RELEASE_LIB "${OpenCV_DIR}/../../lib/DBoW2.lib")
set(DBOW2_DEBUG_LIB "${OpenCV_DIR}/../../debug/lib/DBoW2.lib")

include_directories(
   ${PROJECT_SOURCE_DIR}
   ${PROJECT_SOURCE_DIR}/include
)

#add_library(${PROJECT_NAME} ${SOURCE_FILE})
#link_general(${PROJECT_NAME})

add_executable(${PROJECT_NAME} Examples/Monocular/mono_kitti.cc ${SOURCE_FILE})
target_link_libraries(${PROJECT_NAME} pangolin g2o::core g2o::stuff g2o::types_icp g2o::types_sba)
target_link_libraries(${PROJECT_NAME} opencv_ml opencv_dnn opencv_core opencv_flann)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} debug ${DBOW2_DEBUG_LIB})
target_link_libraries(${PROJECT_NAME} optimized ${DBOW2_RELEASE_LIB})
