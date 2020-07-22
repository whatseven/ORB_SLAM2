cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

FILE(GLOB SOURCE_FILE include/*.h)

find_package(Pangolin CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(g2o CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)

set(DBOW2_RELEASE_LIB "${OpenCV_DIR}/../../lib/DBoW2.lib")
set(DBOW2_DEBUG_LIB "${OpenCV_DIR}/../../debug/lib/DBoW2.lib")

include_directories(
   ${PROJECT_SOURCE_DIR}
   ${PROJECT_SOURCE_DIR}/include
)

#add_executable(${PROJECT_NAME} Examples/Monocular/mono_kitti.cpp ${SOURCE_FILE})
add_executable(${PROJECT_NAME} Examples/Monocular/mono_kitti.cpp 
src/main_system.cpp src/tracking.cpp src/frame.cpp
${SOURCE_FILE}
)

target_link_libraries(${PROJECT_NAME} pangolin)
#target_link_libraries(${PROJECT_NAME} g2o::core g2o::stuff g2o::types_icp g2o::types_sba)
target_link_libraries(${PROJECT_NAME}  ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} debug ${DBOW2_DEBUG_LIB})
target_link_libraries(${PROJECT_NAME} optimized ${DBOW2_RELEASE_LIB})
