cmake_minimum_required(VERSION 3.11)
project(visualOdometry)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

add_library(visualOdometry SHARED utils.cpp bucket.cpp feature.cpp visualOdometry.cpp Wrapper.hpp)
target_include_directories(visualOdometry PUBLIC ${OpenCV_INCLUDE_DIRS})
target_include_directories(visualOdometry PUBLIC ${EIGEN3_INCLUDE_DIR})
target_link_libraries(visualOdometry ${OpenCV_LIBS})


add_executable(run main.cpp)
target_link_libraries(run ${OpenCV_LIBS})
target_link_libraries(run visualOdometry)