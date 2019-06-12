find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(evaluate)

add_library(visualOdometry SHARED utils.cpp bucket.cpp feature.cpp visualOdometry.cpp ../Wrapper.hpp)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
message(${PROJECT_SOURCE_DIR})

add_executable(run main.cpp)
target_link_libraries(visualOdometry ${OpenCV_LIBS})

target_link_libraries(run ${OpenCV_LIBS} visualOdometry)