cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 11)

project(Capture)

find_package(freenect2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${freenect2_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} capture.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${freenect2_LIBRARIES})
