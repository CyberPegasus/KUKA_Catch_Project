cmake_minimum_required(VERSION 3.16)
project(camera_client)

set(CMAKE_CXX_STANDARD 14)
set(INC_DIR /home/wxj/freenect2/include)
set(LINK_DIR /home/wxj/freenect2/lib)

include_directories(${INC_DIR})
link_directories(${LINK_DIR})
link_libraries(freenect2)

find_package(OpenCV REQUIRED)

add_executable(camera_client main.cpp)

target_link_libraries(camera_client freenect2)
target_link_libraries(camera_client ${OpenCV_LIBS})