project(slam_example)
cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(/usr/local/include include)

find_package(unitree_sdk2 REQUIRED)

add_executable(keyDemo src/keyDemo.cpp)
target_link_libraries(keyDemo unitree_sdk2)

add_executable(demo src/demo.cpp)
target_link_libraries(demo unitree_sdk2)

add_executable(demo1 src/demo1.cpp)
target_link_libraries(demo1 unitree_sdk2)

add_executable(demo2 src/demo2.cpp)
target_link_libraries(demo2 unitree_sdk2)

add_executable(slamv1 src/slamv1.cpp)
target_link_libraries(slamv1 unitree_sdk2)

add_executable(demo1_1 src/demo1_1.cpp)
target_link_libraries(demo1_1 unitree_sdk2)
add_executable(nav_final src/nav_final.cpp)
target_link_libraries(nav_final unitree_sdk2)


add_executable(keydemo0710 src/keydemo0710.cpp)
target_link_libraries(keydemo0710 unitree_sdk2)

add_executable(demowater src/demowater.cpp)
target_link_libraries(demowater unitree_sdk2)

add_executable(notts src/notts.cpp)
target_link_libraries(notts unitree_sdk2)


add_executable(slam src/notts1.cpp)
target_link_libraries(slam unitree_sdk2)
