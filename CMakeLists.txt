cmake_minimum_required(VERSION 2.8)

project(flocking-cmake)

find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} sfml-graphics sfml-window sfml-system)
