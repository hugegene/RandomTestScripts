cmake_minimum_required(VERSION 3.16.3)

project(threading)

find_package (Threads REQUIRED)



add_executable (threading_main main.cpp)

target_link_libraries(threading_main Threads::Threads)
