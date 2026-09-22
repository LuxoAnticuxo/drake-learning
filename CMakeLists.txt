cmake_minimum_required(VERSION 3.20)
project(drake_learning CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(drake CONFIG REQUIRED)

add_executable(drake_sim src/main.cc
        src/main.cc)
target_link_libraries(drake_sim PRIVATE drake::drake)
