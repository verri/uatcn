cmake_minimum_required(VERSION 3.10)

add_subdirectory(external/CLI11)
add_subdirectory(external/uatlib)

add_executable(simulation src/main.cpp src/astar.cpp)

find_package(Boost 1.56 REQUIRED COMPONENTS iostreams)
target_link_libraries(simulation PRIVATE CLI11::CLI11 uat Boost::iostreams)
target_include_directories(simulation PRIVATE include)
