cmake_minimum_required(VERSION 3.10)

add_subdirectory(external/CLI11)
add_subdirectory(external/uatlib)

add_executable(simulation src/main.cpp src/astar.cpp src/hexgrid.cpp src/anxious.cpp)
target_compile_features(simulation PRIVATE cxx_std_17)

find_package(ZLIB)
target_link_libraries(simulation PRIVATE CLI11::CLI11 uat ZLIB::ZLIB)
target_include_directories(simulation PRIVATE include)
