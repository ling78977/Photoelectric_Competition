cmake_minimum_required(VERSION 3.10.0)
project(aa VERSION 2023.7.12 LANGUAGES CXX)
# Find and add opencv as required
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
set(CMAKE_BUILD_TYPE Debug)


#添加自定义的库
add_library(uart SHARED src/uart/src/uart.cpp)
set_target_properties(uart PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY lib)
target_compile_options(uart PRIVATE -w)
#链接外部库
target_link_libraries(uart ${OpenCV_LIBS})


#宝藏识别库 
add_library(treasure SHARED src/treasure/src/treasure.cpp)
set_target_properties(treasure PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY lib)
target_compile_options(treasure PRIVATE -w)
#链接外部库
target_link_libraries(treasure ${OpenCV_LIBS})


#宝藏图识别库 
add_library(treasuremp SHARED src/treasure_map/src/TreasureMap.cpp)
set_target_properties(treasuremp PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY lib)
target_compile_options(treasuremp PRIVATE -w)
#链接外部库
target_link_libraries(treasuremp ${OpenCV_LIBS})

#A*算法库
add_library(astar SHARED src/Astar/src/myAstar.cpp)
set_target_properties(astar PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY lib)

#测试工具

#生成可执行文件
add_executable(testtreasure utils/testtreasure.cpp)
#链接本地库
target_link_libraries(testtreasure treasure)
#放入指定目录
set_target_properties(testtreasure PROPERTIES RUNTIME_OUTPUT_DIRECTORY bin/utils)

#生成可执行文件
add_executable(testserial utils/testserial.cpp)
#链接本地库
target_link_libraries(testserial uart)
#放入指定目录
set_target_properties(testserial PROPERTIES RUNTIME_OUTPUT_DIRECTORY bin/utils)

#生成可执行文件
add_executable(test_treasuremap utils/test_treasuremap.cpp)
#链接本地库
target_link_libraries(test_treasuremap treasuremp)
#放入指定目录
set_target_properties(test_treasuremap PROPERTIES RUNTIME_OUTPUT_DIRECTORY bin/utils)

#生成可执行文件
add_executable(test_astar utils/test_astar.cpp)
#链接本地库和外部库
target_link_libraries(test_astar astar)
target_link_libraries(test_astar uart)
target_link_libraries(test_astar ${OpenCV_LIBS})
#放入指定目录
set_target_properties(test_astar PROPERTIES RUNTIME_OUTPUT_DIRECTORY bin/utils)

#生成主函数程序
add_executable(main main.cpp)
#链接本地库
target_link_libraries(main uart)
target_link_libraries(main astar)
target_link_libraries(main treasuremp)
target_link_libraries(main treasure)
#放进指定目录
set_target_properties(main PROPERTIES RUNTIME_OUTPUT_DIRECTORY bin)


add_executable(write_point write_point.cpp)
#链接本地库
target_link_libraries(write_point uart)
target_link_libraries(write_point astar)
target_link_libraries(write_point treasuremp)
target_link_libraries(write_point treasure)
#放进指定目录
set_target_properties(write_point PROPERTIES RUNTIME_OUTPUT_DIRECTORY bin)

add_executable(start_competition start_competition.cpp)
#链接本地库
target_link_libraries(start_competition uart)
target_link_libraries(start_competition astar)
target_link_libraries(start_competition treasuremp)
target_link_libraries(start_competition treasure)
target_link_libraries(start_competition ${OpenCV_LIBS})
#放进指定目录
set_target_properties(start_competition PROPERTIES RUNTIME_OUTPUT_DIRECTORY bin)


#生成主函数程序
add_executable(only_path only_path.cpp)
#链接本地库
target_link_libraries(only_path uart)
target_link_libraries(only_path astar)
target_link_libraries(only_path treasuremp)
target_link_libraries(only_path treasure)
#放进指定目录
set_target_properties(only_path PROPERTIES RUNTIME_OUTPUT_DIRECTORY bin)
