#指定CMake最低版本
cmake_minimum_required (VERSION 3.10)

#指定项目名称
project ("hs_exoskeleton")

#指定c++标准
set(CMAKE_CXX_STANDARD 17)

#指定头文件路径
set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(INCLUDE_DIRS "include")
include_directories("include/Kvaser_H")
include_directories("include/Robot_H")
include_directories("include/DataManager_H")
include_directories("include/Controller_H")
include_directories("include/Trajectory_H")


include_directories(KVASER_INCLUDE_DIRS "D:/Kvaser/Canlib/INC"
                    EIGEN3_INCLUDE_DIRS "D:/eigen-3.4.0")
link_directories(KVASER_LIBRARY_DIRS "D:/Kvaser/Canlib/Lib/x64" )


#搜索源文件
#aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SRC)
file(GLOB SRC "src/*.cpp" 
"src/Kvaser_CPP/*.cpp" 
"src/Robot_CPP/*.cpp"
"src/DataManager_CPP/*.cpp"
"src/Controller_CPP/*.cpp"
"src/Trajectory_CPP/*.cpp")

#将源代码添加到此项目的可执行文件。
add_executable (${PROJECT_NAME} hs_exoskeleton.cpp ${SRC})

target_link_libraries(hs_exoskeleton "D:/Kvaser/Canlib/Lib/x64/canlib32.lib")

#测试用可执行文件
add_executable(test test.cpp ${SRC})
target_link_libraries(test "D:/Kvaser/Canlib/Lib/x64/canlib32.lib")


