cmake_minimum_required(VERSION 3.6)
set(PROJECT_NAME Boolean)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 20)
# 全局设置使用动态运行时库
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/executable/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/executable/${CMAKE_BUILD_TYPE})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/executable/${CMAKE_BUILD_TYPE})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
add_compile_definitions(_CRT_SECURE_NO_WARNINGS)

set(BOOLEAN_TOOL BooleanTool)

file(GLOB SOURCE_FILES
    src/Boolean/*
    src/vec.cpp
    src/vec.h
    Boolean/*.cpp
)

add_executable(${BOOLEAN_TOOL} ${SOURCE_FILES})
find_package(Eigen3 REQUIRED)
target_include_directories(${BOOLEAN_TOOL} PUBLIC ./include
    PRIVATE
    src/Boolean/
    src/
    ThirdParty/HMath/include
)

target_link_libraries(${BOOLEAN_TOOL} PRIVATE 
    Eigen3::Eigen
)
