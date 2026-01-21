cmake_minimum_required(VERSION 3.10)

project(AtlasSDK LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(AtlasSDK
        AtlasSDK.cpp
        AtlasSDK.h
)

add_subdirectory(atlas)
add_subdirectory(tests)

target_include_directories(AtlasSDK PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

