# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.4.1)

set(NDK_LIBS ${CMAKE_CURRENT_SOURCE_DIR}/../ndkLibs)

add_library(x264-lib STATIC IMPORTED)

set_target_properties(x264-lib PROPERTIES IMPORTED_LOCATION ${NDK_LIBS}/x264/lib/${ANDROID_ABI}/libx264.a)

add_library(rtmp-lib STATIC IMPORTED)

set_target_properties(rtmp-lib PROPERTIES IMPORTED_LOCATION ${NDK_LIBS}/rtmpdump/lib/${ANDROID_ABI}/librtmp.a)

add_library(fdkaac-lib STATIC IMPORTED)

set_target_properties(fdkaac-lib PROPERTIES IMPORTED_LOCATION ${NDK_LIBS}/fdkaac/lib/${ANDROID_ABI}/libFraunhoferAAC.a)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

add_library(
             native-lib
             SHARED
             src/main/cpp/native-lib.cpp
             src/main/cpp/queue.c )

target_include_directories(native-lib PRIVATE
           ${NDK_LIBS}/x264/include
           ${NDK_LIBS}/rtmpdump/include
           ${NDK_LIBS}/fdkaac/include)

target_link_libraries( # Specifies the target library.
                       native-lib x264-lib rtmp-lib fdkaac-lib

                       # Links the target library to the log library
                       # included in the NDK.
                       log
                       )