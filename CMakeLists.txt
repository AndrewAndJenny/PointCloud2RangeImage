cmake_minimum_required (VERSION 3.1)
project(rangeimagetool CXX)

#set path
set(LASLIB_INCLUDE_DIR "/usr/local/include/LASlib")
set(LASLIB_LIBRARY_PATH "/usr/local/lib/LASlib")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

aux_source_directory(./src SRC_DIR)

#pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

#gdal
find_package(GDAL REQUIRED)
include_directories(${GDAL_INCLUDE_DIR})

#Qt
find_package(Qt5 COMPONENTS Core REQUIRED)
include_directories(${Qt5Core_INCLUDE_DIRS})

#LASLIB
find_library(LASLIB_LIBRARY libLASlib.a HINTS ${LASLIB_LIBRARY_PATH})
include_directories(${LASLIB_INCLUDE_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# set MD to MT
if(OPTION_BUILD_MT)
    set(CompilerFlags
            CMAKE_CXX_FLAGS
            CMAKE_CXX_FLAGS_DEBUG
            CMAKE_CXX_FLAGS_RELEASE
            CMAKE_C_FLAGS
            CMAKE_C_FLAGS_DEBUG
            CMAKE_C_FLAGS_RELEASE
            )
foreach(CompilerFlags ${CompilerFlags})
    string(REPLACE "/MD" "/MT" ${CompilerFlags} "${${CompilerFlags}}")
    string(REPLACE "/Od" "/O2" ${CompilerFlags} "${${CompilerFlags}}")
endforeach()
endif()

add_executable (${PROJECT_NAME} ${SRC_DIR})

target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC ${GDAL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC Qt5::Core)
target_link_libraries(${PROJECT_NAME} PUBLIC ${LASLIB_LIBRARY})
