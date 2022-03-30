cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(UseLidarInstall OFF)
PROJECT(LidarVisu)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#Include directories
include_directories(
     ${CMAKE_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/external
)
add_executable(${PROJECT_NAME} src/main.cpp src/Implementation/YLidar_x4.cpp src/Implementation/VisualizationHandler.cpp)

if(UseLidarInstall)
    #YLIDAR USE INSTALL
    #find ydlidar_sdk package
    find_package(ydlidar_sdk REQUIRED)
    #Include directories
    include_directories(
      ${YDLIDAR_SDK_INCLUDE_DIRS}
    )
    #link library directories
    link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})
    #Link your project to ydlidar_sdk library.
    target_link_libraries(${PROJECT_NAME} ${YDLIDAR_SDK_LIBRARIES})
else()
    # YDLIDAR SDK STATIC LINK
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)
    target_link_libraries(${PROJECT_NAME}  Threads::Threads)

    find_library(CMAKE_SOURCE_DIR/external/libydlidar_sdk.a REQUIRED)
    add_library(libydlidar_sdk STATIC IMPORTED)
    set_target_properties(libydlidar_sdk PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/external/libydlidar_sdk.a)
    target_link_libraries(${PROJECT_NAME} libydlidar_sdk)
endif()
############## YDLIDAR SDK END#####################################
############## PCL START ##########################################
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
############## PCL END##########################################
