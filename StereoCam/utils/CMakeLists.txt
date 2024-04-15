# so let's require it now.
cmake_minimum_required(VERSION 3.0.0)

#project(GeneralCapture)


#set(TARGET_NAME "multi_channel_common")

# Create library file from sources.
#add_library(GeneralCapture STATIC src/GeneralCapture.cpp)


# PUBLIC needed to make both hello.h and hello library available elsewhere in project
#target_include_directories( ${PROJECT_NAME} 
#    PUBLIC ${PROJECT_SOURCE_DIR}/include)
##############################33

file(GLOB_RECURSE HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/*")
file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*")

source_group("src" FILES ${SOURCES})
source_group("include" FILES ${HEADERS})

#set(OpenCV_DIR /home/leukos/opencv)

add_library(utils STATIC ${HEADERS} ${SOURCES})
#add_library(GeneralCapture STATIC src/GeneralCapture.cpp)
target_include_directories(utils PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(utils PRIVATE opencv_core opencv_imgcodecs opencv_videoio)

