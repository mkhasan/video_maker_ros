cmake_minimum_required(VERSION 2.8.3)
project(video_maker)


add_compile_options(-std=c++14)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ggdb -Wall -D_GNU_SOURCE=1 -D_REENTRANT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb -Wall -fpermissive -D_GNU_SOURCE=1 -D_REENTRANT")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  client_interface
  
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES video_maker
   CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  /usr/local/include/ImageMagick-7
  $ENV{MAGICK_ROOT} 
  ${catkin_INCLUDE_DIRS}
)


find_library(MQTT mosquittopp)
find_library(ACE ACE)
find_library(MAGIC_HDRI Magick++-7.Q16HDRI HINTS "/usr/local/lib")


add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}.cpp
   src/JPEG_buffer.cpp 
   src/JPEG_video_source.cpp
   src/AVI_file_sink.cpp
   src/utils.cpp
   
)
target_link_libraries(${PROJECT_NAME} jpeg ${MAGIC_HDRI} )

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp src/cmd_handler.cpp )

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${MQTT}	
  ${catkin_LIBRARIES}
)

