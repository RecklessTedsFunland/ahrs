cmake_minimum_required(VERSION 2.8.3)
project(ahrs)


find_package(catkin REQUIRED COMPONENTS roscpp genmsg std_msgs geometry_msgs sensor_msgs)

find_package(PkgConfig)
find_package(Boost REQUIRED COMPONENTS program_options)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg FILES Imu.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

#catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   DEPENDS sensor_msgs geometry_msgs std_msgs
#   )
   
#include_directories(include ${catkin_INCLUDE_DIRS})

################ YAML-CPP #########################
pkg_check_modules(YAML REQUIRED yaml-cpp)
if(YAML_FOUND)
	message("==== Found Yaml-Cpp ${YAML_VERSION} ====")
	include_directories(${YAML_INCLUDE_DIRS})
	link_directories(${YAML_LIBRARY_DIRS})
endif(YAML_FOUND)


################ IMU Node #########################
add_executable(ahrs src/ahrs.cpp)
target_link_libraries(ahrs ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${YAML_LIBRARIES})


################ IMU Node #########################
#rosbuild_add_executable(ahrs2 src/ahrs2.cpp)
#rosbuild_link_boost(ahrs2 program_options)

################ IMU Viewer #########################
# Qt
pkg_check_modules(QT REQUIRED QtXml QtCore QtGui QtOpenGL)
if(QT_FOUND)
	message("======== FOUND Qt ${QT_VERSION} =============")
	add_definitions(-DQT_NO_KEYWORDS)
	include_directories(${QT_INCLUDE_DIRS})
	link_directories(${QT_LIBRARY_DIRS})
	#message("Include: ${QT_INCLUDE_DIRS}")
	#message("Libraries: ${QT_LIBRARIES} ${QT_LIBRARY_DIRS} ${QT_LIBS}")
	#message("Libraries2: " pkg-config --libs QtXml QtCore QtGui QtOpenGL )
endif(QT_FOUND)

add_executable(ahrs_viewer src/ahrs_viewer.cpp)
target_link_libraries(ahrs_viewer ${catkin_LIBRARIES} ${OPENGL_LIBRARY} ${QT_LIBRARIES} )
set(CMAKE_EXE_LINKER_FLAGS "-framework OpenGL -F/usr/local/Cellar/libqglviewer/2.3.17 -framework QGLViewer -F/usr/local/Cellar/qt/4.8.3/lib -framework QtCore -framework QtOpenGL -framework QtXml -framework QtGui")
#set(CMAKE_EXE_LINKER_FLAGS "-framework OpenGL -F/usr/local/Cellar/libqglviewer/2.3.17  -framework QGLViewer")

install(TARGETS ahrs ahrs_viewer RUNTIME DESTINATION bin) 

#install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
#)