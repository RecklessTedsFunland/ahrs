cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

################ IMU Node #########################
pkg_check_modules(YAML yaml-cpp)
if(YAML_FOUND)
	message("==== Found Yaml-Cpp ${YAML_VERSION} ====")
	include_directories(${YAML_INCLUDE_DIRS})
	link_directories(${YAML_LIBRARY_DIRS})
endif(YAML_FOUND)
rosbuild_add_executable(ahrs src/ahrs.cpp)
rosbuild_link_boost(ahrs program_options)
target_link_libraries(ahrs ${YAML_LIBRARIES})

################ IMU Node #########################
#rosbuild_add_executable(ahrs2 src/ahrs2.cpp)
#rosbuild_link_boost(ahrs2 program_options)

################ IMU Viewer #########################
# Qt
#find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
pkg_check_modules(QT REQUIRED QtXml QtCore QtGui QtOpenGL)
if(QT_FOUND)
	message("======== FOUND Qt ${QT_VERSION} =============")
	#include(${QT_USE_FILE})
	add_definitions(-DQT_NO_KEYWORDS)
	include_directories(${QT_INCLUDE_DIRS} /usr/local/Cellar/qt/4.8.0/include/QtXml)
	link_directories(${QT_LIBRARY_DIRS})
	message("--${QT_INCLUDE_DIRS}")
	message("--${QT_LIBS}")
endif(QT_FOUND)

rosbuild_add_executable(ahrs_viewer src/ahrs_viewer.cpp)
target_link_libraries(ahrs_viewer ${OPENGL_LIBRARY} ${QT_LIBRARIES})
set(CMAKE_EXE_LINKER_FLAGS "-framework OpenGL -F/usr/local/Cellar/libqglviewer/2.3.15 -framework QGLViewer -F/usr/local/Cellar/qt/4.8.0/lib -framework QtCore -framework QtOpenGL -framework QtXml -framework QtGui")
