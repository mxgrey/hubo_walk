cmake_minimum_required(VERSION 2.8.3)
project(hubo_walk)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package( catkin REQUIRED )


## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp rospy
    DEPENDS ach QtCore QtGui )


## Find if hubomz is installed. If not, do not use zmp command stuff
find_path(HUBOMZ_INCLUDE "zmp-daemon.h" PATHS ${CMAKE_INCLUDE_PATH})

if(${HUBOMZ_INCLUDE} STREQUAL HUBOMZ_INCLUDE-NOTFOUND)
  message("\n-- INFO: could not find hubomz includes. will not load zmp tab\n")
  set(HAVE_HUBOMZ 0)
else()
  message("\n-- found hubomz includes at ${HUBOMZ_INCLUDE}\n")
  add_definitions(-DHAVE_HUBOMZ)
  set(HAVE_HUBOMZ 1)
endif()


## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  include/hubo_walk.h
)


## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/hubo_walk.cpp
  src/hubo_walk_slots.cpp
  include/hubo_walk.h
  ${MOC_FILES}
)

include_directories( include )

## Set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` (which is
## "rviz_plugin_tutorials", or whatever your version of this project
## is called) and specify the list of source files we collected above
## in ``${SOURCE_FILES}``.
#rosbuild_add_library(${PROJECT_NAME} ${SOURCE_FILES})

add_library( ${PROJECT_NAME} ${SOURCE_FILES} )


## Link the library with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above.
##
## Although this puts "rviz_plugin_tutorials" (or whatever you have
## called the project) as the name of the library, cmake knows it is a
## library and names the actual file something like
## "librviz_plugin_tutorials.so", or whatever is appropriate for your
## particular OS.
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ach huboparams)

