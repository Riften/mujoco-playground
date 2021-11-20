cmake_minimum_required(VERSION 3.20)
project(mujoco_playground)

find_library(libmujoco mujoco210 HINTS ${CMAKE_CURRENT_SOURCE_DIR}/mujoco210/bin)
find_library(libglfw libglfw.so.3 HINTS ${CMAKE_CURRENT_SOURCE_DIR}/mujoco210/bin)
find_library(libglew glew HINTS ${CMAKE_CURRENT_SOURCE_DIR}/mujoco210/bin)
find_library(libgl GL)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(ompl REQUIRED)

set(MUJOCO_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mujoco210/include)
MESSAGE(STATUS "Found mujoco ${libmujoco}")
MESSAGE(STATUS "Found glwf ${libglfw}")
MESSAGE(STATUS "Found glew ${libglew}")
MESSAGE(STATUS "Found OpenGL ${OPENGL_LIBRARY}")
MESSAGE(STATUS "Found GLEW ${GLEW_LIBRARIES}")
MESSAGE(STATUS "Found GL ${libgl}")
MESSAGE(STATUS "Found OMPL ${OMPL_LIBRARIES}")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_subdirectory(demo)