cmake_minimum_required(VERSION 3.16)
project(mujoco_playground)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Build Options
option(BUILD_PYTHON_BINDING "Whether to build python bindings." ON)

# RPATH Setting
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH $ORIGIN)

find_library(libmujoco mujoco210 HINTS ${CMAKE_CURRENT_SOURCE_DIR}/mujoco210/bin)
find_library(libglfw libglfw.so.3 HINTS ${CMAKE_CURRENT_SOURCE_DIR}/mujoco210/bin)
find_library(libglew glew HINTS ${CMAKE_CURRENT_SOURCE_DIR}/mujoco210/bin)
find_library(libgl GL)
find_library(libkdl liborocos-kdl.so.1.5.1 HINTS ${CMAKE_CURRENT_SOURCE_DIR}/extern/kdl-1.5.1/lib)
find_library(libpsopt libPSOPT.a HINTS ${CMAKE_CURRENT_SOURCE_DIR}/extern/local/lib/PSOPT)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(ompl REQUIRED)
find_package(Log4cxx REQUIRED)
# KDL Chain Solver use Eigen3 publicly
find_package(Eigen3 REQUIRED)
find_package(IPOPT REQUIRED)
find_package(ADOLC REQUIRED)

set(MUJOCO_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mujoco210/include)
set(KDL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/extern/kdl-1.5.1/include)
set(LOCAL_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/extern/local/include)
MESSAGE(STATUS "CMAKE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}")
MESSAGE(STATUS "Found mujoco ${libmujoco}")
MESSAGE(STATUS "Found glwf ${libglfw}")
MESSAGE(STATUS "Found glew ${libglew}")
MESSAGE(STATUS "Found OpenGL ${OPENGL_LIBRARY}")
MESSAGE(STATUS "Found GLEW ${GLEW_LIBRARIES}")
MESSAGE(STATUS "Found GL ${libgl}")
MESSAGE(STATUS "Found OMPL ${OMPL_LIBRARIES}")
MESSAGE(STATUS "Found kdl ${libkdl}")
MESSAGE(STATUS "Found Eigen3 include ${EIGEN3_INCLUDE_DIR}")
MESSAGE(STATUS "Found PSOPT ${libpsopt}")
MESSAGE(STATUS "IPOPT_INCLUDEDIR ${IPOPT_INCLUDEDIR}")
MESSAGE(STATUS "IPOPT_LIBRARIES ${IPOPT_LIBRARIES}")
MESSAGE(STATUS "ADOLC_LIBRARIES ${ADOLC_LIBRARIES}")
# fix bug in IpSmartPtr.hpp
add_definitions(-DHAVE_CSTDDEF)


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
SET(INTERFACE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/interfaces")
SET(PHYSICS_MUJOCO_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/physics_mujoco/include")
#SET(FABRIK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/extern/ik-1.1/ik/include/public")

if (BUILD_PYTHON_BINDING)
    SET(PYBIND11_PYTHON_VERSION "3.8")
    find_package(Python ${PYBIND11_PYTHON_VERSION} REQUIRED COMPONENTS Interpreter Development)
    MESSAGE(STATUS "Python_SITELIB: ${Python_SITELIB}")
    add_subdirectory(extern/pybind11)
endif ()

include_directories(${LOCAL_INCLUDE})

#add_subdirectory(extern/ik-1.1)
add_subdirectory(extern/psopt)
add_subdirectory(physics_mujoco)
add_subdirectory(physics_ompl)
add_subdirectory(demo)

if (BUILD_PYTHON_BINDING)
    add_subdirectory(python)
endif ()

## INSTALL
if(SKBUILD)
    message(STATUS "The project is built using scikit-build")
    # Do not use targets when install kdl and mujoco
    # kdl and mujoco is not targets built.
    install(FILES  "extern/kdl-1.5.1/lib/liborocos-kdl.so.1.5.1"
            "extern/kdl-1.5.1/lib/liborocos-kdl.so.1.5"
            "extern/kdl-1.5.1/lib/liborocos-kdl.so"
            ${libmujoco} DESTINATION .)
    install(TARGETS physics_mujoco _physics_mujoco DESTINATION .)
endif()