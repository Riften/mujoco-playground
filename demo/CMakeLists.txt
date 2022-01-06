add_executable(demo demo.cpp)
target_link_libraries(demo ${libmujoco} ${libglfw} ${libglew} OpenGL::GL physics_mujoco
        #ik
        ${libkdl} ${OMPL_LIBRARIES})
target_include_directories(demo
        PRIVATE
        ${MUJOCO_INCLUDE_DIR}
        ${OPENGL_INCLUDE_DIR}
        ${PHYSICS_MUJOCO_INCLUDE_DIR}
        #${FABRIK_INCLUDE_DIR}
        ${KDL_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${OMPL_INCLUDE_DIRS})

add_executable(demo_render demo_render.cpp)
target_link_libraries(demo_render ${libmujoco} ${libglfw} ${libglew} OpenGL::GL physics_mujoco mujoco_render
        #ik
        ${libkdl} ${OMPL_LIBRARIES})
target_include_directories(demo_render
        PRIVATE
        ${MUJOCO_INCLUDE_DIR}
        ${OPENGL_INCLUDE_DIR}
        ${PHYSICS_MUJOCO_INCLUDE_DIR}
        #${FABRIK_INCLUDE_DIR}
        ${KDL_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${OMPL_INCLUDE_DIRS}
        ${MUJOCO_RENDER_INCLUDE_DIR})

add_executable(demo_ur5e demo_ur5e.cpp)
target_link_libraries(demo_ur5e ${libmujoco} ${libglfw} ${libglew} OpenGL::GL physics_mujoco
        #ik
        ${libkdl} ${OMPL_LIBRARIES})
target_include_directories(demo_ur5e
        PRIVATE
        ${MUJOCO_INCLUDE_DIR}
        ${OPENGL_INCLUDE_DIR}
        ${PHYSICS_MUJOCO_INCLUDE_DIR}
        #${FABRIK_INCLUDE_DIR}
        ${KDL_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${OMPL_INCLUDE_DIRS})


add_executable(psopt_demo psopt_demo.cpp)
target_link_libraries(psopt_demo ${libmujoco} ${libglfw} ${libglew} OpenGL::GL physics_mujoco
        #ik
        ${libkdl} ${OMPL_LIBRARIES}
        # opt
        ${libpsopt}
        ${IPOPT_LIBRARIES}
        ${ADOLC_LIBRARIES})
target_include_directories(psopt_demo
        PRIVATE
        ${MUJOCO_INCLUDE_DIR}
        ${OPENGL_INCLUDE_DIR}
        ${PHYSICS_MUJOCO_INCLUDE_DIR}
        #${FABRIK_INCLUDE_DIR}
        ${KDL_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${OMPL_INCLUDE_DIRS}
        ${IPOPT_INCLUDEDIR})

add_executable(basic basic.cpp)
target_link_libraries(basic ${libmujoco} ${libglfw} ${libglew} OpenGL::GL)
target_include_directories(basic
        PRIVATE
        ${MUJOCO_INCLUDE_DIR}
        ${OPENGL_INCLUDE_DIR})

add_executable(derivative derivative.cpp)
target_link_libraries(derivative ${libmujoco} ${libglfw} ${libglew} OpenGL::GL)
target_include_directories(derivative
        PRIVATE
        ${MUJOCO_INCLUDE_DIR}
        ${OPENGL_INCLUDE_DIR})