add_library(mujoco_render
        src/mujoco_render.cpp
        src/glfw_thread.cpp)

target_link_libraries(mujoco_render ${libmujoco} ${libglfw} ${libglew} OpenGL::GL)
set_property(TARGET mujoco_render PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(mujoco_render
        PUBLIC
        ${MUJOCO_INCLUDE_DIR}
        ${OPENGL_INCLUDE_DIR}
        #${PHYSICS_MUJOCO_INCLUDE_DIR}
        #${FABRIK_INCLUDE_DIR}
        ${KDL_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${OMPL_INCLUDE_DIRS}
        ${MUJOCO_RENDER_INCLUDE_DIR}
        PRIVATE
        ${Log4cxx_INCLUDE_DIR})