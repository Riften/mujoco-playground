add_library(mujoco_render src/mujoco_render.cpp)

target_link_libraries(mujoco_render ${libmujoco} ${libglfw} ${GLEW_LIBRARIES} OpenGL::GL)
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