add_executable(demo demo.cpp)
target_link_libraries(demo ${libmujoco} ${libglfw} ${GLEW_LIBRARIES} OpenGL::GL)
target_include_directories(demo
        PRIVATE
        ${MUJOCO_INCLUDE_DIR}
        ${OPENGL_INCLUDE_DIR})

add_executable(basic basic.cpp)
target_link_libraries(basic ${libmujoco} ${libglfw} ${GLEW_LIBRARIES} OpenGL::GL)
target_include_directories(basic
        PRIVATE
        ${MUJOCO_INCLUDE_DIR}
        ${OPENGL_INCLUDE_DIR})