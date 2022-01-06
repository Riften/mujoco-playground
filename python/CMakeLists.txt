pybind11_add_module(_physics_mujoco
        src/mujoco_joint_group_py.cpp)

target_include_directories(_physics_mujoco
        PRIVATE
        ${PHYSICS_MUJOCO_INCLUDE_DIR}
        ${MUJOCO_INCLUDE_DIR}
        ${OMPL_INCLUDE_DIRS}
        ${Log4cxx_INCLUDE_DIR}
        ${KDL_INCLUDE_DIR}
        ${MUJOCO_RENDER_INCLUDE_DIR})

target_link_libraries(_physics_mujoco PRIVATE physics_mujoco mujoco_render)
