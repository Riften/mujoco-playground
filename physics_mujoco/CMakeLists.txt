add_library(physics_mujoco
        src/mujoco_joint_group.cpp
        src/mujoco_joint_controller.cpp
        src/mujoco_kinematic_tree.cpp
        src/utils.cpp
        src/body_tree.cpp)

target_link_libraries(physics_mujoco ${libmujoco} ${OMPL_LIBRARIES} ${Log4cxx_LIBRARY} ${libkdl})
target_include_directories(physics_mujoco
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${INTERFACE_INCLUDE_DIR}
        PRIVATE
        ${MUJOCO_INCLUDE_DIR}
        ${OMPL_INCLUDE_DIRS}
        ${Log4cxx_INCLUDE_DIR}
        ${KDL_INCLUDE_DIR})
