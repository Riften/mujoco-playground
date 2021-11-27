add_library(physics_ompl
        src/state_validity_checker.cpp)

target_link_libraries(physics_ompl ${OMPL_LIBRARIES} ${Log4cxx_LIBRARY})
target_include_directories(physics_ompl
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${INTERFACE_INCLUDE_DIR}
        PRIVATE
        ${OMPL_INCLUDE_DIRS}
        ${Log4cxx_INCLUDE_DIR})