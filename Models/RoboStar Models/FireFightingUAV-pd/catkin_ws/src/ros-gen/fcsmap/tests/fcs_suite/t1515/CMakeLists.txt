cmake_minimum_required(VERSION 3.0.2)
project(fcsmap_fcs_suite_t1515)
add_compile_options(-std=c++17 $<$<CONFIG:DEBUG>:-DDEBUG>)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    fcs_interface
    fcsmap_fcs_suite
)

catkin_package(CATKIN_DEPENDS
    roscpp
    fcs_interface
    fcsmap_fcs_suite
)

include_directories(
    ../../../fcs_suite/include
    ../../../fcs_suite/usr
    ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node
	src/main.cpp
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

add_dependencies(${PROJECT_NAME}_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    fcsmap_fcs_suite_node
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    boost_program_options
    fcsmap_fcs_suite_node
)
