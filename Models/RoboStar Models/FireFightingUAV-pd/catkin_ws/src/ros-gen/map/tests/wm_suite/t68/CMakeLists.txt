cmake_minimum_required(VERSION 3.0.2)
project(map_wm_suite_t68)
add_compile_options(-std=c++17 $<$<CONFIG:DEBUG>:-DDEBUG>)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    battery_monitor
    water_monitor
    map_wm_suite
)

catkin_package(CATKIN_DEPENDS
    roscpp
    battery_monitor
    water_monitor
    map_wm_suite
)

include_directories(
    ../../../wm_suite/include
    ../../../wm_suite/usr
    ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node
	src/main.cpp
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

add_dependencies(${PROJECT_NAME}_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    map_wm_suite_node
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    boost_program_options
    map_wm_suite_node
)
