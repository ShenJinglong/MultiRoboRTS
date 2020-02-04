project(jps_planner)
set(CMAKE_BUILD_TYPE Release)

include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
    SHARED
    jps_planner.cpp
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    roborts_costmap
)

add_library(planning::global_planner::${PROJECT_NAME} ALIAS ${PROJECT_NAME})