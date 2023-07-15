add_executable(
    pathfinder_test 
    src/pathfinder_test.cpp
)

target_link_libraries(
    pathfinder_test
    PRIVATE
        astar
)

add_test(
    NAME pathfinder_test
    COMMAND pathfinder_test
)