### Unit tests
#
#   Only configured when CATKIN_ENABLE_TESTING is true.

# These dependencies are only needed for unit testing
find_package(rostest REQUIRED)

# C++ gtests
catkin_add_gtest(${PROJECT_NAME}_test test.cpp)
target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})

