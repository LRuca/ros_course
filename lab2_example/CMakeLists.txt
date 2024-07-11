cmake_minimum_required(VERSION 2.8.3)
project(exercise2)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  plan_env
  message_generation
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(exercise2_node
  src/exercise2.cpp 
  )
target_link_libraries(exercise2_node 
  ${catkin_LIBRARIES}
  )

  add_executable(exercise2_param_node
  src/exercise2_param.cpp 
  )
target_link_libraries(exercise2_param_node 
  ${catkin_LIBRARIES}
  )


