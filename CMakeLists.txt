cmake_minimum_required(VERSION 3.5)
project(MobileRobot_Sandbox)

## Find catkin macros and libraries, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  controller_manager
)

## Declare ROS messages, services and actions ##

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


## catkin specific configuration ##
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES package_name
 CATKIN_DEPENDS roscpp rospy std_msgs controller_manager
#  DEPENDS system_lib
# CATKIN_DEPENDS
# message_runtime
)

## Build ##
## Specify additional locations of header files
include_directories(
  include # the path is <package_name>/include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable, the .cpp is what the executable program need
# add_executable(add_two_ints_server src/add_two_ints_server.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable, keep the program after the target will be built first
# add_dependencies(add_two_ints_server package_name_gencpp)




# cmake_minimum_required(VERSION 3.5)
# project(MobileRobot_Sandbox)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# install(
#   DIRECTORY config description launch worlds
#   DESTINATION share/${PROJECT_NAME}
# )

# ament_package()
