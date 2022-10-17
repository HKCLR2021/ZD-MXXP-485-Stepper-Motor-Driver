cmake_minimum_required(VERSION 3.5)
project(heatchamber_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

###########
## Build ##
###########
add_executable(test_rtu src/test_rtu.cpp ${sdk_common})
target_link_libraries(test_rtu ${PROJECT_NAME} )

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 src
)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED   
   src/modbus.c  
   src/modbus-data.c  
   src/modbus-rtu.c  
   src/modbus-tcp.c  
   src/zdxxmp.c
)

#############
## Install ##
#############

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(TARGETS test_rtu
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
