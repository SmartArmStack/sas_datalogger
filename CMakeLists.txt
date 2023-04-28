cmake_minimum_required(VERSION 3.8)
project(sas_datalogger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sas_core REQUIRED)
find_package(sas_common REQUIRED)
find_package(sas_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

##### CPP LIBRARY #####

add_library(${PROJECT_NAME} SHARED
    src/sas_datalogger_client.cpp
    )

ament_target_dependencies(${PROJECT_NAME} Eigen3 rclcpp sas_core sas_common sas_msgs)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 rclcpp sas_core sas_common sas_msgs)

target_link_libraries(
    ${PROJECT_NAME}
    )

install(
    DIRECTORY include/
    DESTINATION include
    )

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    #ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    )

##END## CPP LIBRARY #####

##### CPP BINARY #####

add_executable(sas_datalogger_client_example
    src/examples/sas_datalogger_client_example.cpp)

target_link_libraries(sas_datalogger_client_example
    ${PROJECT_NAME}
    Eigen3::Eigen
    )

install(TARGETS
  sas_datalogger_client_example
  DESTINATION lib/${PROJECT_NAME})

##END## CPP BINARY #####

##### PYTHON EXECUTABLES #####
install(PROGRAMS
  scripts/sas_datalogger_node.py
  DESTINATION lib/${PROJECT_NAME}
)
##END## PYTHON EXECUTABLES #####

##### LAUNCH FILES #####

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

##END## LAUNCH FILES #####


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
