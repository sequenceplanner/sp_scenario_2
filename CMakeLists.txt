cmake_minimum_required(VERSION 3.5)

project(sp_scenario_2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config meshes robots scenario_r1 scenario_r2 scenario_r3
 DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
