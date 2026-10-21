cmake_minimum_required(VERSION 3.16)
project(bootstrap)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS ../../scripts/record_session.py ../../scripts/replay_perception.py
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(system_tests "${CMAKE_CURRENT_SOURCE_DIR}/../../tests"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../.."
    TIMEOUT 240)
endif()
ament_package()
