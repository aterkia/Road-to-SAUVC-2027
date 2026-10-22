cmake_minimum_required(VERSION 3.16)
project(bootstrap)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS ../../scripts/record_session.py ../../scripts/replay_perception.py
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Give each ROS test group a fresh Python/DDS process; groups share one isolated domain.
  file(GLOB system_test_files CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/../../tests/test_*.py")
  foreach(test_file IN LISTS system_test_files)
    get_filename_component(test_name "${test_file}" NAME_WE)
    ament_add_pytest_test(${test_name} "${test_file}"
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../.."
      TIMEOUT 180)
    set_tests_properties(${test_name} PROPERTIES RUN_SERIAL TRUE)
  endforeach()
endif()
ament_package()
