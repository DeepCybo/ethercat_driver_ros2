cmake_minimum_required(VERSION 3.8)
project(ethercat_interface)


# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)

# EtherLab
set(ETHERLAB_LOCAL_DIR /usr/local/etherlab)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
option(ETHERCAT_DPDK_USERMODE "Build against the IgH DPDK userspace master library" OFF)

find_path(ETHERCAT_INCLUDE_DIR NAMES ecrt.h PATHS ${ETHERLAB_LOCAL_DIR}/include)
if(ETHERCAT_DPDK_USERMODE)
  find_library(ETHERCAT_LIB NAMES ethercatd PATHS ${ETHERLAB_LOCAL_DIR}/lib)
  set(ETHERCAT_INTERFACE_COMPILE_DEFINITIONS
    ETHERCAT_INTERFACE_DPDK_USERMODE
    EC_USERMODE
    EC_ENABLE_DAEMON=0
  )
else()
  find_library(ETHERCAT_LIB NAMES ethercat PATHS ${ETHERLAB_LOCAL_DIR}/lib)
  set(ETHERCAT_INTERFACE_COMPILE_DEFINITIONS "")
endif()

if(NOT ETHERCAT_INCLUDE_DIR OR NOT ETHERCAT_LIB)
  message(FATAL_ERROR
    "Could not find the IgH EtherCAT master (ecrt.h / libethercat or libethercatd). "
    "Looked in standard system paths and in ${ETHERLAB_LOCAL_DIR}.")
endif()

message(STATUS "EtherCAT library: ${ETHERCAT_LIB}")
message(STATUS "EtherCAT DPDK userspace mode: ${ETHERCAT_DPDK_USERMODE}")

ament_export_include_directories(
  include
  ${ETHERCAT_INCLUDE_DIR}
)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/ec_master.cpp
  src/ec_pdo_channel_manager.cpp
  src/ec_pdo_single_interface_channel_manager.cpp
  src/ec_pdo_group_interface_channel_manager.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${ETHERCAT_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} ${ETHERCAT_LIB})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${ETHERCAT_INTERFACE_COMPILE_DEFINITIONS})

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  # find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  find_package(yaml_cpp_vendor REQUIRED)
  find_package(yaml-cpp REQUIRED)

  ament_lint_auto_find_test_dependencies()

  # Test PdoChannelManager
  ament_add_gmock(
    test_ec_pdo_channel_manager
    test/test_ec_pdo_channel_manager.cpp
  )
  target_include_directories(test_ec_pdo_channel_manager PRIVATE include ${ETHERCAT_INCLUDE_DIR})

  target_link_libraries(test_ec_pdo_channel_manager
    ${PROJECT_NAME}
    yaml-cpp)

  ament_add_gmock(
    test_ec_master_userspace_init
    test/test_ec_master_userspace_init.cpp
  )
  target_include_directories(test_ec_master_userspace_init PRIVATE include ${ETHERCAT_INCLUDE_DIR})
  target_link_libraries(test_ec_master_userspace_init
    ${PROJECT_NAME}
    yaml-cpp)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
  ${ETHERCAT_LIB}
)
ament_export_definitions(${ETHERCAT_INTERFACE_COMPILE_DEFINITIONS})
ament_export_dependencies(
  rclcpp
)
ament_package()
