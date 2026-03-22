#=============================================================================
# CMake configuration file for the SCM Teleop project - deformable terrain
#   teleoperation demo with sensor visualization
#=============================================================================

#--------------------------------------------------------------
# List of all executables
#--------------------------------------------------------------

set(DEMOS
    proj_HIL_scm_teleop
)

#--------------------------------------------------------------
# Find the Chrono package with required and optional components
#--------------------------------------------------------------

# Invoke find_package in CONFIG mode
find_package(Chrono
             COMPONENTS SynChrono Vehicle Sensor Irrlicht
             CONFIG
)
find_package(SDL2 REQUIRED)
find_package(rclcpp QUIET)
find_package(geometry_msgs QUIET)
find_package(teleop_bridge_msgs QUIET)

# If Chrono and/or the required component(s) were not found, return now.
if(NOT Chrono_FOUND)
  message("Could not find requirements for the SCM Teleop Project")
  return()
endif()

if(rclcpp_FOUND AND geometry_msgs_FOUND AND teleop_bridge_msgs_FOUND)
  set(HAVE_ROS2_BRIDGE ON)
else()
  set(HAVE_ROS2_BRIDGE OFF)
endif()

#--------------------------------------------------------------
# Include paths and libraries
#--------------------------------------------------------------

message(STATUS "Projects for SCM Teleop...")

foreach(PROGRAM ${DEMOS})

  message(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM} "cpp_hil/${PROGRAM}.cpp")
  source_group("" FILES "cpp_hil/${PROGRAM}.cpp")

  target_compile_definitions(${PROGRAM}
      PUBLIC
          "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
          "PROJECTS_DATA_DIR=\"${PROJECTS_DATA_DIR}\""
  )
  target_compile_options(${PROGRAM} PUBLIC ${CHRONO_CXX_FLAGS})
  target_link_options(${PROGRAM} PUBLIC ${CH_LINKERFLAG_SHARED})

  target_include_directories(${PROGRAM} PRIVATE ${CMAKE_SOURCE_DIR})

  target_link_libraries(${PROGRAM}
      PRIVATE
          ChronoEngine_hil
          Chrono::Chrono_vehicle
          Chrono::ChronoModels_vehicle
          Chrono::Chrono_synchrono
          Chrono::Chrono_sensor
          Chrono::Chrono_irrlicht
          SDL2::SDL2
  )

  if(PROGRAM STREQUAL "proj_HIL_scm_teleop")
    if(HAVE_ROS2_BRIDGE)
      target_sources(${PROGRAM} PRIVATE cpp_hil/Ros2Bridge.cpp)
      target_compile_definitions(${PROGRAM} PRIVATE ENABLE_ROS2_BRIDGE)
      target_link_libraries(${PROGRAM}
        PRIVATE
          rclcpp::rclcpp
          teleop_bridge_msgs::teleop_bridge_msgs__rosidl_generator_cpp
          teleop_bridge_msgs::teleop_bridge_msgs__rosidl_typesupport_cpp
      )
    else()
      message(STATUS "ROS2 bridge disabled for ${PROGRAM} (missing ROS2 dependencies)")
    endif()
  endif()

endforeach(PROGRAM)

#--------------------------------------------------------------
# SCM Data Collection Tool (C++ with ChTireTestRig)
#--------------------------------------------------------------

message(STATUS "...add collect_scm_data (C++ data collection tool)")

add_executable(collect_scm_data "cpp_collect/collect_scm_data.cpp")
source_group("" FILES "cpp_collect/collect_scm_data.cpp")

target_compile_definitions(collect_scm_data
    PUBLIC
        "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
)
target_compile_options(collect_scm_data PUBLIC ${CHRONO_CXX_FLAGS})
target_link_options(collect_scm_data PUBLIC ${CH_LINKERFLAG_SHARED})

target_link_libraries(collect_scm_data
    PRIVATE
        Chrono::Chrono_vehicle
        Chrono::ChronoModels_vehicle
        Chrono::Chrono_irrlicht
)

#--------------------------------------------------------------
# Fast SCM Data Collection Tool (with OpenMP parallelization)
#--------------------------------------------------------------

message(STATUS "...add collect_scm_data_fast (optimized C++ data collection)")

find_package(OpenMP QUIET)

add_executable(collect_scm_data_fast "cpp_collect/collect_scm_data_fast.cpp")
source_group("" FILES "cpp_collect/collect_scm_data_fast.cpp")

target_compile_definitions(collect_scm_data_fast
    PUBLIC
        "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
)
target_compile_options(collect_scm_data_fast PUBLIC ${CHRONO_CXX_FLAGS})
target_link_options(collect_scm_data_fast PUBLIC ${CH_LINKERFLAG_SHARED})

target_link_libraries(collect_scm_data_fast
    PRIVATE
        Chrono::Chrono_vehicle
        Chrono::ChronoModels_vehicle
)

# Add OpenMP if available
if(OpenMP_CXX_FOUND)
    target_compile_definitions(collect_scm_data_fast PRIVATE CHRONO_OPENMP)
    target_link_libraries(collect_scm_data_fast PRIVATE OpenMP::OpenMP_CXX)
    message(STATUS "    OpenMP enabled for parallel data collection")
else()
    message(STATUS "    OpenMP not found - single-threaded only")
endif()

#--------------------------------------------------------------
# Temporal SCM Data Collection (time-series for temporal NN)
#--------------------------------------------------------------

message(STATUS "...add collect_temporal_data (temporal/time-series data collection)")

add_executable(collect_temporal_data "cpp_collect/collect_temporal_data.cpp")
source_group("" FILES "cpp_collect/collect_temporal_data.cpp")

target_compile_definitions(collect_temporal_data
    PUBLIC
        "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
)
target_compile_options(collect_temporal_data PUBLIC ${CHRONO_CXX_FLAGS})
target_link_options(collect_temporal_data PUBLIC ${CH_LINKERFLAG_SHARED})

target_link_libraries(collect_temporal_data
    PRIVATE
        Chrono::Chrono_vehicle
        Chrono::ChronoModels_vehicle
)

if(OpenMP_CXX_FOUND)
    target_compile_definitions(collect_temporal_data PRIVATE CHRONO_OPENMP)
    target_link_libraries(collect_temporal_data PRIVATE OpenMP::OpenMP_CXX)
endif()

#--------------------------------------------------------------
# Full Vehicle SCM Data Collection (using HMMWV_Full)
#--------------------------------------------------------------

message(STATUS "...add collect_vehicle_data (full vehicle data collection)")

add_executable(collect_vehicle_data "cpp_collect/collect_vehicle_data.cpp")
source_group("" FILES "cpp_collect/collect_vehicle_data.cpp")

target_compile_definitions(collect_vehicle_data
    PUBLIC
        "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
)
target_compile_options(collect_vehicle_data PUBLIC ${CHRONO_CXX_FLAGS})
target_link_options(collect_vehicle_data PUBLIC ${CH_LINKERFLAG_SHARED})

target_link_libraries(collect_vehicle_data
    PRIVATE
        Chrono::Chrono_vehicle
        Chrono::ChronoModels_vehicle
)

# Add OpenMP if available
if(OpenMP_CXX_FOUND)
    target_compile_definitions(collect_vehicle_data PRIVATE CHRONO_OPENMP)
    target_link_libraries(collect_vehicle_data PRIVATE OpenMP::OpenMP_CXX)
    message(STATUS "    OpenMP enabled for vehicle data collection")
else()
    message(STATUS "    OpenMP not found - single-threaded only")
endif()
