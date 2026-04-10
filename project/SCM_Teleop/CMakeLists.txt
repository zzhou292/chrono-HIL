#=============================================================================
# CMake configuration file for the SCM Teleop project - deformable terrain
#   teleoperation demo with sensor visualization
#=============================================================================

#--------------------------------------------------------------
# Find the Chrono package with required and optional components
#--------------------------------------------------------------

# Invoke find_package in CONFIG mode
find_package(Chrono
             COMPONENTS Vehicle Irrlicht
             CONFIG
)

# If Chrono and/or the required component(s) were not found, return now.
if(NOT Chrono_FOUND)
  message("Could not find requirements for the SCM Teleop Project")
  return()
endif()

#--------------------------------------------------------------
# Temporal SCM Data Collection (time-series for temporal NN)
# NOTE: collect_scm_data, collect_scm_data_fast, collect_rate_data,
#       collect_vehicle_data, and proj_HIL_scm_teleop have been archived.
#--------------------------------------------------------------

message(STATUS "...add collect_temporal_data (temporal/time-series data collection)")

find_package(OpenMP QUIET)

add_executable(collect_temporal_data "data_collection/collect_temporal_data.cpp")
source_group("" FILES "data_collection/collect_temporal_data.cpp")

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

if(TARGET Chrono::Chrono_irrlicht)
    target_compile_definitions(collect_temporal_data PRIVATE CHRONO_IRRLICHT)
    target_link_libraries(collect_temporal_data PRIVATE Chrono::Chrono_irrlicht)
endif()

if(OpenMP_CXX_FOUND)
    target_compile_definitions(collect_temporal_data PRIVATE CHRONO_OPENMP)
    target_link_libraries(collect_temporal_data PRIVATE OpenMP::OpenMP_CXX)
endif()

