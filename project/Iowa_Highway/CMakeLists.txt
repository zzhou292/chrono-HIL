#=============================================================================
# CMake configuration file for the SynChrono Highway project - an extended
#   demo of a highway environment, interfaced to a human-driven simulator
#=============================================================================

#--------------------------------------------------------------
# List of all executables
#--------------------------------------------------------------

set(DEMOS
    proj_HIL_highway
    proj_HIL_highway_2
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

# If Chrono and/or the required component(s) were not found, return now.
if(NOT Chrono_FOUND)
  message("Could not find requirements for the SynChrono Highway Project")
  return()
endif()

#--------------------------------------------------------------
# Include paths and libraries
#--------------------------------------------------------------

message(STATUS "Projects for SynChrono Highway...")

foreach(PROGRAM ${DEMOS})

  message(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM} "${PROGRAM}.cpp")
  source_group("" FILES "${PROGRAM}.cpp")

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
          Chrono::Chrono_synchrono
          Chrono::Chrono_sensor
          Chrono::Chrono_irrlicht
          SDL2::SDL2
  )

endforeach(PROGRAM)
