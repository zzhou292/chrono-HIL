#=============================================================================
# CMake configuration file for the SynChrono Highway project - an extended
#   demo of a highway environment, interfaced to a human-driven simulator
#=============================================================================

#--------------------------------------------------------------
# List of all executables
#--------------------------------------------------------------

set(DEMOS
	proj_HIL_highway
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

include_directories(
    ${CHRONO_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
    ${SDL2_INCLUDE_DIRS}
)

set(EXT_LIBRARIES 
	ChronoEngine_hil
)



#--------------------------------------------------------------
# Append to the parent's list of DLLs
#--------------------------------------------------------------

list(APPEND ALL_DLLS "${CHRONO_DLLS}")
set(ALL_DLLS "${ALL_DLLS}" PARENT_SCOPE)

#--------------------------------------------------------------
# Compilation flags
#--------------------------------------------------------------

set(COMPILE_FLAGS ${CHRONO_CXX_FLAGS})

# Disable some warnings triggered by Irrlicht (Windows only)
#if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
#    SET(COMPILE_FLAGS "${COMPILE_FLAGS} /wd4275")
#endif()

#--------------------------------------------------------------
# Loop over all demo programs and build them
#--------------------------------------------------------------

message(STATUS "Projects for SynChrono Highway...")

foreach(PROGRAM ${DEMOS})

  message(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM}  "${PROGRAM}.cpp")
  source_group(""  FILES "${PROGRAM}.cpp")

  target_compile_definitions(${PROGRAM} PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
  target_compile_definitions(${PROGRAM} PUBLIC "PROJECTS_DATA_DIR=\"${PROJECTS_DATA_DIR}\"") 
  target_compile_options(${PROGRAM} PUBLIC ${CHRONO_CXX_FLAGS})
  target_link_options(${PROGRAM} PUBLIC ${CH_LINKERFLAG_SHARED})

	target_link_libraries(${PROGRAM} ${EXT_LIBRARIES} ${CHRONO_LIBRARIES} "-L/usr/local/cuda/lib64")# -lcudart")

endforeach(PROGRAM)
