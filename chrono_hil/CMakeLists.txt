message(STATUS "==== Chrono::HIL Module ====")

set(DRIVER_FILES
    #driver/ChCSLDriver.h
    #driver/ChCSLDriver.cpp
    driver/ChNSF_Drivers.h
    driver/ChNSF_Drivers.cpp
    driver/ChSDLInterface.h
    driver/ChSDLInterface.cpp
    driver/ChIDM_Follower.h
    driver/ChIDM_Follower.cpp
    #driver/joystick.h
)

if(NOT DEFINED ENABLE_MODULE_SENSOR)
    set(ENABLE_MODULE_SENSOR ${Chrono_SENSOR_AVAILABLE})
endif()

if(ENABLE_MODULE_SENSOR)
    set(DRIVER_FILES ${DRIVER_FILES}
    driver/ChLidarWaypointDriver.h
    driver/ChLidarWaypointDriver.cpp
    )  
endif()

source_group("driver" FILES ${DRIVER_FILES})

set(TIMER_FILES
    timer/ChRealtimeCumulative.h
)
source_group("timer" FILES ${TIMER_FILES})

set(SOUND_FILES
    sound/ChCSLSoundEngine.h
)
source_group("sound" FILES ${SOUND_FILES})

set(ROM_FILES
    ROM/veh/rom_Eightdof.h
    ROM/veh/rom_Eightdof.cpp
    ROM/veh/rom_TMeasy.h
    ROM/veh/rom_TMeasy.cpp
    ROM/veh/rom_utils.h
    ROM/veh/rom_utils.cpp
    ROM/veh/Ch_8DOF_vehicle.h
    ROM/veh/Ch_8DOF_vehicle.cpp


    ROM/driver/ChROM_PathFollowerDriver.h
    ROM/driver/ChROM_PathFollowerDriver.cpp
    ROM/driver/ChROM_IDMFollower.h
    ROM/driver/ChROM_IDMFollower.cpp

    ROM/syn/Ch_8DOF_zombie.h
    ROM/syn/Ch_8DOF_zombie.cpp


    )
source_group("rom" FILES ${ROM_FILES})

set(NETWORK_FILES
    network/udp/ChBoostInStreamer.h
    network/udp/ChBoostInStreamer.cpp
    network/udp/ChBoostOutStreamer.h
    network/udp/ChBoostOutStreamer.cpp

    network/tcp/ChTCPClient.h
    network/tcp/ChTCPClient.cpp
    network/tcp/ChTCPServer.h
    network/tcp/ChTCPServer.cpp

    network/sim/ChDelaySim.h
    network/sim/ChDelaySim.cpp
    )

# Camera delay simulation requires sensor module (GLFW/OpenGL)
if(ENABLE_MODULE_SENSOR)
    set(NETWORK_FILES ${NETWORK_FILES}
        network/sim/ChCameraDelaySim.h
        network/sim/ChCameraDelaySim.cpp
        network/sim/ChCameraDelaySimThreaded.h
        network/sim/ChCameraDelaySimThreaded.cpp
    )
endif()

source_group("network" FILES ${NETWORK_FILES})

set(CXX_FLAGS ${CH_CXX_FLAGS})

set(LIBRARIES "")
if(CHRONO_LIBRARIES)
    list(APPEND LIBRARIES ${CHRONO_LIBRARIES})
elseif(DEFINED CHRONO_TARGETS AND CHRONO_TARGETS)
    list(APPEND LIBRARIES ${CHRONO_TARGETS})
endif()

if(NOT LIBRARIES)
    message(FATAL_ERROR "Chrono libraries were not provided by the Chrono package; check the requested Chrono components.")
endif()


add_library(ChronoEngine_hil SHARED 
            ${DRIVER_FILES}
            ${TIMER_FILES}
            ${SOUND_FILES}
            ${ROM_FILES}
            ${NETWORK_FILES}
)

find_package(SDL2 REQUIRED)

list(APPEND ALL_DLLS "${CHRONO_DLLS}" "${SDL2_DLL}")
set(ALL_DLLS "${ALL_DLLS}" PARENT_SCOPE)

find_package(Boost REQUIRED)

set(_chrono_include_dirs "${CHRONO_INCLUDE_DIRS}")
if(NOT _chrono_include_dirs AND TARGET Chrono::Chrono_core)
    get_target_property(_chrono_core_includes Chrono::Chrono_core INTERFACE_INCLUDE_DIRECTORIES)
    if(_chrono_core_includes)
        set(_chrono_include_dirs "${_chrono_core_includes}")
    endif()
endif()

find_path(HACD_INCLUDE_DIR
    NAMES hacdVersion.h
    HINTS ${_chrono_include_dirs}
    PATH_SUFFIXES chrono_thirdparty/HACD
)
if(NOT HACD_INCLUDE_DIR)
    message(WARNING "HACD headers (hacdVersion.h) not found. Building chrono_hil without HACD support.")
    set(HACD_INCLUDE_DIR "")  # dummy so it's defined
endif()

find_package(Eigen3 3.3 REQUIRED)


set_target_properties(ChronoEngine_hil PROPERTIES
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}")

target_compile_definitions(ChronoEngine_hil PRIVATE "CH_API_COMPILE_HIL")
target_compile_definitions(ChronoEngine_hil PRIVATE "CH_IGNORE_DEPRECATED")

target_include_directories(ChronoEngine_hil
    PUBLIC
        ${_chrono_include_dirs}
        ${Boost_INCLUDE_DIRS}
        ${HACD_INCLUDE_DIR}
    PRIVATE
        ${SDL2_INCLUDE_DIRS}
        ${IRRKLANG_INSTALL_DIR}
)

if(TARGET Eigen3::Eigen)
    target_link_libraries(ChronoEngine_hil PUBLIC Eigen3::Eigen)
elseif(DEFINED EIGEN3_INCLUDE_DIR)
    target_include_directories(ChronoEngine_hil PUBLIC ${EIGEN3_INCLUDE_DIR})
else()
    message(FATAL_ERROR "Eigen3 was found but the include directory was not provided.")
endif()

target_link_libraries(ChronoEngine_hil
    PUBLIC
        ${LIBRARIES}
    PRIVATE
        ${SDL2_LIBRARIES}
        ${IRRKLANG_LIBRARY}
)

# Link GLFW and OpenGL for camera delay simulation when sensor is available
if(ENABLE_MODULE_SENSOR)
    find_package(OpenGL REQUIRED)
    find_package(glfw3 QUIET)
    find_package(GLEW QUIET)
    
    if(OpenGL_FOUND)
        target_link_libraries(ChronoEngine_hil PRIVATE OpenGL::GL)
    endif()
    if(glfw3_FOUND)
        target_link_libraries(ChronoEngine_hil PRIVATE glfw)
    endif()
    if(GLEW_FOUND)
        target_link_libraries(ChronoEngine_hil PRIVATE GLEW::GLEW)
    endif()
endif()

install(TARGETS ChronoEngine_hil
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_hil
        FILES_MATCHING PATTERN "*.h")
install(FILES ChApiModels.h DESTINATION include/chrono_hil)
