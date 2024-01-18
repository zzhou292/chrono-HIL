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
    )
source_group("network" FILES ${NETWORK_FILES})

set(CXX_FLAGS ${CH_CXX_FLAGS})
set(LIBRARIES ${CHRONO_LIBRARIES})


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


set_target_properties(ChronoEngine_hil PROPERTIES
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}")

include_directories(${CHRONO_INCLUDE_DIRS} ${SDL2_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${IRRKLANG_INSTALL_DIR})


target_compile_definitions(ChronoEngine_hil PRIVATE "CH_API_COMPILE_HIL")
target_compile_definitions(ChronoEngine_hil PRIVATE "CH_IGNORE_DEPRECATED")


target_link_libraries(ChronoEngine_hil ${LIBRARIES} ${SDL2_LIBRARIES} ${IRRKLANG_LIBRARY})

install(TARGETS ChronoEngine_hil
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_hil
        FILES_MATCHING PATTERN "*.h")
install(FILES ChApiModels.h DESTINATION include/chrono_hil)



