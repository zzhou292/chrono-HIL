option(ENABLE_TESTS "Build the Chrono_HIL test programs" OFF)

if(ENABLE_TESTS)
  message(STATUS "\n==== Chrono HIL tests ====")
  add_subdirectory(sdl_driver)
  add_subdirectory(networking)

  if(CHRONO_SENSOR_FOUND)
    add_subdirectory(Rom_Models)
    #add_subdirectory(ROM_parallel)
    #add_subdirectory(ROM_Benchmark)
  endif()
  
endif()
