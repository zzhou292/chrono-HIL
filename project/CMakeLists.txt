option(ENABLE_PROJECTS "Build the Chrono_HIL project programs" OFF)

if(ENABLE_PROJECTS)
  message(STATUS "\n==== Chrono HIL projects ====")

  add_subdirectory(NADS)

  if(CHRONO_SENSOR_FOUND)
    #add_subdirectory(Iowa_Highway)
    #add_subdirectory(SanFran_City)
    #add_subdirectory(Jsonfy_Env)
    #add_subdirectory(Ring_Exp)
    #add_subdirectory(Rom_Models)


      #---------------
    string(FIND "${CHRONO_LIBRARIES}" "fastcdr" matchres)
    if(NOT ${matchres} EQUAL -1)
      #add_subdirectory(HMMWV_MHIL)
    endif()
  endif()




  
endif()
