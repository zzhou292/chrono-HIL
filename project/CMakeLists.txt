option(ENABLE_PROJECTS "Build the Chrono_HIL project programs" OFF)

if(ENABLE_PROJECTS)
    message(STATUS "\n==== Chrono HIL projects ====")

    # 1. Check if Chrono::Sensor was actually found/built
    if(CHRONO_SENSOR_FOUND)
        # add_subdirectory(Teleop)
        #add_subdirectory(Iowa_Highway)
        #add_subdirectory(SanFran_City)
        #add_subdirectory(Ring_Exp)
        #add_subdirectory(Rom_Models)
        add_subdirectory(SCM_Teleop)

        # 2. Check for fastcdr using the Package Found variable or the Target.
        # Chrono's SynChrono module exports the fastcdr dependency.
        find_package(fastcdr QUIET)

        # We check both the variable and if the SynChrono module is in our library list
        if(fastcdr_FOUND OR TARGET fastcdr OR (";${CHRONO_LIBRARIES};" MATCHES ";Chrono::SynChrono;"))
            
            message(STATUS "Enabling HMMWV_MHIL (FastCDR/SynChrono dependency satisfied).")
            #add_subdirectory(HMMWV_MHIL)
            
        else()
            message(WARNING
                "Skipping HMMWV_MHIL: required dependency 'fastcdr' or 'SynChrono' not found.\n"
                "  CHRONO_LIBRARIES='${CHRONO_LIBRARIES}'\n"
                "  Tip: Ensure Chrono was built with ENABLE_MODULE_SYNCHRONO=ON and FastDDS is installed."
            )
        endif()

    else()
        message(WARNING
            "Skipping Chrono HIL projects that require Chrono::Sensor (including Teleop/HMMWV_MHIL).\n"
            "CHRONO_SENSOR_FOUND is OFF.\n"
            "Tip: Configure Chrono with sensor enabled and satisfy its dependencies (OptiX/OpenGL/etc.)."
        )
    endif()

else()
    message(STATUS "Chrono HIL projects are disabled (ENABLE_PROJECTS=OFF).")
endif()