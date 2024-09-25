function(generate_robot)
    cmake_parse_arguments(ARG "" "DIR" "" ${ARGN})
    set(banned_ext "md;pdf;png")
    set(banned_dir "scripts;launch")

    # Gen files to put in the share directory
    if (ARG_DIR)
        message("   -> Installing robot in directory `${ARG_DIR}`")
        file(GLOB children LIST_DIRECTORIES true "${ARG_DIR}/**")
        foreach(child ${children})
            if (IS_DIRECTORY ${child})
                # Check for banned extensions
                set(VALID_EXT ON)
                foreach(dir ${banned_dir})
                    if (${child} MATCHES ".${dir}$")
                        set(VALID_EXT OFF)
                    endif()
                endforeach()

                if (VALID_EXT)
                    message("      -> Installing ${child}")
                    install(DIRECTORY ${child} DESTINATION "share/${PROJECT_NAME}/${ARG_DIR}")
                endif()
            else()
                # Check for banned extensions
                set(VALID_EXT ON)
                foreach(ext ${banned_ext})
                    if (${child} MATCHES ".${ext}$")
                        set(VALID_EXT OFF)
                    endif()
                endforeach()
                
                if (VALID_EXT)
                    message("      -> Installing ${child}")
                    install(FILES ${child} DESTINATION "share/${PROJECT_NAME}/${ARG_DIR}")
                endif()
            endif()
        endforeach()
    else()
        message(SEND_ERROR "The given directory ${ARG_DIR} doesn't exist")
    endif()

    # Gen scripts
    if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/${ARG_DIR}/scripts")
        message("      -> Installing scripts for the robot")
        file(GLOB_RECURSE children LIST_DIRECTORIES false "${ARG_DIR}/scripts/**")
        foreach(child ${children})
            message("        -> Installing script ${child}")
            install(
                PROGRAMS ${child}
                DESTINATION "lib/${PROJECT_NAME}/${ARG_DIR}"
            )
        endforeach()
    endif()

    # Gen launch files
    if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/${ARG_DIR}/launch")
        message("      -> Installing `launch` directory")
        install(
            DIRECTORY "${ARG_DIR}/launch"
            DESTINATION "share/${PROJECT_NAME}/${ARG_DIR}"
        )
    endif()

endfunction()