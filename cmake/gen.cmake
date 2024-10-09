macro(generate_robot)
    cmake_parse_arguments(ARG "" "DIR;DEF_EXEC_NAME" "CPPSRC" ${ARGN})
    set(banned_ext "md;pdf;png")
    set(banned_dir "scripts;launch;src;include;${ARG_CPPSRC}")

    if (NOT ARG_CPPSRC)
        set(ARG_CPPSRC "src")
    endif()

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
                DESTINATION "lib/${PROJECT_NAME}"
            )
        endforeach()
    endif()

    # C++ sources / headers
    foreach(cpp_src ${ARG_CPPSRC})
        if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/${ARG_DIR}/${cpp_src}")
            # Define executable name
            if ("${cpp_src}" STREQUAL "src")
                if (ARG_DEF_EXEC_NAME)
                    set(ex_name "${ARG_DEF_EXEC_NAME}")
                else()
                    set(ex_name "${ARG_DIR}")   
                endif()
            else()
                set(ex_name "${cpp_src}")
            endif()
            message("      -> Registering new executable `${ex_name}`")

            # Register executable target
            file(GLOB_RECURSE src_files LIST_DIRECTORIES false "${ARG_DIR}/${cpp_src}/**")
            ament_auto_add_executable("${ex_name}" ${src_files})
            set_target_properties("${ex_name}" PROPERTIES LINKER_LANGUAGE CXX)

            # Look for robot include directory
            if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/${ARG_DIR}/include")
                message("        -> Linking robot custom include directory")
                target_include_directories("${ex_name}" PRIVATE "${ARG_DIR}/include")
            endif()

            if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/include")
                message("        -> Linking project include directory")
                target_include_directories("${ex_name}" PRIVATE "include")
            endif()
            
        endif()
    endforeach()
    
    # Gen launch files
    if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/${ARG_DIR}/launch")
        message("      -> Installing `launch` directory")
        install(
            DIRECTORY "${ARG_DIR}/launch"
            DESTINATION "share/${PROJECT_NAME}/${ARG_DIR}"
        )
    endif()

endmacro()