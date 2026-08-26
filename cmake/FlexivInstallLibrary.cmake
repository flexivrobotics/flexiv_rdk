# This macro will install ${PROJECT_NAME} to ${CMAKE_INSTALL_PREFIX} when running make install
#
# FlexivInstallLibrary() will install all subfolders of ${CMAKE_CURRENT_SOURCE_DIR}/include
# FlexivInstallLibrary(install_directories) will install only the specified install_directories
#
# Requirements:
# 1. project structure should resemble:
#    project
#     - README.md
#     - CMakeLists.txt that calls this macro
#     - cmake/${PROJECT_NAME}-config.cmake.in
#     - include/subfolder/*.h or *.hpp
# 2. build the library using cmake target functions
#    - add_library(${PROJECT_NAME} ...) before calling this macro
#    - target_include_directories(${PROJECT_NAME} ...)
#    - target_link_libraries(${PROJECT_NAME} ...)
#    - target_compile_features(${PROJECT_NAME} ...)
#    - target_compile_options(${PROJECT_NAME} ...)
#
# Installed files:
# - include/subfolder/*.h or *.hpp
# - lib/lib{PROJECT_NAME}
# - lib/cmake/{PROJECT_NAME}/

macro(FlexivInstallLibrary)
    # copy the executables and libraries to the CMAKE_INSTALL_PREFIX DIRECTORY
    # GNUInstallDirs will set CMAKE_INSTALL* to be the standard relative paths
    include(GNUInstallDirs)
    install(TARGETS ${PROJECT_NAME}
        EXPORT "${PROJECT_NAME}-targets"
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

    if(${ARGC} EQUAL 0)
        # install all subfolders of ${CMAKE_CURRENT_SOURCE_DIR}/include
        file(GLOB install_directories ${CMAKE_CURRENT_SOURCE_DIR}/include/*)
        foreach(install_directory ${install_directories})
            if(IS_DIRECTORY ${install_directory})
                install(DIRECTORY ${install_directory}
                        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
                        FILES_MATCHING 
                        PATTERN "*.h"
                        PATTERN "*.hpp"
                )
            endif()
        endforeach()
    elseif(${ARGC} EQUAL 1)
        # install specified directories only
        foreach(install_directory ${ARGV0})
            install(DIRECTORY ${install_directory}
                    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
                    FILES_MATCHING 
                    PATTERN "*.h"
                    PATTERN "*.hpp"
                    )
        endforeach()
    else()
        message(FATAL_ERROR "FlexivInstallLibrary take 0 or 1 argument, but given ${ARGC}")
    endif()

    # Create a *config-version.cmake file so that find_package can have a version specified
    include(CMakePackageConfigHelpers)
    write_basic_package_version_file(
        "${PROJECT_NAME}-config-version.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
        )

    # Copy the *-targets.cmake file to the CMAKE_INSTALL_PREFIX directory
    install(EXPORT "${PROJECT_NAME}-targets"
            FILE "${PROJECT_NAME}-targets.cmake"
            NAMESPACE "flexiv::"
            DESTINATION "lib/cmake/${PROJECT_NAME}"
            )

    # Copy the *.-config file to the CMAKE_INSTALL_PREFIX directory. This will specify the dependencies.
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake.in" "${PROJECT_NAME}-config.cmake" @ONLY)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
                  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
            DESTINATION "lib/cmake/${PROJECT_NAME}"
            )

    # Replace the dummy library built above with the actual prebuilt library that was downloaded.
    if(RDK_STATIC_PACKAGING)
        # Single static archive.
        set(_rdk_installed_lib
            "${CMAKE_STATIC_LIBRARY_PREFIX}${PROJECT_NAME}${CMAKE_STATIC_LIBRARY_SUFFIX}")
        install(CODE
                "file(REMOVE ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/${_rdk_installed_lib})")
        install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${RDK_LIB}
                DESTINATION ${CMAKE_INSTALL_LIBDIR}
                RENAME ${_rdk_installed_lib}
                )
    elseif(WIN32)
        # A DLL target produces two files: the import library (.lib, link time, installed to lib/)
        # and the runtime DLL (.dll, installed to bin/). Replace both with the downloaded artifacts.
        set(_rdk_import_lib
            "${CMAKE_IMPORT_LIBRARY_PREFIX}${PROJECT_NAME}${CMAKE_IMPORT_LIBRARY_SUFFIX}")
        set(_rdk_runtime_lib
            "${CMAKE_SHARED_LIBRARY_PREFIX}${PROJECT_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}")
        install(CODE
                "file(REMOVE ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/${_rdk_import_lib})")
        install(CODE
                "file(REMOVE ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/${_rdk_runtime_lib})")
        install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${RDK_LIB}
                DESTINATION ${CMAKE_INSTALL_LIBDIR}
                RENAME ${_rdk_import_lib})
        install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${RDK_RUNTIME_LIB}
                DESTINATION ${CMAKE_INSTALL_BINDIR}
                RENAME ${_rdk_runtime_lib})
    else()
        # Single shared-library artifact (.so on Linux, .dylib on macOS).
        set(_rdk_installed_lib
            "${CMAKE_SHARED_LIBRARY_PREFIX}${PROJECT_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}")
        install(CODE
                "file(REMOVE ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/${_rdk_installed_lib})")
        install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${RDK_LIB}
                DESTINATION ${CMAKE_INSTALL_LIBDIR}
                RENAME ${_rdk_installed_lib}
                )
    endif()

    # Use the CPack Package Generator
    set(CPACK_PACKAGE_VENDOR "Flexiv")
    set(CPACK_PACKAGE_CONTACT "support@flexiv.com")
    set(CPACK_PACKAGE_DESCRIPTION "Flexiv RDK (Robotic Development Kit)")
    set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
    set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
    set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
    set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
    set(CPACK_RESOURCE_FILE_README  "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
    include(CPack)
endmacro()