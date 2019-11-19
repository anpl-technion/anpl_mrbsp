## ---------------------------------------------------------------------------
 #
 # Autonomous Navigation and Perception Lab (ANPL),
 # Technion, Israel Institute of Technology,
 # Faculty of Aerospace Engineering,
 # Haifa, Israel, 32000
 # All Rights Reserved
 #
 # See LICENSE for the license information
 #
 # -------------------------------------------------------------------------- */
##
 # @file: findIt.cmake
 # @brief: cmake macro which sets LIBNAME_INCLUDE_DIRS and LIBNAME_LIBRARIES vars by searching ${ANPL_PREFIX} directory
 # @brief: or by using a pkg-config module for CMake
 # @brief: ${ANPL_PREFIX} directory must be set prior to call of this macro
 # @author: Andrej Kitanov
 #
 ##


macro(findIt LIBNAME header)

    string(TOLOWER ${LIBNAME} libname)
        if(PKG_CONFIG_FOUND)
            pkg_check_modules(PC_${LIBNAME} ${libname})
            pkg_check_modules(PC_LIB${LIBNAME} lib${libname})
        endif()

        find_path(${LIBNAME}_INCLUDE_DIR ${libname}/${header}
                PATHS ${ANPL_PREFIX}/include ${PC_${LIBNAME}_INCLUDE_DIRS} ${PC_LIB${LIBNAME}_INCLUDE_DIRS} NO_DEFAULT_PATH)

        # Using find_library() even if pkg-config is available ensures that the full
        # path to the ${libname} library is available in ${LIBNAME}_LIBRARIES
        find_library(${LIBNAME}_LIBRARY ${libname}
                PATHS ${ANPL_PREFIX}/lib ${PC_${LIBNAME}_LIBRARY_DIRS} ${PC_LIB${LIBNAME}_LIBRARY_DIRS} NO_DEFAULT_PATH)

        # lib${libname} links to LibM on UNIX.
        #if(CYGWIN OR NOT WIN32)
        #    find_library(M_LIBRARY m)
        #endif()

        if(${LIBNAME}_INCLUDE_DIR AND ${LIBNAME}_LIBRARY)
            set(${LIBNAME}_INCLUDE_DIRS "${${LIBNAME}_INCLUDE_DIR}")
            set(${LIBNAME}_LIBRARIES "${${LIBNAME}_LIBRARY}" "${M_LIBRARY}")
            set(${LIBNAME}_FOUND ON)

            mark_as_advanced(${LIBNAME}_INCLUDE_DIR ${LIBNAME}_LIBRARY)
        endif()


    if(NOT ${LIBNAME}_FOUND)
        message(FATAL_ERROR "${LIBNAME} NOT FOUND")
    else()
        message("${LIBNAME} LIBRARY ${${LIBNAME}_LIBRARY} FOUND.")
    endif()
endmacro()
