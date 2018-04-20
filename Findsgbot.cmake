# - Try to find libsgbot
# Once done this will define
#  sgbot_FOUND - System has libsgbot
#  sgbot_INCLUDE_DIRS - The libsgbot include directories
#  sgbot_LIBRARIES - The libraries needed to use libsgbot
#  sgbot_DEFINITIONS - Compiler switches required for using libsgbot

find_path(SGBOT_INCLUDE_DIR sgbot.h
          HINTS
          /usr/include/libsgbot
          /usr/local/include/libsgbot
          /home/cybernik/workspace/libsgbot/include
          #PATH_SUFFIXES libsgbot
         )

find_library(SGBOT_LIBRARIE NAMES sgbot libsgbot
             HINTS
             /lib
             /usr/lib
             /usr/local/lib
             /home/cybernik/workspace/libsgbot/build/src
            )

set(sgbot_INCLUDE_DIRS ${SGBOT_INCLUDE_DIR})
set(sgbot_LIBRARIES ${SGBOT_LIBRARIE})

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set sgbot_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(sgbot DEFAULT_MSG
                                  sgbot_LIBRARIES sgbot_INCLUDE_DIRS)

mark_as_advanced(sgbot_INCLUDE_DIRS sgbot_LIBRARIES)
