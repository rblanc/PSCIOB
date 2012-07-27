cmake_minimum_required (VERSION 2.6)
project (PSCIOB)

ENABLE_TESTING()

SET(PSCIOB_VERSION 0.8) 
SET(PSCIOB_MINOR_VERSION 0)

# rpath settings
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Setup build locations.
if(NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()


set(PSCIOB_ROOT_DIR ${CMAKE_SOURCE_DIR})
set(PSCIOB_LIB_DIR ${CMAKE_BINARY_DIR})



add_subdirectory(GeneralUtils)
add_subdirectory(PSCIOB)

mark_as_advanced(PSCIOB_DIR)
mark_as_advanced(GeneralUtils_DIR)

#reset the default directory for installation
if (NOT CMAKE_INSTALL_PREFIX_SET_ONCE)
	set(CMAKE_INSTALL_PREFIX_SET_ONCE true CACHE BOOL "Have we set the install prefix yet?" FORCE)
	mark_as_advanced(CMAKE_INSTALL_PREFIX_SET_ONCE)
	set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}-Install" CACHE PATH "Install path prefix, prepended onto install directories" FORCE)
endif (NOT CMAKE_INSTALL_PREFIX_SET_ONCE)
