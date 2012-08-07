cmake_minimum_required (VERSION 2.8.7) 

IF(WIN32)
	SET(CMAKE_CXX_FLAGS "/EHsc")
ENDIF(WIN32)

project (PSCIOB)

SET(PSCIOB_MAJOR_VERSION 0) 
SET(PSCIOB_MINOR_VERSION 8)
SET(PSCIOB_VERSION ${PSCIOB_MAJOR_VERSION}.${PSCIOB_MINOR_VERSION}) 


FIND_PACKAGE(VTK REQUIRED)
include (${VTK_USE_FILE})
include_directories(${VTK_INCLUDE_DIR})
 
FIND_PACKAGE(ITK REQUIRED)
include (${ITK_USE_FILE})
include_directories(${ITK_INCLUDE_DIRS})

FIND_PACKAGE(GeneralUtils REQUIRED HINTS ${CMAKE_SOURCE_DIR}/GeneralUtils NO_CMAKE_BUILDS_PATH)
foreach(dir ${GENERALUTILS_SUBDIRS})
	include_directories(${GeneralUtils_DIR}/${dir})
endforeach(dir) 


FIND_PACKAGE(PSCIOB REQUIRED HINTS ${CMAKE_SOURCE_DIR}/PSCIOB NO_CMAKE_BUILDS_PATH)
foreach(dir ${PSCIOB_SUBDIRS})
	include_directories(${PSCIOB_DIR}/${dir})
endforeach(dir) 



#process the current directory - though it shouldn't contain anything really important...
FILE(GLOB LOCAL_HEADERS *.h *.hpp  *.hxx  *.txx)
FILE(GLOB LOCAL_SOURCES *.cxx *.cpp  *.c)

SOURCE_GROUP("Core\\Header Files" FILES ${LOCAL_HEADERS}) 
SOURCE_GROUP("Core\\Source Files" FILES ${LOCAL_SOURCES})

SET(HEADER_FILES ${LOCAL_HEADERS})
SET(SOURCE_FILES ${LOCAL_SOURCES})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})	
install(FILES ${LOCAL_HEADERS} DESTINATION include/PSCIOB)
	
#process the subfolders
foreach(dir ${PSCIOB_SUBDIRS})
	SET(curdir ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
	SET(LOCAL_HEADERS)
	SET(LOCAL_SOURCES)
	FILE(GLOB LOCAL_HEADERS ${dir}/*.h ${dir}/*.hpp ${dir}/*.hxx ${dir}/*.txx)
	FILE(GLOB LOCAL_SOURCES ${dir}/*.cxx ${dir}/*.cpp ${dir}/*.c)

	SET(header_name "${dir}\\Header Files")		
	SET(source_name "${dir}\\Source Files")

	SOURCE_GROUP(${header_name} FILES ${LOCAL_HEADERS}) 
	SOURCE_GROUP(${source_name} FILES ${LOCAL_SOURCES})
		
	LIST(APPEND HEADER_FILES ${LOCAL_HEADERS})
	LIST(APPEND SOURCE_FILES ${LOCAL_SOURCES})
	include_directories(${curdir})
	install(FILES ${LOCAL_HEADERS} DESTINATION include/PSCIOB/${dir})
endforeach(dir) 


#define the target
add_library (PSCIOB ${HEADER_FILES} ${SOURCE_FILES})
ADD_DEPENDENCIES(PSCIOB GeneralUtils)
SET_TARGET_PROPERTIES(PSCIOB PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries (PSCIOB ${GENERALUTILS_LIBRARIES} ${ITK_LIBRARIES} ${VTK_LIBRARIES}) 

#install the target
install(TARGETS PSCIOB DESTINATION lib/) 
#install the cmake configuration for the target
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/PSCIOB-config.cmake.in  ${CMAKE_CURRENT_BINARY_DIR}/PSCIOB-config.cmake @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/PSCIOB-config.cmake DESTINATION lib/cmake/)




option (BUILD_PSCIOB_TESTS "Build the Test projects" OFF) 	
if (BUILD_PSCIOB_TESTS)
	add_subdirectory (Tests)
endif (BUILD_PSCIOB_TESTS)

option (BUILD_PSCIOB_EXAMPLE_APPLICATIONS "Build the Example Application projects" OFF) 	
if (BUILD_PSCIOB_EXAMPLE_APPLICATIONS)
	add_subdirectory (ExampleApplications)
endif (BUILD_PSCIOB_EXAMPLE_APPLICATIONS)

