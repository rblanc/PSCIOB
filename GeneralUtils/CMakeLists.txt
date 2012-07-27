cmake_minimum_required (VERSION 2.8.7) 

IF(WIN32)
	SET(CMAKE_CXX_FLAGS "/EHsc")
ENDIF(WIN32)

project(GeneralUtils)


SET(GeneralUtils_MAJOR_VERSION 0) 
SET(GeneralUtils_MINOR_VERSION 8)
SET(GeneralUtils_VERSION ${GeneralUtils_MAJOR_VERSION}.${GeneralUtils_MINOR_VERSION}) 

FIND_PACKAGE(VTK REQUIRED)
include (${VTK_USE_FILE})
include_directories(${VTK_INCLUDE_DIR})
 
FIND_PACKAGE(ITK REQUIRED)
include (${ITK_USE_FILE})
include_directories(${ITK_INCLUDE_DIRS})


SET(GeneralUtils_SUBDIRS BoundingBoxesUtils GeneralUtils ITKUtils ITKVTKUtils LinearTransformUtils RandomVariables VTKClipperWrapper VTKUtils
#add any new directory created in the source dir, here and in the cmake.in file
)

#process the current directory - though it shouldn't contain anything really important...
FILE(GLOB LOCAL_HEADERS *.h *.hpp *.hxx *.txx)
FILE(GLOB LOCAL_SOURCES *.cxx *.cpp *.c)

SOURCE_GROUP("Core\\Header Files" FILES ${LOCAL_HEADERS}) 
SOURCE_GROUP("Core\\Source Files" FILES ${LOCAL_SOURCES})

SET(HEADER_FILES ${LOCAL_HEADERS})
SET(SOURCE_FILES ${LOCAL_SOURCES})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})	
install(FILES ${LOCAL_HEADERS} DESTINATION include/GeneralUtils)
	
#process the subfolders
foreach(dir ${GeneralUtils_SUBDIRS})
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
	install(FILES ${LOCAL_HEADERS} DESTINATION include/GeneralUtils/${dir} )
endforeach(dir) 

#add the clipper files
SET(curdir ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/clipper_ver4.7.3/cpp)
include_directories(${curdir})	
install(FILES ${curdir}/clipper.hpp DESTINATION include/GeneralUtils/3rdParty/clipper_ver4.7.3)	
LIST(APPEND HEADER_FILES ${curdir}/clipper.hpp)
LIST(APPEND SOURCE_FILES ${curdir}/clipper.cpp)


#define the target
add_library(GeneralUtils ${HEADER_FILES} ${SOURCE_FILES})
SET_TARGET_PROPERTIES(GeneralUtils PROPERTIES LINKER_LANGUAGE CXX)

#install the target
install(TARGETS GeneralUtils DESTINATION lib/) 
#install the cmake configuration for the target
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/GeneralUtils-config.cmake.in  ${CMAKE_CURRENT_BINARY_DIR}/GeneralUtils-config.cmake @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/GeneralUtils-config.cmake DESTINATION lib/cmake/)

	
option (BUILD_GeneralUtilsLib_TESTS "Build the Test projects" OFF) 	
if (BUILD_GeneralUtilsLib_TESTS)
	add_subdirectory(Tests)
endif (BUILD_GeneralUtilsLib_TESTS)



