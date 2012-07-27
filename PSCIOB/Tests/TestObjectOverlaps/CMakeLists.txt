PROJECT(DeformableModelsObjectOverlapTests)
cmake_policy(SET CMP0015 OLD)


IF(WIN32)
	SET(CMAKE_CXX_FLAGS "/EHsc")
ENDIF(WIN32)


add_executable (TestObjectOverlaps TestObjectOverlaps.cxx)
ADD_DEPENDENCIES(TestObjectOverlaps PSCIOB)
target_link_libraries (TestObjectOverlaps ${PSCIOB_LIBRARIES} ${GENERALUTILS_LIBRARIES} ${ITK_LIBRARIES} ${VTK_LIBRARIES}) 

