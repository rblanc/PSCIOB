PROJECT(DeformableModelsShapesTests)
cmake_policy(SET CMP0015 OLD)


IF(WIN32)
	SET(CMAKE_CXX_FLAGS "/EHsc")
ENDIF(WIN32)



add_executable (TestShapes TestShapes.cxx)
ADD_DEPENDENCIES(TestShapes PSCIOB)
target_link_libraries (TestShapes ${PSCIOB_LIBRARIES} ${GENERALUTILS_LIBRARIES} ${ITK_LIBRARIES} ${VTK_LIBRARIES}) 


add_executable (TestFastShapes TestFastShapes.cxx)
ADD_DEPENDENCIES(TestFastShapes PSCIOB)
target_link_libraries (TestFastShapes ${PSCIOB_LIBRARIES} ${GENERALUTILS_LIBRARIES} ${ITK_LIBRARIES} ${VTK_LIBRARIES}) 
