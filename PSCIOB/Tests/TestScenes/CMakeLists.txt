PROJECT(DeformableModelsSceneTests)
cmake_policy(SET CMP0015 OLD)



IF(WIN32)
	SET(CMAKE_CXX_FLAGS "/EHsc")
ENDIF(WIN32)



add_executable (TestScenes TestScenes.cxx)
ADD_DEPENDENCIES(TestScenes PSCIOB)
target_link_libraries (TestScenes ${PSCIOB_LIBRARIES} ${GENERALUTILS_LIBRARIES} ${ITK_LIBRARIES} ${VTK_LIBRARIES}) 

