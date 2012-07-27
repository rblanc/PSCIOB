PROJECT(DeformableModelsCostFunctionTests)
cmake_policy(SET CMP0015 OLD)


IF(WIN32)
	SET(CMAKE_CXX_FLAGS "/EHsc")
ENDIF(WIN32)


add_executable (TestEnergy TestEnergy.cxx)
ADD_DEPENDENCIES(TestEnergy PSCIOB)
target_link_libraries (TestEnergy ${PSCIOB_LIBRARIES} ${GENERALUTILS_LIBRARIES} ${ITK_LIBRARIES} ${VTK_LIBRARIES}) 

add_executable (TestSensedSceneToImageCostFunctions TestSensedSceneToImageCostFunctions.cxx)
ADD_DEPENDENCIES(TestSensedSceneToImageCostFunctions PSCIOB)
target_link_libraries (TestSensedSceneToImageCostFunctions ${PSCIOB_LIBRARIES} ${GENERALUTILS_LIBRARIES} ${ITK_LIBRARIES} ${VTK_LIBRARIES}) 


add_executable (TestSensedSingleObjectToImageCostFunctions TestSensedSingleObjectToImageCostFunctions.cxx)
ADD_DEPENDENCIES(TestSensedSingleObjectToImageCostFunctions PSCIOB)
target_link_libraries (TestSensedSingleObjectToImageCostFunctions ${PSCIOB_LIBRARIES} ${GENERALUTILS_LIBRARIES} ${ITK_LIBRARIES} ${VTK_LIBRARIES}) 
