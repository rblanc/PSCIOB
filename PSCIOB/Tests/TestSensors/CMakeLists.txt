PROJECT(DeformableModelsSensorTests)
cmake_policy(SET CMP0015 OLD)


IF(WIN32)
	SET(CMAKE_CXX_FLAGS "/EHsc")
ENDIF(WIN32)


add_executable (TestSensors TestSensors.cxx)
ADD_DEPENDENCIES(TestSensors PSCIOB)
target_link_libraries (TestSensors ${PSCIOB_LIBRARIES} ${GENERALUTILS_LIBRARIES} ${ITK_LIBRARIES} ${VTK_LIBRARIES}) 

