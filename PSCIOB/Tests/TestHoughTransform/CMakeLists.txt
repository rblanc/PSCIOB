PROJECT(HoughTranformTests)
cmake_policy(SET CMP0015 OLD)


IF(WIN32)
	SET(CMAKE_CXX_FLAGS "/EHsc")
ENDIF(WIN32)


add_executable (TestHoughTranform TestHoughTranform.cxx)
ADD_DEPENDENCIES(TestHoughTranform PSCIOB)
target_link_libraries (TestHoughTranform ${PSCIOB_LIBRARIES} ${GENERALUTILS_LIBRARIES} ${ITK_LIBRARIES} ${VTK_LIBRARIES}) 
