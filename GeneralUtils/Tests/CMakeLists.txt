PROJECT(GeneralUtilsTests)
cmake_policy(SET CMP0015 OLD)

IF(WIN32)
	SET(CMAKE_CXX_FLAGS "/EHsc")
ENDIF(WIN32)

#since this is part of the same tree as the core library, it does not seem necessary to add a link_directories command... or to find the package...

add_executable(GU_SomeTests SomeTests.cxx)
target_link_libraries(GU_SomeTests GeneralUtils ${ITK_LIBRARIES} ${VTK_LIBRARIES})

add_executable(GU_TestInheritance TestInheritance.cxx)
target_link_libraries(GU_TestInheritance GeneralUtils ${ITK_LIBRARIES} ${VTK_LIBRARIES})


add_executable(GU_Test_C_Features Test_C_Features.cxx)
target_link_libraries(GU_Test_C_Features GeneralUtils ${ITK_LIBRARIES} ${VTK_LIBRARIES}) 


add_executable(GU_Test_ITK_Features Test_ITK_Features.cxx)
target_link_libraries(GU_Test_ITK_Features GeneralUtils ${ITK_LIBRARIES} ${VTK_LIBRARIES})


add_executable(GU_Test_VTK_Features Test_VTK_Features.cxx)
target_link_libraries(GU_Test_VTK_Features GeneralUtils ${ITK_LIBRARIES} ${VTK_LIBRARIES})

add_executable(GU_TestClipper TestClipper.cxx)
target_link_libraries(GU_TestClipper GeneralUtils ${ITK_LIBRARIES} ${VTK_LIBRARIES})

