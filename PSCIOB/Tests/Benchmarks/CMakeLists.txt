PROJECT(DeformableModelsBenchmarks)
cmake_policy(SET CMP0015 OLD)


IF(WIN32)
	SET(CMAKE_CXX_FLAGS "/EHsc")
ENDIF(WIN32)


add_executable (Benchmarks Benchmarks.cxx Ellipses2D_Synthese.cxx Ellipses2D_Synthese.h)
ADD_DEPENDENCIES(Benchmarks PSCIOB)
target_link_libraries (Benchmarks ${PSCIOB_LIBRARIES} ${IMAGEPROCESSING_LIBRARIES} ${GENERALUTILS_LIBRARIES} ${ITK_LIBRARIES} ${VTK_LIBRARIES}) 

