cmake_policy(SET CMP0015 OLD)


IF(WIN32)
	SET(CMAKE_CXX_FLAGS "/EHsc")
ENDIF(WIN32)


add_executable (Example_SyntheticEllipses Example_SyntheticEllipses.cxx Example_Ellipses2D_Synthese.cxx Example_Ellipses2D_Synthese.h)
ADD_DEPENDENCIES(Example_SyntheticEllipses PSCIOB)
target_link_libraries (Example_SyntheticEllipses ${PSCIOB_LIBRARIES} ${GENERALUTILS_LIBRARIES} ${ITK_LIBRARIES} ${VTK_LIBRARIES}) 

add_executable (Example_FallingSpheres Example_FallingSpheres.cxx)
ADD_DEPENDENCIES(Example_FallingSpheres PSCIOB)
target_link_libraries (Example_FallingSpheres ${PSCIOB_LIBRARIES} ${GENERALUTILS_LIBRARIES} ${ITK_LIBRARIES} ${VTK_LIBRARIES}) 


add_executable (Example_3DForceBiased_Algo Example_3DForceBiased_Algo.cxx)
ADD_DEPENDENCIES(Example_3DForceBiased_Algo PSCIOB)
target_link_libraries (Example_3DForceBiased_Algo ${PSCIOB_LIBRARIES} ${GENERALUTILS_LIBRARIES} ${ITK_LIBRARIES} ${VTK_LIBRARIES}) 

