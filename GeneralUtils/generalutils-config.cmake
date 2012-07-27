


SET(GENERALUTILS_SUBDIRS 3rdParty/clipper_ver4.7.3/cpp BoundingBoxesUtils GeneralUtils ITKUtils ITKVTKUtils LinearTransformUtils RandomVariables VTKClipperWrapper VTKUtils
#add any new directory created in the source dir
)

SET(GENERALUTILS_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
foreach(dir ${GENERALUTILS_INCLUDE_DIRS})
	SET(curdir ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
	LIST(APPEND GENERALUTILS_INCLUDE_DIRS ${curdir})
endforeach(dir) 

SET(GENERALUTILS_LIBRARIES GeneralUtils) 
