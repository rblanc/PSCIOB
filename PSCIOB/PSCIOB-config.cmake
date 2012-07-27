
SET(PSCIOB_SUBDIRS CostFunctions ParametricObject GeometricShapes ObjectCollection ObjectInteractionPriors OptimizationManager Optimizer PoseTransforms ReversibleSceneModifierKernels SceneGlobalPriors Sensors
#add any new directory created in the source dir
)

SET(PSCIOB_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
foreach(dir ${PSCIOB_SUBDIRS})
	SET(curdir ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
	LIST(APPEND PSCIOB_INCLUDE_DIRS ${curdir})
endforeach(dir) 

SET(PSCIOB_LIBRARIES PSCIOB) 