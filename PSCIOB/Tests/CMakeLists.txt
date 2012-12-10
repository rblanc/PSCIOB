cmake_minimum_required (VERSION 2.6) 

#TODO: add conditions? make a build separate from the main library...?
add_subdirectory(TestShapes)
add_subdirectory(TestScenes)    
add_subdirectory(TestSensors)    
add_subdirectory(TestCostFunctions)
add_subdirectory(Benchmarks)
add_subdirectory(TestObjectOverlaps)
add_subdirectory(TestHoughTransform)
