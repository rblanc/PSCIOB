/*
 * This file is part of the psciob library.
 *
 * Copyright (c) 2012, Remi Blanc, University of Bordeaux
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met: 
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


//seems to enable IntelliSence for VisualStudio...
#pragma once

#include <iostream>
#include <typeinfo>

#include <vtkDataSet.h>

#include "ITKUtils.h"
#include "VTKUtils.h"
#include "GeneralUtils.h"

#include "PoseTransformedBinaryShape.h"
#include "shape3DSphere.h"
#include "Translation3DTransform.h"
#include "TranslationScale3DTransform.h"
#include "Direct3DSphere.h"
#include "VTKScene.h"

#include "DummyScene.h"

#include "itkLabelMapToBinaryImageFilter.h"
#include "itkLabelMapToLabelImageFilter.h"

//#include "VTKObjectOverlap.h"

#include "LabelMapScene.h"
#include "PixelSetObjectOverlap.h"
#include "LabelObjectOverlap.h"

#include "LabelImageScene.h"

#include "Fast2DEllipse.h"

using namespace psciob;

//
void TestVTKScene() {
	std::cout<<"\n\n TestVTKScene: draw a few spheres, remove some, save as vtk polydata file & labelimage"<<std::endl;

	shape3DSphere::Pointer sphereShape = shape3DSphere::New();
	TranslationScale3DTransform::Pointer transform = TranslationScale3DTransform::New();
	PoseTransformedBinaryShape<3>::Pointer sphere = PoseTransformedBinaryShape<3>::New();

	vnl_vector<double> pose3D(4); pose3D.fill(0); pose3D(3) = 2.215; 
	transform->SetParameters(pose3D);
	sphere->SetShapeAndTransform(sphereShape, transform);

	typedef VTKScene<3> Scene3DType;
	//typedef itk::Image<unsigned char, 2> Image2DType;	typedef itk::Image<unsigned char, 3> Image3DType;

	Scene3DType::Pointer scene3D = Scene3DType::New();
	//define the characteristics of the scene
	vnl_vector<double> sceneSpacing(3);	sceneSpacing.fill(0.5);
	vnl_vector<double> sceneBBox(6);	for (unsigned i=0 ; i<3 ; i++) { sceneBBox(2*i) = -10;	sceneBBox(2*i+1) = 10; }
	scene3D->SetPhysicalDimensions(sceneBBox, sceneSpacing);

	std::cout<<"allocated the scene"<<std::endl;
	//add a few objects to the scene
	vnl_vector<double> sphereParams(4);
	sphereParams(0) = 0;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	
	std::cout<<"setting parameters"<<std::endl;
	sphere->SetParameters(sphereParams);	
	std::cout<<"adding object to scene"<<std::endl;
	Scene3DType::IDType id = scene3D->AddObject(sphere);
	std::cout<<"ADDED OBJECT 1, id = "<<id<<std::endl;

	scene3D->InteractiveSceneVisualization();

	std::cout<<"accessing first object in the scene: "<<std::endl;
	Scene3DType::ObjectInScene *objptr = scene3D->GetObject( id );
	if (!objptr) std::cout<<"GetObject apparently failed..."<<std::endl;
	std::cout<<"label value = "<<objptr->id<<std::endl;
	WriteITKImageToFile<itk::Image<unsigned char, 3>>("test3DScene_Sphere1Alone.nii", scene3D->GetObject(1)->obj->GetObjectAsBinaryImage());
	sphereParams(0) = 1;	sphereParams(1) = 1;	sphereParams(2) = 0;	sphereParams(3) = 1.5;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	std::cout<<"ADDED OBJECT 2"<<std::endl;
	sphereParams(0) = -4.2;	sphereParams(1) = 2.3;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	std::cout<<"ADDED OBJECT 3"<<std::endl;
	sphereParams(0) = 2.2;	sphereParams(1) = 4.3;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	std::cout<<"ADDED OBJECT 4"<<std::endl;

	scene3D->InteractiveSceneVisualization();

	std::cout<<"added "<<scene3D->GetNumberOfObjects()<<" spheres to the scene"<<std::endl;
	std::cout<<"writing 3D representations of the scene to files: test3DScene.vtk and test3DScene_Labels.nii"<<std::endl;
	
	WriteMirrorPolyDataToFile("test3DVTKScene_1.vtk", scene3D->GetSceneAsVTKPolyData());
	//scene3D->WriteSimpleVTKRepresentation("test3DLabelImageScene_1.vtk");
	std::cout<<"writing 3D label image"<<std::endl;
	
	Scene3DType::LabelMapType::Pointer labelMap = scene3D->GetSceneAsLabelMap();

	typedef itk::LabelMapToBinaryImageFilter< Scene3DType::LabelMapType, Scene3DType::BinaryImageType> LabelMapToBinaryImageFilterType;
	LabelMapToBinaryImageFilterType::Pointer m_labelMapToBinaryImageFilter = LabelMapToBinaryImageFilterType::New();
	m_labelMapToBinaryImageFilter->SetInput(labelMap);
	m_labelMapToBinaryImageFilter->Update();
	WriteITKImageToFile<Scene3DType::BinaryImageType>("test3DVTKScene_1_BinaryImage.nii", m_labelMapToBinaryImageFilter->GetOutput());

	typedef itk::LabelMapToLabelImageFilter< Scene3DType::LabelMapType, Scene3DType::LabelImageType> LabelMapToLabelImageFilterType;
	LabelMapToLabelImageFilterType::Pointer m_labelMapToLabelImageFilter = LabelMapToLabelImageFilterType::New();
	m_labelMapToLabelImageFilter->SetInput(labelMap);
	m_labelMapToLabelImageFilter->Update();
	WriteITKImageToFile<Scene3DType::LabelImageType>("test3DVTKScene_1_LabelImage.nii", m_labelMapToLabelImageFilter->GetOutput());

	std::cout<<"number of objects: "<<scene3D->GetNumberOfObjects()<<std::endl;

	std::cout<<"removing object with label 2"<<std::endl;
	scene3D->RemoveObject(2);
	std::cout<<"number of objects: "<<scene3D->GetNumberOfObjects()<<std::endl;
	std::cout<<"writing 3D representations of the scene to files: test3DScene.vtk and test3DScene_Labels.nii"<<std::endl;

	std::cout<<"removing object with label 4"<<std::endl;
	if (!scene3D->RemoveObject(4)) std::cout<<"  pb removing object..."<<std::endl;;
	std::cout<<"number of objects: "<<scene3D->GetNumberOfObjects()<<std::endl;


	WriteMirrorPolyDataToFile("test3DVTKScene_2.vtk", scene3D->GetSceneAsVTKPolyData());
	//scene3D->WriteSimpleVTKRepresentation("test3DLabelImageScene_1.vtk");
	std::cout<<"writing 3D label image"<<std::endl;
	
	labelMap = scene3D->GetSceneAsLabelMap();

	m_labelMapToBinaryImageFilter = LabelMapToBinaryImageFilterType::New();
	m_labelMapToBinaryImageFilter->SetInput(labelMap);
	m_labelMapToBinaryImageFilter->Update();
	WriteITKImageToFile<Scene3DType::BinaryImageType>("test3DVTKScene_2_BinaryImage.nii", m_labelMapToBinaryImageFilter->GetOutput());

	m_labelMapToLabelImageFilter = LabelMapToLabelImageFilterType::New();
	m_labelMapToLabelImageFilter->SetInput(labelMap);
	m_labelMapToLabelImageFilter->Update();
	WriteITKImageToFile<Scene3DType::LabelImageType>("test3DVTKScene_2_LabelImage.nii", m_labelMapToLabelImageFilter->GetOutput());

	std::cout<<"\n\nnumber of objects: "<<scene3D->GetNumberOfObjects()<<std::endl;

	std::cout<<"\n\n TestVTKScene: completed successfully "<<std::endl;
}

#include "VTKObjectOverlap.h"
void Test3DVTKSceneWithOverlapMonitoring() {
	std::cout<<"\n\n Test3DVTKSceneWithOverlapMonitoring: draw a few spheres, remove some, save as vtk polydata file & labelimage"<<std::endl;
	clock_t t0;

	shape3DSphere::Pointer sphereShape = shape3DSphere::New();
	TranslationScale3DTransform::Pointer transform = TranslationScale3DTransform::New();
	PoseTransformedBinaryShape<3>::Pointer sphere = PoseTransformedBinaryShape<3>::New();

	vnl_vector<double> pose3D(4); pose3D.fill(0); pose3D(3) = 2.215; 
	transform->SetParameters(pose3D);
	sphere->SetShapeAndTransform(sphereShape, transform);

	typedef VTKScene<3> Scene3DType;
	Scene3DType::Pointer scene3D = Scene3DType::New();

	typedef VTKObjectOverlap<Scene3DType> InteractionManagerType;
	InteractionManagerType::Pointer interactionManager = InteractionManagerType::New();
	
	scene3D->SetInteractionManager( (Scene3DType::ObjectInteractionManagerType *)interactionManager.GetPointer() );

	//define the characteristics of the scene
	vnl_vector<double> sceneSpacing(3);	sceneSpacing.fill(0.5);
	vnl_vector<double> sceneBBox(6);	for (unsigned i=0 ; i<3 ; i++) { sceneBBox(2*i) = -10;	sceneBBox(2*i+1) = 10; }
	scene3D->SetPhysicalDimensions(sceneBBox, sceneSpacing);

	//add a few objects to the scene
	vnl_vector<double> sphereParams(4);
	t0 = clock();
	sphereParams(0) = 0; sphereParams(1) = 0; sphereParams(2) = 0; sphereParams(3) = 2; sphere->SetParameters(sphereParams);	
	Scene3DType::IDType id = scene3D->AddObject(sphere);
	std::cout<<"ADDED OBJECT 1, id = "<<id<<" in "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;

	scene3D->InteractiveSceneVisualization();
	std::cout<<"number of objects in the scene: "<<scene3D->GetNumberOfObjects()<<std::endl;
	std::cout<<"number of objects interacting with object label 1: "<<scene3D->GetObject(1)->interactionData.size()<<std::endl;

	std::cout<<"\nadding a second object, not overlapping"<<std::endl;
	t0 = clock();
	sphereParams(0) = 5;	sphereParams(1) = 3;	sphereParams(2) = 0;	sphereParams(3) = 1.5;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	std::cout<<"ADDED OBJECT 2, id = "<<id<<" in "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;

	scene3D->InteractiveSceneVisualization();
	std::cout<<"number of objects in the scene: "<<scene3D->GetNumberOfObjects()<<std::endl;
	std::cout<<"number of objects interacting with object label 1: "<<scene3D->GetObject(1)->interactionData.size()<<std::endl;
	std::cout<<"number of objects interacting with object label 2: "<<scene3D->GetObject(2)->interactionData.size()<<std::endl;

	//
	std::cout<<"\nadding a third object, overlapping with object 1"<<std::endl;
	t0=clock();
	sphereParams(0) = 1; sphereParams(1) =-0.5; sphereParams(2) = 0; sphereParams(3) = 3; sphere->SetParameters(sphereParams); scene3D->AddObject(sphere);
	std::cout<<"ADDED OBJECT 3, id = "<<id<<" in "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;

	scene3D->InteractiveSceneVisualization();
	std::cout<<"number of objects in the scene: "<<scene3D->GetNumberOfObjects()<<std::endl;
	std::cout<<"number of objects interacting with object label 1: "<<scene3D->GetObject(1)->interactionData.size()<<std::endl;
	std::cout<<"number of objects interacting with object label 2: "<<scene3D->GetObject(2)->interactionData.size()<<std::endl;
	std::cout<<"number of objects interacting with object label 3: "<<scene3D->GetObject(3)->interactionData.size()<<std::endl;
	std::cout<<"interaction cost for interaction 1-3: flag = "<<scene3D->GetObject(1)->interactionData[3].interactionCostFlag<<", "<<scene3D->GetObject(1)->interactionData[3].interactionCost<<", and reverse: "<<scene3D->GetObject(3)->interactionData[1].interactionCostFlag<<", "<<scene3D->GetObject(3)->interactionData[1].interactionCost<<std::endl;

	std::cout<<"\nadding a fourth object, overlapping with object 1 and 2, maybe 3..."<<std::endl;
	t0=clock();
	sphereParams(0) = 2.5;	sphereParams(1) =1.5;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	std::cout<<"ADDED OBJECT 4, id = "<<id<<" in "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;

	std::cout<<"number of objects interacting with object label 1: "<<scene3D->GetObject(1)->interactionData.size()<<std::endl;
	std::cout<<"number of objects interacting with object label 2: "<<scene3D->GetObject(2)->interactionData.size()<<std::endl;
	std::cout<<"number of objects interacting with object label 3: "<<scene3D->GetObject(3)->interactionData.size()<<std::endl;
	std::cout<<"number of objects interacting with object label 4: "<<scene3D->GetObject(4)->interactionData.size()<<std::endl;

	scene3D->InteractiveSceneVisualization();

	std::cout<<"\nRemoving object with label 1..."<<std::endl;
	t0=clock();
	scene3D->RemoveObject(1);
	std::cout<<"REMOVED OBJECT 1, id = "<<id<<" in "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
	std::cout<<"number of objects in the scene: "<<scene3D->GetNumberOfObjects()<<std::endl;
	std::cout<<"number of objects interacting with object label 2: "<<scene3D->GetObject(2)->interactionData.size()<<std::endl;
	std::cout<<"number of objects interacting with object label 3: "<<scene3D->GetObject(3)->interactionData.size()<<std::endl;
	std::cout<<"number of objects interacting with object label 4: "<<scene3D->GetObject(4)->interactionData.size()<<std::endl;

	scene3D->InteractiveSceneVisualization();

	std::cout<<"\nModifying the parameters of object with label 3..."<<std::endl;
	t0=clock();
	sphereParams(0) = -2; sphereParams(1) = -3; sphereParams(2) = 0; sphereParams(3) = 3;
	scene3D->ModifyObjectParameters(3, sphereParams);
	std::cout<<"MODIFIED OBJECT 3, id = "<<id<<" in "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
	std::cout<<"number of objects in the scene: "<<scene3D->GetNumberOfObjects()<<std::endl;
	std::cout<<"number of objects interacting with object label 2: "<<scene3D->GetObject(2)->interactionData.size()<<std::endl;
	std::cout<<"number of objects interacting with object label 3: "<<scene3D->GetObject(3)->interactionData.size()<<std::endl;
	std::cout<<"number of objects interacting with object label 4: "<<scene3D->GetObject(4)->interactionData.size()<<std::endl;

	scene3D->InteractiveSceneVisualization();

	std::cout<<"\nModifying back the parameters of object with label 3..."<<std::endl;
	t0=clock();
	sphereParams(0) = 1; sphereParams(1) =-0.5; sphereParams(2) = 0; sphereParams(3) = 3;
	scene3D->ModifyObjectParameters(3, sphereParams);
	std::cout<<"MODIFIED OBJECT 3, id = "<<id<<" in "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
	std::cout<<"number of objects in the scene: "<<scene3D->GetNumberOfObjects()<<std::endl;
	std::cout<<"number of objects interacting with object label 2: "<<scene3D->GetObject(2)->interactionData.size()<<std::endl;
	std::cout<<"number of objects interacting with object label 3: "<<scene3D->GetObject(3)->interactionData.size()<<std::endl;
	std::cout<<"number of objects interacting with object label 4: "<<scene3D->GetObject(4)->interactionData.size()<<std::endl;

	scene3D->InteractiveSceneVisualization();

	//std::cout<<"loop modifying back and forth the object... just to check for memory leak..."<<std::endl;
	//for (unsigned i=0 ; i<1000 ; i++) {
	//	sphereParams(0) = -2; sphereParams(1) = -3; sphereParams(2) = 0; sphereParams(3) = 3;
	//	scene3D->ModifyObjectParameters(3, sphereParams);

	//	sphereParams(0) = -5; sphereParams(1) =-7; sphereParams(2) = 0; sphereParams(3) = 3;
	//	scene3D->ModifyObjectParameters(3, sphereParams);

	//	scene3D->InteractiveSceneVisualization();
	//}
	//TODO: check for 2D, check also that modifications of object parameters also work.
	std::cout<<"\n\n Test3DVTKSceneWithOverlapMonitoring: completed successfully "<<std::endl;
}

//
void TestLabelMapScene() {
	std::cout<<"\n\n TestLabelMapScene, using LabelObjectOverlap: draw a few spheres, remove some, save as vtk polydata file & labelimage "<<std::endl;


	shape3DSphere::Pointer sphereShape = shape3DSphere::New();
	TranslationScale3DTransform::Pointer transform = TranslationScale3DTransform::New();
	PoseTransformedBinaryShape<3>::Pointer sphere = PoseTransformedBinaryShape<3>::New();

	vnl_vector<double> pose3D(4); pose3D.fill(0); pose3D(3) = 2.215; 
	transform->SetParameters(pose3D);
	sphere->SetShapeAndTransform(sphereShape, transform);

	//typedef LabelMapScene<3, unsigned char, unsigned short, ObjectCostsAndPixelSetContainer, PixelSetIntersectionContainer> Scene3DType;
	//typedef PixelSetObjectOverlap<Scene3DType> InteractionManagerType;
	typedef LabelMapScene<3, unsigned char, unsigned short, ObjectCostsContainer, LabelObjectIntersectionContainer<unsigned short, 3>> Scene3DType;
	typedef LabelObjectOverlap<Scene3DType> InteractionManagerType;

	Scene3DType::Pointer scene3D = Scene3DType::New();

	//define the characteristics of the scene
	vnl_vector<double> sceneSpacing(3);	sceneSpacing.fill(0.3);
	vnl_vector<double> sceneBBox(6);	for (unsigned i=0 ; i<3 ; i++) { sceneBBox(2*i) = -10;	sceneBBox(2*i+1) = 10; }
	scene3D->SetPhysicalDimensions(sceneBBox, sceneSpacing);

	InteractionManagerType::Pointer interactionManager = InteractionManagerType::New();
	scene3D->SetInteractionManager( (Scene3DType::ObjectInteractionManagerType *)interactionManager.GetPointer() );

	std::cout<<"allocated the scene"<<std::endl;
	//add a few objects to the scene
	vnl_vector<double> sphereParams(4);
	sphereParams(0) = 0;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	
	std::cout<<"setting parameters"<<std::endl;
	sphere->SetParameters(sphereParams);	
	std::cout<<"adding object to scene"<<std::endl;
	Scene3DType::IDType id = scene3D->AddObject(sphere);
	std::cout<<"ADDED OBJECT 1, id = "<<id<<std::endl;

	std::cout<<"accessing first object in the scene: "<<std::endl;
	Scene3DType::ObjectInScene *objptr = scene3D->GetObject( id );
	if (!objptr) std::cout<<"GetObject apparently failed..."<<std::endl;
	std::cout<<"label value = "<<objptr->id<<std::endl;
	WriteITKImageToFile<itk::Image<unsigned char, 3>>("test3DScene_Sphere1Alone.nii", scene3D->GetObject(1)->obj->GetObjectAsBinaryImage());
	sphereParams(0) = 1;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	std::cout<<"ADDED OBJECT 2"<<std::endl;
	sphereParams(0) = -4.2;	sphereParams(1) = 2.3;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	std::cout<<"ADDED OBJECT 3"<<std::endl;
	sphereParams(0) = 6;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	std::cout<<"ADDED OBJECT 4"<<std::endl;

	std::cout<<"browsing all objects in the scene, and checking for interactions"<<std::endl;
	SceneObjectIterator<Scene3DType> it(scene3D);
	for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
		std::cout<<"object label: "<<it.GetID()<<std::endl;
		std::cout<<"number of interacting objects: "<<it.GetObjectInScene()->interactionData.size()<<std::endl;
		for (Scene3DType::ObjectInteractionIterator iit = it.GetObjectInScene()->interactionData.begin() ; iit!=it.GetObjectInScene()->interactionData.end() ; iit++) {
			std::cout<<"  interaction with object: "<<iit->first<<", interaction cost: "<<iit->second.interactionCost<<", nb of overlapping pixels: "<<iit->second.nbOverlappingPixels<<std::endl;
		}
	}


	std::cout<<"\nAdding an object which overlaps with objects 2 and 4"<<std::endl;
	sphereParams(0) = 3.5;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 1.6;	sphere->SetParameters(sphereParams);	
	id = scene3D->AddObject(sphere);
	std::cout<<"browsing all objects in the scene, and checking for interactions"<<std::endl;
	for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
		std::cout<<"object label: "<<it.GetID()<<" - number of interacting objects: "<<it.GetObjectInScene()->interactionData.size()<<std::endl;
		for (Scene3DType::ObjectInteractionIterator iit = it.GetObjectInScene()->interactionData.begin() ; iit!=it.GetObjectInScene()->interactionData.end() ; iit++) {
			std::cout<<"  interaction with object: "<<iit->first<<", interaction cost: "<<iit->second.interactionCost<<", nb of overlapping pixels: "<<iit->second.nbOverlappingPixels<<std::endl;
		}
	}

	std::cout<<"\nRemoving an object which overlaps with objects 2 and 4"<<std::endl;
	scene3D->RemoveObject(id);
	std::cout<<"browsing all objects in the scene, and checking for interactions"<<std::endl;
	for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
		std::cout<<"object label: "<<it.GetID()<<" - number of interacting objects: "<<it.GetObjectInScene()->interactionData.size()<<std::endl;
		for (Scene3DType::ObjectInteractionIterator iit = it.GetObjectInScene()->interactionData.begin() ; iit!=it.GetObjectInScene()->interactionData.end() ; iit++) {
			std::cout<<"  interaction with object: "<<iit->first<<", interaction cost: "<<iit->second.interactionCost<<", nb of overlapping pixels: "<<iit->second.nbOverlappingPixels<<std::endl;
		}
	}


	std::cout<<"\nAdding and removing the previous object repeatdly to check for computation time , memory leaks, ..."<<std::endl;
	clock_t t0 = clock();
	for (unsigned i=0 ; i<1001 ; i++) {
		id = scene3D->AddObject(sphere);
		scene3D->RemoveObject(id);

		if (i==100) std::cout<<"100 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s. - id = "<<id<<std::endl;
		if (i==500) std::cout<<"500 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s. - id = "<<id<<std::endl;
		if (i==1000) std::cout<<"1000 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s. - id = "<<id<<std::endl;
	}

	std::cout<<"\n\n TestLabelMapScene: completed successfully "<<std::endl;
}

//
void TestLabelImageScene() {
	std::cout<<"\n\n TestLabelImageScene: draw a few spheres, some with overlaps, remove some, test that the overlaps are properly managed."<<std::endl;

	shape3DSphere::Pointer sphereShape = shape3DSphere::New();
	TranslationScale3DTransform::Pointer transform = TranslationScale3DTransform::New();
	PoseTransformedBinaryShape<3>::Pointer sphere = PoseTransformedBinaryShape<3>::New();

	vnl_vector<double> pose3D(4); pose3D.fill(0); pose3D(3) = 2.215; 
	transform->SetParameters(pose3D);
	sphere->SetShapeAndTransform(sphereShape, transform);

	//typedef LabelImageScene<3, unsigned char, unsigned short, ObjectCostsAndPixelSetContainer, PixelSetIntersectionContainer> Scene3DType;
	typedef LabelImageScene<3, unsigned char, unsigned short, ObjectCostsContainer, LabelObjectIntersectionContainer<unsigned short, 3>> Scene3DType;

	Scene3DType::Pointer scene3D = Scene3DType::New();

	//define the characteristics of the scene
	vnl_vector<double> sceneSpacing(3);	sceneSpacing.fill(0.3);
	vnl_vector<double> sceneBBox(6);	for (unsigned i=0 ; i<3 ; i++) { sceneBBox(2*i) = -10;	sceneBBox(2*i+1) = 10; }
	scene3D->SetPhysicalDimensions(sceneBBox, sceneSpacing);
	std::cout<<"number of pixels in the scene: "<<scene3D->GetSceneImageRegion().GetSize(0)<<", "<<scene3D->GetSceneImageRegion().GetSize(1)<<", "<<scene3D->GetSceneImageRegion().GetSize(2)<<std::endl;
	//add a few objects to the scene
	vnl_vector<double> sphereParams(4);
	sphereParams(0) = 0;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	
	std::cout<<"setting parameters"<<std::endl;
	sphere->SetParameters(sphereParams);	
	std::cout<<"adding object to scene"<<std::endl;
	Scene3DType::IDType id = scene3D->AddObject(sphere);
	std::cout<<"ADDED OBJECT 1, id = "<<id<<std::endl;

	std::cout<<"accessing first object in the scene: "<<std::endl;
	Scene3DType::ObjectInScene *objptr = scene3D->GetObject( id );
	if (!objptr) std::cout<<"GetObject apparently failed..."<<std::endl;
	std::cout<<"label value = "<<objptr->id<<std::endl;
	WriteITKImageToFile<itk::Image<unsigned char, 3>>("test3DScene_Sphere1Alone.nii", scene3D->GetObject(1)->obj->GetObjectAsBinaryImage());
	sphereParams(0) = 1;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	std::cout<<"ADDED OBJECT 2"<<std::endl;
	sphereParams(0) = -4.2;	sphereParams(1) = 2.3;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	std::cout<<"ADDED OBJECT 3"<<std::endl;
	sphereParams(0) = 6;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	std::cout<<"ADDED OBJECT 4"<<std::endl;

	std::cout<<"browsing all objects in the scene, and checking for interactions"<<std::endl;
	SceneObjectIterator<Scene3DType> it(scene3D);
	for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
		std::cout<<"object label: "<<it.GetID()<<std::endl;
		std::cout<<"number of interacting objects: "<<it.GetObjectInScene()->interactionData.size()<<std::endl;
		for (Scene3DType::ObjectInteractionIterator iit = it.GetObjectInScene()->interactionData.begin() ; iit!=it.GetObjectInScene()->interactionData.end() ; iit++) {
			std::cout<<"  interaction with object: "<<iit->first<<", interaction cost: "<<iit->second.interactionCost<<", nb of overlapping pixels: "<<iit->second.nbOverlappingPixels<<std::endl;
		}
	}


	std::cout<<"\nAdding an object which overlaps with objects 2 and 4"<<std::endl;
	sphereParams(0) = 3.5;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 1.6;	sphere->SetParameters(sphereParams);	
	id = scene3D->AddObject(sphere);
	std::cout<<"browsing all objects in the scene, and checking for interactions"<<std::endl;
	for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
		std::cout<<"object label: "<<it.GetID()<<" - number of interacting objects: "<<it.GetObjectInScene()->interactionData.size()<<std::endl;
		for (Scene3DType::ObjectInteractionIterator iit = it.GetObjectInScene()->interactionData.begin() ; iit!=it.GetObjectInScene()->interactionData.end() ; iit++) {
			std::cout<<"  interaction with object: "<<iit->first<<", interaction cost: "<<iit->second.interactionCost<<", nb of overlapping pixels: "<<iit->second.nbOverlappingPixels<<std::endl;
		}
	}

	std::cout<<"\nRemoving an object which overlaps with objects 2 and 4"<<std::endl;
	scene3D->RemoveObject(id);
	std::cout<<"browsing all objects in the scene, and checking for interactions"<<std::endl;
	for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
		std::cout<<"object label: "<<it.GetID()<<" - number of interacting objects: "<<it.GetObjectInScene()->interactionData.size()<<std::endl;
		for (Scene3DType::ObjectInteractionIterator iit = it.GetObjectInScene()->interactionData.begin() ; iit!=it.GetObjectInScene()->interactionData.end() ; iit++) {
			std::cout<<"  interaction with object: "<<iit->first<<", interaction cost: "<<iit->second.interactionCost<<", nb of overlapping pixels: "<<iit->second.nbOverlappingPixels<<std::endl;
		}
	}


	std::cout<<"\nAdding and removing the previous object repeatdly to check for computation time , memory leaks, ..."<<std::endl;
	clock_t t0 = clock();
	for (unsigned i=0 ; i<1001 ; i++) {
		id = scene3D->AddObject(sphere);
		scene3D->RemoveObject(id);

		if (i==100) std::cout<<"100 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s. - id = "<<id<<std::endl;
		if (i==500) std::cout<<"500 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s. - id = "<<id<<std::endl;
		if (i==1000) std::cout<<"1000 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s. - id = "<<id<<std::endl;
	}

	std::cout<<"\n\n TestLabelImageScene: completed successfully "<<std::endl;
}


void TestModificationsLabelImageScene() {
	std::cout<<"\n\n TestModificationsLabelImageScene: draw 3 spheres, two partially overlapping. A fourth ball is introduced, and translated such that it passes through various overlap configurations..."<<std::endl;

	shape3DSphere::Pointer sphereShape = shape3DSphere::New();
	TranslationScale3DTransform::Pointer transform = TranslationScale3DTransform::New();
	PoseTransformedBinaryShape<3>::Pointer sphere = PoseTransformedBinaryShape<3>::New();

	vnl_vector<double> pose3D(4); pose3D.fill(0); pose3D(3) = 2.215; 
	transform->SetParameters(pose3D);
	sphere->SetShapeAndTransform(sphereShape, transform);

	typedef PoseTransformedBinaryShape<3>::BaseClass Base3DObjectType;	

	typedef LabelImageScene<3, unsigned char, unsigned short> Scene3DType;

	Scene3DType::Pointer scene3D = Scene3DType::New();

	//define the characteristics of the scene
	vnl_vector<double> sceneSpacing(3);	sceneSpacing.fill(0.3);
	vnl_vector<double> sceneBBox(6);	for (unsigned i=0 ; i<3 ; i++) { sceneBBox(2*i) = -30;	sceneBBox(2*i+1) = 30; }
	scene3D->SetPhysicalDimensions(sceneBBox, sceneSpacing);
	std::cout<<"number of pixels in the scene: "<<scene3D->GetSceneImageRegion().GetSize(0)<<", "<<scene3D->GetSceneImageRegion().GetSize(1)<<", "<<scene3D->GetSceneImageRegion().GetSize(2)<<std::endl;
	//add a few objects to the scene
	vnl_vector<double> sphereParams(4);
	sphereParams(0) = 0;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 5;	
	sphere->SetParameters(sphereParams);	
	Scene3DType::IDType id = scene3D->AddObject(sphere);
	std::cout<<"ADDED OBJECT 1, id = "<<id<<std::endl;

	std::cout<<"accessing first object in the scene: "<<std::endl;
	Scene3DType::ObjectInScene *objptr = scene3D->GetObject( id );
	if (!objptr) std::cout<<"GetObject apparently failed..."<<std::endl;
	std::cout<<"label value = "<<objptr->id<<std::endl;
	WriteITKImageToFile<itk::Image<unsigned char, 3>>("test3DScene_Sphere1Alone.nii", scene3D->GetObject(1)->obj->GetObjectAsBinaryImage());
	sphereParams(0) = 7.5;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 5;	sphere->SetParameters(sphereParams); scene3D->AddObject(sphere);
	std::cout<<"ADDED OBJECT 2"<<std::endl;
	sphereParams(0) = 20;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 5;	sphere->SetParameters(sphereParams); scene3D->AddObject(sphere);
	std::cout<<"ADDED OBJECT 3"<<std::endl;
	sphereParams(0) = -10;	sphereParams(1) = 2.5;	sphereParams(2) = 0;	sphereParams(3) = 1;	sphere->SetParameters(sphereParams); id=scene3D->AddObject(sphere);
	std::cout<<"ADDED OBJECT 4 - smaller ball that will move around..."<<std::endl;

	std::vector<double> pos_x;
	pos_x.push_back(-7.5); pos_x.push_back(-5); pos_x.push_back(-4); pos_x.push_back(0); pos_x.push_back(2.5); pos_x.push_back(3); pos_x.push_back(3.5); pos_x.push_back(4);
	pos_x.push_back(5); pos_x.push_back(7.5); pos_x.push_back(12); pos_x.push_back(12.5); pos_x.push_back(13); pos_x.push_back(14); pos_x.push_back(15.5);
	pos_x.push_back(20); pos_x.push_back(25); pos_x.push_back(27);

	for (unsigned i=0 ; i<pos_x.size() ; i++) {
		std::cout<<"\nmoving the small ball to position : "<<pos_x[i]<<std::endl;
		sphereParams(0) = pos_x[i];
		scene3D->ModifyObjectParameters(id, sphereParams);

		std::cout<<"browsing all objects in the scene, and checking for interactions"<<std::endl;
		SceneObjectIterator<Scene3DType> it(scene3D);
		for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
			std::cout<<"object label: "<<it.GetID()<<", number of interacting objects: "<<it.GetObjectInScene()->interactionData.size()<<std::endl;
			for (Scene3DType::ObjectInteractionIterator iit = it.GetObjectInScene()->interactionData.begin() ; iit!=it.GetObjectInScene()->interactionData.end() ; iit++) {
				std::cout<<"  interaction with object: "<<iit->first<<", interaction cost: "<<iit->second.interactionCost<<", nb of overlapping pixels: "<<iit->second.nbOverlappingPixels<<std::endl;
			}
		}
	}

	std::cout<<"\n\n Exchanging the roles of ball 1 and 4, and moving the small sphere (now id 1) back to its original position"<<std::endl;
	scene3D->ModifyObjectParameters(1, sphereParams);
	sphereParams(0) = 0;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 5;
	scene3D->ModifyObjectParameters(4, sphereParams);

	sphereParams(0) = -10;	sphereParams(1) = 2.5;	sphereParams(2) = 0;	sphereParams(3) = 1;
	for (unsigned i=0 ; i<pos_x.size() ; i++) {
		std::cout<<"\nmoving the small ball to position : "<<pos_x[pos_x.size()-i-1]<<std::endl;
		sphereParams(0) = pos_x[pos_x.size()-i-1];
		scene3D->ModifyObjectParameters(1, sphereParams);

		std::cout<<"browsing all objects in the scene, and checking for interactions"<<std::endl;
		SceneObjectIterator<Scene3DType> it(scene3D);
		for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
			std::cout<<"object label: "<<it.GetID()<<", number of interacting objects: "<<it.GetObjectInScene()->interactionData.size()<<std::endl;
			for (Scene3DType::ObjectInteractionIterator iit = it.GetObjectInScene()->interactionData.begin() ; iit!=it.GetObjectInScene()->interactionData.end() ; iit++) {
				std::cout<<"  interaction with object: "<<iit->first<<", interaction cost: "<<iit->second.interactionCost<<", nb of overlapping pixels: "<<iit->second.nbOverlappingPixels<<std::endl;
			}
		}
	}


	std::cout<<"\n\n TestModificationsLabelImageScene: completed successfully "<<std::endl;
}


//
int main(int argc, char** argv) {

	try {
		TestVTKScene();

//		Test2DVTKSceneWithOverlapMonitoring(); //TODO
		Test3DVTKSceneWithOverlapMonitoring();

		TestLabelMapScene();
		TestLabelImageScene();
		TestModificationsLabelImageScene();

	}
	catch (DeformableModelException& e) {
		std::cout << "Exception occured while running the test cases" << std::endl;
		std::cout << e.what() << std::endl;
	}

}
