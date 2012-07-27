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
#include "ObjectOverlap_Pow2DTree.h"

using namespace psciob;

//3D Benchmarks

//
//Benchmark1
typedef PoseTransformedBinaryShape<3>::BaseClass BenchmarkObjectType;	
//typedef BaseScene<BenchmarkObjectType::Dimension, BenchmarkObjectType::TexturedPixelType, unsigned short, ObjectPriorCostAndPixelSetContainer, PixelSetIntersectionContainer> BenchmarkSceneType;
typedef BaseScene<BenchmarkObjectType::Dimension, BenchmarkObjectType::TexturedPixelType, unsigned short, ObjectPriorCostContainer, LabelObjectIntersectionContainer<unsigned short, BenchmarkObjectType::Dimension>> BenchmarkSceneType;
void Benchmark1(BenchmarkSceneType *scene3D) {
	clock_t t0 = clock();
	//shape3DSphere::Pointer sphereShape = shape3DSphere::New();
	//Translation3DTransform::Pointer translation3D = Translation3DTransform::New();
	//PoseTransformedBinaryShape<3>::Pointer sphere = PoseTransformedBinaryShape<3>::New();
	//vnl_vector<double> sphereRadius(1);	sphereRadius(0) = 2.215;	sphereShape->SetParameters(sphereRadius);
	//vnl_vector<double> trans3D(3);		trans3D.fill(0);			translation3D->SetParameters(trans3D);
	//sphere->SetShapeAndTransform(sphereShape, translation3D);
	Direct3DSphere::Pointer sphere = Direct3DSphere::New();

	//define the characteristics of the scene
	vnl_vector<double> sceneSpacing(3);	sceneSpacing.fill(0.3);
	vnl_vector<double> sceneBBox(6);	for (unsigned i=0 ; i<3 ; i++) { sceneBBox(2*i) = -10;	sceneBBox(2*i+1) = 10; }
	scene3D->SetPhysicalDimensions(sceneBBox, sceneSpacing);
	sphere->SetImageSpacing(sceneSpacing);

	//add a few objects to the scene
	vnl_vector<double> sphereParams(4);
	sphereParams(0) = 0;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	
	sphere->SetParameters(sphereParams);	
	BenchmarkSceneType::IDType id = scene3D->AddObject(sphere);

	sphereParams(0) = 1;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = -4.2;	sphereParams(1) = 2.3;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = 6;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);

	sphereParams(0) = -5;	sphereParams(1) = -5;	sphereParams(2) = 1;	sphereParams(3) = 2;
	sphere->SetParameters(sphereParams);
	for (unsigned i=0 ; i<500 ; i++) {
		id = scene3D->AddObject(sphere);
		scene3D->RemoveObject(id);
	}
	std::cout<<"  time for benchmark1 (add&remove): "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
}
void Benchmark2(BenchmarkSceneType *scene3D) {
	clock_t t0 = clock();
	//shape3DSphere::Pointer sphereShape = shape3DSphere::New();
	//Translation3DTransform::Pointer translation3D = Translation3DTransform::New();
	//PoseTransformedBinaryShape<3>::Pointer sphere = PoseTransformedBinaryShape<3>::New();
	//vnl_vector<double> sphereRadius(1);	sphereRadius(0) = 2.215;	sphereShape->SetParameters(sphereRadius);
	//vnl_vector<double> trans3D(3);		trans3D.fill(0);			translation3D->SetParameters(trans3D);
	//sphere->SetShapeAndTransform(sphereShape, translation3D);
	Direct3DSphere::Pointer sphere = Direct3DSphere::New();

	//define the characteristics of the scene
	vnl_vector<double> sceneSpacing(3);	sceneSpacing.fill(0.3);
	vnl_vector<double> sceneBBox(6);	for (unsigned i=0 ; i<3 ; i++) { sceneBBox(2*i) = -10;	sceneBBox(2*i+1) = 10; }
	scene3D->SetPhysicalDimensions(sceneBBox, sceneSpacing);
	sphere->SetImageSpacing(sceneSpacing);

	//add a few objects to the scene
	vnl_vector<double> sphereParams(4);
	sphereParams(0) = 0;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	
	sphere->SetParameters(sphereParams);	
	BenchmarkSceneType::IDType id = scene3D->AddObject(sphere);

	sphereParams(0) = 1;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = -4.2;	sphereParams(1) = 2.3;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = 6;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);

	sphereParams(0) = -5;	sphereParams(1) = -5;	sphereParams(2) = 1;	sphereParams(3) = 2;
	sphere->SetParameters(sphereParams);
	id = scene3D->AddObject(sphere);
	for (unsigned i=0 ; i<500 ; i++) {
		sphereParams(0) += 0.01;
		scene3D->ModifyObjectParameters(id, sphereParams);
	}
	std::cout<<"  time for benchmark2 (modify): "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
}
void Benchmark3(BenchmarkSceneType *scene3D) {
	clock_t t0 = clock();
	//shape3DSphere::Pointer sphereShape = shape3DSphere::New();
	//Translation3DTransform::Pointer translation3D = Translation3DTransform::New();
	//PoseTransformedBinaryShape<3>::Pointer sphere = PoseTransformedBinaryShape<3>::New();
	//vnl_vector<double> sphereRadius(1);	sphereRadius(0) = 2.215;	sphereShape->SetParameters(sphereRadius);
	//vnl_vector<double> trans3D(3);		trans3D.fill(0);			translation3D->SetParameters(trans3D);
	//sphere->SetShapeAndTransform(sphereShape, translation3D);
	Direct3DSphere::Pointer sphere = Direct3DSphere::New();

	//define the characteristics of the scene
	vnl_vector<double> sceneSpacing(3);	sceneSpacing.fill(0.3);
	vnl_vector<double> sceneBBox(6);	for (unsigned i=0 ; i<3 ; i++) { sceneBBox(2*i) = -10;	sceneBBox(2*i+1) = 10; }
	scene3D->SetPhysicalDimensions(sceneBBox, sceneSpacing);
	sphere->SetImageSpacing(sceneSpacing);

	//add a few objects to the scene
	vnl_vector<double> sphereParams(4);
	sphereParams(0) = 0;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	
	sphere->SetParameters(sphereParams);	
	BenchmarkSceneType::IDType id = scene3D->AddObject(sphere);

	sphereParams(0) = 1;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = -4.2;	sphereParams(1) = 2.3;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = 6;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);

	sphereParams(0) = -5;	sphereParams(1) = -5;	sphereParams(2) = 1;	sphereParams(3) = 2;
	sphere->SetParameters(sphereParams);
	//sphere->GetObjectAsLabelMap();
	sphere->GetObjectAsVTKPolyData();
	for (unsigned i=0 ; i<500 ; i++) {
		sphereParams(0) += 0.01;
		sphere->SetParameters(sphereParams);
		sphere->GetObjectAsVTKPolyData();
	}
	std::cout<<"  time for benchmark3 (modify only the object vtk polydata): "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
}

void Benchmark4(BenchmarkSceneType *scene3D) {
	clock_t t0 = clock();
	//shape3DSphere::Pointer sphereShape = shape3DSphere::New();
	//Translation3DTransform::Pointer translation3D = Translation3DTransform::New();
	//PoseTransformedBinaryShape<3>::Pointer sphere = PoseTransformedBinaryShape<3>::New();
	//vnl_vector<double> sphereRadius(1);	sphereRadius(0) = 2.215;	sphereShape->SetParameters(sphereRadius);
	//vnl_vector<double> trans3D(3);		trans3D.fill(0);			translation3D->SetParameters(trans3D);
	//sphere->SetShapeAndTransform(sphereShape, translation3D);
	Direct3DSphere::Pointer sphere = Direct3DSphere::New();

	//define the characteristics of the scene
	vnl_vector<double> sceneSpacing(3);	sceneSpacing.fill(0.3);
	vnl_vector<double> sceneBBox(6);	for (unsigned i=0 ; i<3 ; i++) { sceneBBox(2*i) = -10;	sceneBBox(2*i+1) = 10; }
	scene3D->SetPhysicalDimensions(sceneBBox, sceneSpacing);
	sphere->SetImageSpacing(sceneSpacing);

	//add a few objects to the scene
	vnl_vector<double> sphereParams(4);
	sphereParams(0) = 0;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	
	sphere->SetParameters(sphereParams);	
	BenchmarkSceneType::IDType id = scene3D->AddObject(sphere);

	sphereParams(0) = 1;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = -4.2;	sphereParams(1) = 2.3;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = 6;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);

	sphereParams(0) = -5;	sphereParams(1) = -5;	sphereParams(2) = 1;	sphereParams(3) = 2;
	sphere->SetParameters(sphereParams);
	sphere->GetObjectAsLabelMap();
	for (unsigned i=0 ; i<500 ; i++) {
		sphereParams(0) += 0.01;
		sphere->SetParameters(sphereParams);
		sphere->GetObjectAsLabelMap();
	}
	std::cout<<"  time for benchmark4 (modify the object pixelset): "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
}

//
void BenchmarkPixelSetWithoutIntersection() {
	std::cout<<"\n\n BenchmarkPixelSetWithoutIntersection"<<std::endl;
	typedef PoseTransformedBinaryShape<3>::BaseClass Base3DObjectType;	
	//typedef LabelMapScene<BenchmarkObjectType::Dimension, BenchmarkObjectType::TexturedPixelType, unsigned short, ObjectPriorCostAndPixelSetContainer, PixelSetIntersectionContainer> Scene3DType;
	typedef LabelMapScene<BenchmarkObjectType::Dimension, BenchmarkObjectType::TexturedPixelType, unsigned short, ObjectPriorCostContainer, LabelObjectIntersectionContainer<unsigned short, 3>> Scene3DType;
	Scene3DType::Pointer scene3D = Scene3DType::New();
	Benchmark1(scene3D); Benchmark2(scene3D); Benchmark3(scene3D); Benchmark4(scene3D);
}
//
void BenchmarkPixelSetWithIntersection() {
	std::cout<<"\n\n BenchmarkPixelSetWithIntersection"<<std::endl;
	typedef PoseTransformedBinaryShape<3>::BaseClass Base3DObjectType;	
	//typedef LabelMapScene<BenchmarkObjectType::Dimension, BenchmarkObjectType::TexturedPixelType, unsigned short, ObjectPriorCostAndPixelSetContainer, PixelSetIntersectionContainer> Scene3DType;
	typedef LabelMapScene<BenchmarkObjectType::Dimension, BenchmarkObjectType::TexturedPixelType, unsigned short, ObjectPriorCostContainer, LabelObjectIntersectionContainer<unsigned short, 3>> Scene3DType;
	Scene3DType::Pointer scene3D = Scene3DType::New();
	//typedef PixelSetObjectOverlap<Scene3DType> InteractionManagerType;
	typedef LabelObjectOverlap<Scene3DType> InteractionManagerType;
	InteractionManagerType::Pointer interactionManager = InteractionManagerType::New();
	scene3D->SetInteractionManager( (Scene3DType::ObjectInteractionManagerType *)interactionManager.GetPointer() );
	Benchmark1(scene3D); Benchmark2(scene3D); Benchmark3(scene3D); Benchmark4(scene3D);
}
//
void BenchmarkLabelImageIntersection() {
	std::cout<<"\n\n BenchmarkLabelImageWithIntersection"<<std::endl;
	typedef PoseTransformedBinaryShape<3>::BaseClass Base3DObjectType;	
	//typedef LabelImageScene<BenchmarkObjectType::Dimension, BenchmarkObjectType::TexturedPixelType, unsigned short, ObjectPriorCostAndPixelSetContainer, PixelSetIntersectionContainer> Scene3DType;
	typedef LabelImageScene<BenchmarkObjectType::Dimension, BenchmarkObjectType::TexturedPixelType, unsigned short, ObjectPriorCostContainer, LabelObjectIntersectionContainer<unsigned short, 3>> Scene3DType;
	Scene3DType::Pointer scene3D = Scene3DType::New();
	Benchmark1(scene3D); Benchmark2(scene3D); Benchmark3(scene3D); Benchmark4(scene3D);
}

//
//
// 2D BENCHMARKS 
//
//
//typedef BaseScene<2, unsigned char, unsigned short, ObjectPriorCostAndPixelSetContainer, PixelSetIntersectionContainer> Benchmark2DSceneType;
typedef BaseScene<2, unsigned char, unsigned short, ObjectPriorCostContainer, LabelObjectIntersectionContainer<unsigned short, 2>> Benchmark2DSceneType;
void Benchmark2D_0(Benchmark2DSceneType *scene) {
	std::cout<<"Benchmark2D_0: clone 10000 FastEllipse2D, feed a vector of ObjectInScene, and update the labelmap representation -- ";
	clock_t t0 = clock();
	Fast2DEllipse::Pointer ellipse = Fast2DEllipse::New();
	Fast2DEllipse::BaseClass::Pointer toto;
	Benchmark2DSceneType::DeformableObjectType::Pointer tutu;
	std::vector<Benchmark2DSceneType::ObjectInScene> collect;
	vnl_vector<double> params;
	for (unsigned i=0 ; i<10000 ; i++) {
		//toto = ellipse->CreateClone();
		//params = scene->GetObjectTypesLibrary()->GetObjectPDF(0, PDF_OBJECTGENERATIONPRIOR)->DrawSample();
		//toto->SetParameters(params);
		
		tutu = scene->GetObjectTypesLibrary()->GenerateNewRandomObject(0);
		
		////toto->GetObjectAsLabelMap();
		//Benchmark2DSceneType::ObjectInScene tata;
		//tata.obj = toto;
		//collect.push_back(tata);
		//collect.push_back(Benchmark2DSceneType::ObjectInScene());
		//collect[i] = tata;
		//collect[i].obj->GetObjectAsLabelMap();
	}
	std::cout<<" took: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
}
//
void Benchmark2D_1(Benchmark2DSceneType *scene) {
	std::cout<<"Benchmark2D_1: generate the VTKPolyData representation (10000 times on the same instance with rnd params) -- ";
	clock_t t0 = clock();
	Benchmark2DSceneType::DeformableObjectType::Pointer obj;
	for (unsigned i=0 ; i<10000 ; i++) {
		obj = scene->GetObjectTypesLibrary()->GenerateNewRandomObject(0);
		obj->GetObjectAsVTKPolyData();
	}
	std::cout<<" took: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
}
//
void Benchmark2D_2(Benchmark2DSceneType *scene) {
	std::cout<<"Benchmark2D_2: generate the LabelMap representation (10000 times on the same instance with rnd params) -- ";
	clock_t t0 = clock();
	Benchmark2DSceneType::DeformableObjectType::Pointer obj;
	for (unsigned i=0 ; i<10000 ; i++) {
		obj = scene->GetObjectTypesLibrary()->GenerateNewRandomObject(0);
		obj->GetObjectAsLabelMap();
	}
	std::cout<<" took: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
}
//
void Benchmark2D_3(Benchmark2DSceneType *scene) {
	std::cout<<"Benchmark2D_3: manipulate the object through the scene (add 10 objects, modify 9880, add&remove 110) -- ";
	clock_t t0 = clock();
	Benchmark2DSceneType::DeformableObjectType::Pointer obj;
	RandomVariableGenerator::Pointer rndgen = RandomVariableGenerator::New();
	
	vnl_vector<double> params;
	Benchmark2DSceneType::IDType id; unsigned int requestedIndex; SceneObjectIterator<Benchmark2DSceneType> objectIt(scene);
	for (unsigned i=0 ; i<10 ; i++) {
		obj = scene->GetObjectTypesLibrary()->GenerateNewRandomObject(0);
		scene->AddObject(obj);
	}
	for (unsigned i=0 ; i<9880 ; i++) {
		//select object...
		params = scene->GetObjectTypesLibrary()->GetObjectPDF(0, PDF_OBJECTGENERATIONPRIOR)->DrawSample();
		requestedIndex = rndgen->GetIntegerVariate(scene->GetNumberOfObjects()-1);
		objectIt.GoToBegin(); objectIt.Advance(requestedIndex); id = objectIt.GetID();
		scene->ModifyObjectParameters(id , params);
	}
	//add & remove...
	for (unsigned i=0 ; i<110 ; i++) {
		//add an object
		obj = scene->GetObjectTypesLibrary()->GenerateNewRandomObject(0);
		scene->AddObject(obj);
		//remove a random object
		requestedIndex = rndgen->GetIntegerVariate(scene->GetNumberOfObjects()-1);
		objectIt.GoToBegin(); objectIt.Advance(requestedIndex); id = objectIt.GetID();
		scene->RemoveObject( id );
	}

	std::cout<<" took: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
}
//
void Benchmark2D_4(Benchmark2DSceneType *scene) {
	std::cout<<"Benchmark2D_4: manipulate the object through the scene (add 10 objects, add&remove 10000) -- ";
	clock_t t0 = clock();
	Benchmark2DSceneType::DeformableObjectType::Pointer obj;
	RandomVariableGenerator::Pointer rndgen = RandomVariableGenerator::New();
	
	vnl_vector<double> params;
	Benchmark2DSceneType::IDType id; unsigned int requestedIndex; SceneObjectIterator<Benchmark2DSceneType> objectIt(scene);
	for (unsigned i=0 ; i<10 ; i++) {
		obj = scene->GetObjectTypesLibrary()->GenerateNewRandomObject(0);
		scene->AddObject(obj);
	}
	for (unsigned i=0 ; i<10000 ; i++) {
		//add an object
		obj = scene->GetObjectTypesLibrary()->GenerateNewRandomObject(0);
		scene->AddObject(obj);
		//remove a random object
		requestedIndex = rndgen->GetIntegerVariate(scene->GetNumberOfObjects()-1);
		objectIt.GoToBegin(); objectIt.Advance(requestedIndex); id = objectIt.GetID();
		scene->RemoveObject( id );
	}

	std::cout<<" took: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
}
//
//
void BenchmarkFastEllipse_DummyScene() {
	std::cout<<"\n\n------------ BenchmarkFastEllipse_DummyScene ----------"<<std::endl;
	//typedef DummyScene<2, unsigned char, unsigned short, ObjectPriorCostAndPixelSetContainer, PixelSetIntersectionContainer> SceneType;
	typedef DummyScene<2, unsigned char, unsigned short, ObjectPriorCostContainer, LabelObjectIntersectionContainer<unsigned short, 2>> SceneType;
	SceneType::Pointer scene = SceneType::New(); vnl_vector<double> sceneBBox(4); sceneBBox(0) = 0; sceneBBox(1) = 100; sceneBBox(2) = 0; sceneBBox(3) = 100; scene->SetPhysicalDimensions(sceneBBox, 1);
	Fast2DEllipse::Pointer ellipse = Fast2DEllipse::New();
	unsigned int typeCode = scene->GetObjectTypesLibrary()->RegisterObjectType(ellipse);

	UniformBoxPDF::Pointer uniformTransPDF = UniformBoxPDF::New(); uniformTransPDF->SetBox(scene->GetSceneBoundingBox());
	UniformPDF::Pointer    uniformRadiusPDF = UniformPDF::New();   uniformRadiusPDF->SetParameters(2, 10);
	UniformPDF::Pointer    uniformElongPDF = UniformPDF::New();    uniformElongPDF->SetParameters(0.45, 1);
	UniformPDF::Pointer    uniformAnglePDF = UniformPDF::New();    uniformAnglePDF->SetParameters(0, PI);
	IndependentPDFs::Pointer ellipseGenerationPDF = IndependentPDFs::New();
	ellipseGenerationPDF->AddMultivariatePDF(uniformTransPDF); ellipseGenerationPDF->AddUnivariatePDF(uniformRadiusPDF);
	ellipseGenerationPDF->AddUnivariatePDF(uniformElongPDF);   ellipseGenerationPDF->AddUnivariatePDF(uniformAnglePDF);

	scene->GetObjectTypesLibrary()->SetObjectPDF(typeCode, PDF_OBJECTGENERATIONPRIOR, ellipseGenerationPDF);

	Benchmark2D_0(scene); Benchmark2D_1(scene); Benchmark2D_2(scene); Benchmark2D_3(scene); Benchmark2D_4(scene);
}
//
void BenchmarkFastEllipse_PixelSetWithoutIntersection() {
	std::cout<<"\n\n------------ BenchmarkFastEllipse_LabelMapWithoutIntersection ----------"<<std::endl;
	//typedef LabelMapScene<2, unsigned char, unsigned short, ObjectPriorCostAndPixelSetContainer, PixelSetIntersectionContainer> SceneType;
	typedef LabelMapScene<2, unsigned char, unsigned short, ObjectPriorCostContainer, LabelObjectIntersectionContainer<unsigned short, 2>> SceneType;
	SceneType::Pointer scene = SceneType::New(); vnl_vector<double> sceneBBox(4); sceneBBox(0) = 0; sceneBBox(1) = 100; sceneBBox(2) = 0; sceneBBox(3) = 100; scene->SetPhysicalDimensions(sceneBBox, 1);
	Fast2DEllipse::Pointer ellipse = Fast2DEllipse::New();
	unsigned int typeCode = scene->GetObjectTypesLibrary()->RegisterObjectType(ellipse);

	UniformBoxPDF::Pointer uniformTransPDF = UniformBoxPDF::New(); uniformTransPDF->SetBox(scene->GetSceneBoundingBox());
	UniformPDF::Pointer    uniformRadiusPDF = UniformPDF::New();   uniformRadiusPDF->SetParameters(2, 10);
	UniformPDF::Pointer    uniformElongPDF = UniformPDF::New();    uniformElongPDF->SetParameters(0.45, 1);
	UniformPDF::Pointer    uniformAnglePDF = UniformPDF::New();    uniformAnglePDF->SetParameters(0, PI);
	IndependentPDFs::Pointer ellipseGenerationPDF = IndependentPDFs::New();
	ellipseGenerationPDF->AddMultivariatePDF(uniformTransPDF); ellipseGenerationPDF->AddUnivariatePDF(uniformRadiusPDF);
	ellipseGenerationPDF->AddUnivariatePDF(uniformElongPDF);   ellipseGenerationPDF->AddUnivariatePDF(uniformAnglePDF);

	scene->GetObjectTypesLibrary()->SetObjectPDF(typeCode, PDF_OBJECTGENERATIONPRIOR, ellipseGenerationPDF);

	Benchmark2D_0(scene); Benchmark2D_1(scene); Benchmark2D_2(scene); Benchmark2D_3(scene); Benchmark2D_4(scene);
}
void BenchmarkFastEllipse_PixelSetWithIntersection() {
	std::cout<<"\n\n------------ BenchmarkFastEllipse_LabelMapWithIntersection ----------"<<std::endl;
	//typedef LabelMapScene<2, unsigned char, unsigned short, ObjectPriorCostAndPixelSetContainer, PixelSetIntersectionContainer> SceneType;
	typedef LabelMapScene<2, unsigned char, unsigned short, ObjectPriorCostContainer, LabelObjectIntersectionContainer<unsigned short, 2>> SceneType;
	SceneType::Pointer scene = SceneType::New(); vnl_vector<double> sceneBBox(4); sceneBBox(0) = 0; sceneBBox(1) = 1000; sceneBBox(2) = 0; sceneBBox(3) = 1000; scene->SetPhysicalDimensions(sceneBBox, 1);
	//PixelSetObjectOverlap<SceneType::BaseClass>::Pointer interactionManager = PixelSetObjectOverlap<SceneType::BaseClass>::New();
	//ObjectOverlap_Pow2DTree<SceneType::BaseClass,PixelSetObjectOverlap<SceneType::BaseClass>>::Pointer interactionManager = ObjectOverlap_Pow2DTree<SceneType::BaseClass,PixelSetObjectOverlap<SceneType::BaseClass>>::New();
	ObjectOverlap_Pow2DTree<SceneType::BaseClass,LabelObjectOverlap<SceneType::BaseClass>>::Pointer interactionManager = ObjectOverlap_Pow2DTree<SceneType::BaseClass,LabelObjectOverlap<SceneType::BaseClass>>::New();
	scene->SetInteractionManager(interactionManager);
	Fast2DEllipse::Pointer ellipse = Fast2DEllipse::New();
	unsigned int typeCode = scene->GetObjectTypesLibrary()->RegisterObjectType(ellipse);

	UniformBoxPDF::Pointer uniformTransPDF = UniformBoxPDF::New(); uniformTransPDF->SetBox(scene->GetSceneBoundingBox());
	UniformPDF::Pointer    uniformRadiusPDF = UniformPDF::New();   uniformRadiusPDF->SetParameters(20, 100);
	UniformPDF::Pointer    uniformElongPDF = UniformPDF::New();    uniformElongPDF->SetParameters(0.45, 1);
	UniformPDF::Pointer    uniformAnglePDF = UniformPDF::New();    uniformAnglePDF->SetParameters(0, PI);
	IndependentPDFs::Pointer ellipseGenerationPDF = IndependentPDFs::New();
	ellipseGenerationPDF->AddMultivariatePDF(uniformTransPDF); ellipseGenerationPDF->AddUnivariatePDF(uniformRadiusPDF);
	ellipseGenerationPDF->AddUnivariatePDF(uniformElongPDF);   ellipseGenerationPDF->AddUnivariatePDF(uniformAnglePDF);

	scene->GetObjectTypesLibrary()->SetObjectPDF(typeCode, PDF_OBJECTGENERATIONPRIOR, ellipseGenerationPDF);

	Benchmark2D_0(scene); Benchmark2D_1(scene); Benchmark2D_2(scene); Benchmark2D_3(scene); Benchmark2D_4(scene);
}
void BenchmarkFastEllipse_LabelImageIntersection() {
	std::cout<<"\n\n------------ BenchmarkFastEllipse_LabelImageIntersection ----------"<<std::endl;
	//typedef LabelImageScene<2, unsigned char, unsigned short, ObjectPriorCostAndPixelSetContainer, PixelSetIntersectionContainer> SceneType;
	typedef LabelImageScene<2, unsigned char, unsigned short, ObjectPriorCostContainer, LabelObjectIntersectionContainer<unsigned short, 2>> SceneType;
	SceneType::Pointer scene = SceneType::New(); vnl_vector<double> sceneBBox(4); sceneBBox(0) = 0; sceneBBox(1) = 1000; sceneBBox(2) = 0; sceneBBox(3) = 1000; scene->SetPhysicalDimensions(sceneBBox, 1);
	Fast2DEllipse::Pointer ellipse = Fast2DEllipse::New();
	unsigned int typeCode = scene->GetObjectTypesLibrary()->RegisterObjectType(ellipse);

	UniformBoxPDF::Pointer uniformTransPDF = UniformBoxPDF::New(); uniformTransPDF->SetBox(scene->GetSceneBoundingBox());
	UniformPDF::Pointer    uniformRadiusPDF = UniformPDF::New();   uniformRadiusPDF->SetParameters(20, 100);
	UniformPDF::Pointer    uniformElongPDF = UniformPDF::New();    uniformElongPDF->SetParameters(0.45, 1);
	UniformPDF::Pointer    uniformAnglePDF = UniformPDF::New();    uniformAnglePDF->SetParameters(0, PI);
	IndependentPDFs::Pointer ellipseGenerationPDF = IndependentPDFs::New();
	ellipseGenerationPDF->AddMultivariatePDF(uniformTransPDF); ellipseGenerationPDF->AddUnivariatePDF(uniformRadiusPDF);
	ellipseGenerationPDF->AddUnivariatePDF(uniformElongPDF);   ellipseGenerationPDF->AddUnivariatePDF(uniformAnglePDF);

	scene->GetObjectTypesLibrary()->SetObjectPDF(typeCode, PDF_OBJECTGENERATIONPRIOR, ellipseGenerationPDF);

	Benchmark2D_0(scene); Benchmark2D_1(scene); Benchmark2D_2(scene); Benchmark2D_3(scene); Benchmark2D_4(scene);
}

//
#include "Ellipses2D_Synthese.h"
#include "SimpleUniformSensor.h"
#include "SensedSingleObjectInOutMMahalanobisDistance.h"
#include <SensedSingleObjectToImageMeanAbsoluteDifferenceCostFunction.h>
void BenchmarkLabelImage_FastEllipse_SimpleSensor_OffContextMBhatCost() {
	std::cout<<"\n\n------------ BenchmarkLabelImage_FastEllipse_SimpleSensor_OffContextMBhatCost ----------"<<std::endl;
	
	//1:generate the reference image
	unsigned int sceneCode = 1;	double       blurSize  = 0;	double       noisePower= 0;	int          rndSeed   = 0;

	Ellipses2D_Synthese sceneGenerator;
	switch(sceneCode) {
			case 1: sceneGenerator.GenerateScene_1(); break;
			default: throw DeformableModelException("unknown scene code..."); break;
	}
	sceneGenerator.SetBlurStd(blurSize);
	sceneGenerator.SetNoiseStd(noisePower);
	sceneGenerator.SetNoiseSeed(rndSeed);

	typedef Ellipses2D_Synthese::ImageType ReferenceImageType;
	Ellipses2D_Synthese::ImageType *referenceImage = sceneGenerator.GetImage();

	//2:define the scene and a generation prior for ellipses
	//typedef LabelImageScene<2, unsigned char, unsigned short, ObjectCostsAndPixelSetContainer, PixelSetIntersectionContainer> SceneType;
	typedef LabelImageScene<2, unsigned char, unsigned short, ObjectCostsContainer, LabelObjectIntersectionContainer<unsigned short, 2>> SceneType;
	SceneType::Pointer scene = SceneType::New(); vnl_vector<double> sceneBBox = sceneGenerator.GetScenePointer()->GetSceneBoundingBox(); scene->SetPhysicalDimensions(sceneBBox, 1);
	Fast2DEllipse::Pointer ellipse = Fast2DEllipse::New();
	unsigned int typeCode = scene->GetObjectTypesLibrary()->RegisterObjectType(ellipse);

	UniformBoxPDF::Pointer uniformTransPDF = UniformBoxPDF::New(); uniformTransPDF->SetBox(scene->GetSceneBoundingBox());
	UniformPDF::Pointer    uniformRadiusPDF = UniformPDF::New();   uniformRadiusPDF->SetParameters(2, 10);
	UniformPDF::Pointer    uniformElongPDF = UniformPDF::New();    uniformElongPDF->SetParameters(0.45, 1);
	UniformPDF::Pointer    uniformAnglePDF = UniformPDF::New();    uniformAnglePDF->SetParameters(0, PI);
	IndependentPDFs::Pointer ellipseGenerationPDF = IndependentPDFs::New();
	ellipseGenerationPDF->AddMultivariatePDF(uniformTransPDF); ellipseGenerationPDF->AddUnivariatePDF(uniformRadiusPDF);
	ellipseGenerationPDF->AddUnivariatePDF(uniformElongPDF);   ellipseGenerationPDF->AddUnivariatePDF(uniformAnglePDF);

	scene->GetObjectTypesLibrary()->SetObjectPDF(typeCode, PDF_OBJECTGENERATIONPRIOR, ellipseGenerationPDF);

	//3: define a sensor
	typedef SimpleUniformSensor<SceneType, ReferenceImageType> SensorType;
	SensorType::Pointer sensor = SensorType::New();
	sensor->SetScene(scene);
	vnl_vector<double> sensorAppearanceParam(1); sensorAppearanceParam(0) = 200;
	sensor->SetAppearanceParameters(sensorAppearanceParam);

	//4: define the metric
	typedef SensedSingleObjectInOutMMahalanobisDistance<SceneType, ReferenceImageType, SensorType::OutputImageType> SingleObjectCostFunctionType;
	//typedef SensedSingleObjectToImageMeanAbsoluteDifferenceCostFunction<SceneType, ReferenceImageType, SensorType::OutputImageType> SingleObjectCostFunctionType;
	SingleObjectCostFunctionType::Pointer singleObjectCostFunction = SingleObjectCostFunctionType::New();
	singleObjectCostFunction->SetReferenceImage( referenceImage );
	singleObjectCostFunction->SetSensor(sensor);
	singleObjectCostFunction->SelectOffContextComputation();
	singleObjectCostFunction->SetSurroundingRadius(2);
	//normalization of the cost function
	GPF_LinearRescaling<double, double>::Pointer normalizationFunction = GPF_LinearRescaling<double, double>::New();
	vnl_vector<double> normFParams(2); normFParams(0) = -5.5; normFParams(1)=1;		
	normalizationFunction->SetParameters(normFParams);
	singleObjectCostFunction->SetNormalizationFunction(normalizationFunction);

	//5: do the real job: generate objects, and compute the cost..
	SceneType::DeformableObjectType::Pointer obj;
	RandomVariableGenerator::Pointer rndgen = RandomVariableGenerator::New();

	vnl_vector<double> params;
	SceneType::IDType id; unsigned int requestedIndex; SceneObjectIterator<SceneType> objectIt(scene);
	
	std::cout<<"Benchmark2D: LabelImageScene + FastEllipse + SimpleSensor + MBhatOffContextCost, \nadd 10 obj, modify 9880, add&remove 110, compute cost each time-- "<<std::endl;
	clock_t t0 = clock();
	for (unsigned i=0 ; i<10 ; i++) {
		obj = scene->GetObjectTypesLibrary()->GenerateNewRandomObject(0);
		id = scene->AddObject(obj);
		std::cout<<"added object with label: "<<id<<", params: "<<scene->GetParametersOfObject(id)<<std::endl;
		singleObjectCostFunction->SelectObject(id); 
		std::cout<<" getting value..."<<std::endl;
		singleObjectCostFunction->GetValue();
		singleObjectCostFunction->PrintDetails();
	}
	std::cout<<"Benchmark2D: add 10... OK ";

	for (unsigned i=0 ; i<9880 ; i++) {
		//select object...
		params = scene->GetObjectTypesLibrary()->GetObjectPDF(0, PDF_OBJECTGENERATIONPRIOR)->DrawSample();
		requestedIndex = rndgen->GetIntegerVariate(scene->GetNumberOfObjects()-1);
		objectIt.GoToBegin(); objectIt.Advance(requestedIndex); id = objectIt.GetID();
		scene->ModifyObjectParameters(id , params);
		singleObjectCostFunction->SelectObject(id); singleObjectCostFunction->GetValue();
	}
	//add & remove...
	for (unsigned i=0 ; i<110 ; i++) {
		//add an object
		obj = scene->GetObjectTypesLibrary()->GenerateNewRandomObject(0);
		id = scene->AddObject(obj);
		singleObjectCostFunction->SelectObject(id); singleObjectCostFunction->GetValue();
		//remove a random object
		requestedIndex = rndgen->GetIntegerVariate(scene->GetNumberOfObjects()-1);
		objectIt.GoToBegin(); objectIt.Advance(requestedIndex); id = objectIt.GetID();
		scene->RemoveObject( id );
	}

	std::cout<<" took: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
}

//
int main(int argc, char** argv) {

	try {
		BenchmarkPixelSetWithoutIntersection();
		BenchmarkPixelSetWithIntersection();
		BenchmarkLabelImageIntersection();

		BenchmarkFastEllipse_DummyScene();
		BenchmarkFastEllipse_PixelSetWithoutIntersection();
		BenchmarkFastEllipse_PixelSetWithIntersection();
		BenchmarkFastEllipse_LabelImageIntersection();

		BenchmarkLabelImage_FastEllipse_SimpleSensor_OffContextMBhatCost();
	}
	catch (DeformableModelException& e) {
		std::cout << "Exception occured while running the test cases" << std::endl;
		std::cout << e.what() << std::endl;
	}

}
