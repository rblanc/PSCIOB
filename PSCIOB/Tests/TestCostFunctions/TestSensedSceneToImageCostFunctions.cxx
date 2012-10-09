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

#include "VTKScene.h"

#include "SimpleSEMSensor.h"

#include "SensedSceneToImageMeanSquareErrorCostFunction.h"


#include "shape2DDisk.h"
#include "Translation2DTransform.h"
#include "SimpleUniformSensor.h"

#include "SensedSingleObjectInOutContrast.h"
#include "SensedSingleObjectToImageMeanSquareErrorCostFunction.h"

using namespace psciob;

void GenerateReferenceDiskImage() {

	shape2DDisk::Pointer diskShape = shape2DDisk::New();
	Translation2DTransform::Pointer translation2D = Translation2DTransform::New();
	PoseTransformedBinaryShape<2>::Pointer disk = PoseTransformedBinaryShape<2>::New();

	vnl_vector<double> diskRadius(1); diskRadius(0) = 2.1; diskShape->SetParameters(diskRadius);
	vnl_vector<double> trans2D(2);    trans2D.fill(0);     translation2D->SetParameters(trans2D);
	disk->SetShapeAndTransform(diskShape, translation2D);

	typedef PoseTransformedBinaryShape<2>::BaseClass Base2DObjectType;

	typedef VTKScene<2> Scene2DType;

	Scene2DType::Pointer scene2D = Scene2DType::New();
	//define the characteristics of the scene
	vnl_vector<double> sceneSpacing(2);	sceneSpacing.fill(1);
	vnl_vector<double> sceneBBox(4);	for (unsigned i=0 ; i<2 ; i++) { sceneBBox(2*i) = -10;	sceneBBox(2*i+1) = 10; }
	scene2D->SetPhysicalDimensions(sceneBBox, sceneSpacing);

	//add a few objects to the scene
	vnl_vector<double> diskParams(3);
	diskParams(0) = 0;   diskParams(1) = 0;   diskParams(2) = 2;   disk->SetParameters(diskParams);	scene2D->AddObject(disk);
	diskParams(0) = 1;   diskParams(1) = 1;   diskParams(2) = 1.5; disk->SetParameters(diskParams);	scene2D->AddObject(disk);
	diskParams(0) =-4.2; diskParams(1) = 2.3; diskParams(2) = 2;   disk->SetParameters(diskParams);	scene2D->AddObject(disk);

	typedef itk::Image<unsigned char, 2> Image2DType;
	typedef SimpleUniformSensor<Scene2DType, Image2DType> SensorType;
	SensorType::Pointer sensor2D = SensorType::New();
	sensor2D->SetScene(scene2D);

	Write2DGreyLevelRescaledImageToFile<Image2DType>("referenceDiskImage.png", sensor2D->GetOutputImage());
	
	Scene2DType::LabelMapType::Pointer labelMap = Scene2DType::LabelMapType::New();
	std::cout<<"getting off context labelmap"<<std::endl;
	sensor2D->GetOffContextObjectLabelMap( scene2D->GetObject(1), labelMap );
	std::cout<<"nb of label Objects: "<<labelMap->GetNumberOfLabelObjects()<<std::endl;
}


void TestMeanSquareSensedSceneToImageCostFunction() {
	std::cout<<"\n\n TestMeanSquareSensedSceneToImageCostFunction: draw a few spheres, use a SimpleSEMSensor to capture images ; save that image ; create a second "<<std::endl;

	GenerateReferenceDiskImage();
	typedef itk::Image<unsigned char, 2> Image2DUCharType;
	typedef itk::Image<float, 2> Image2DFloatType;
	Image2DUCharType::Pointer referenceImage = ReadITKImageFromFile<Image2DUCharType>("referenceDiskImage.png", 1 );

	shape3DSphere::Pointer sphereShape = shape3DSphere::New();
	Translation3DTransform::Pointer translation3D = Translation3DTransform::New();
	PoseTransformedBinaryShape<3>::Pointer sphere = PoseTransformedBinaryShape<3>::New();

	vnl_vector<double> sphereRadius(1);	sphereRadius(0) = 2.215;	sphereShape->SetParameters(sphereRadius);
	vnl_vector<double> trans3D(3);		trans3D.fill(0);			translation3D->SetParameters(trans3D);
	sphere->SetShapeAndTransform(sphereShape, translation3D);

	typedef PoseTransformedBinaryShape<3>::BaseClass Base3DObjectType;	

	typedef VTKScene<3> Scene3DType;
	//typedef itk::Image<unsigned char, 2> Image2DType;	typedef itk::Image<unsigned char, 3> Image3DType;

	Scene3DType::Pointer scene3D = Scene3DType::New();
	//define the characteristics of the scene
	vnl_vector<double> sceneSpacing(3);
	sceneSpacing(0) = referenceImage->GetSpacing()[0];
	sceneSpacing(1) = referenceImage->GetSpacing()[1];
	sceneSpacing(2) = 1;
	vnl_vector<double> sceneBBox(6);	
	sceneBBox(0) = 0; sceneBBox(1) = referenceImage->GetLargestPossibleRegion().GetSize()[0]*sceneSpacing(0);
	sceneBBox(2) = 0; sceneBBox(3) = referenceImage->GetLargestPossibleRegion().GetSize()[1]*sceneSpacing(1); //
	sceneBBox(4) =-5; sceneBBox(5) = 5;

	scene3D->SetPhysicalDimensions(sceneBBox, sceneSpacing);
	std::cout<<"sceneBBox: "<<sceneBBox<<std::endl;
	std::cout<<"sceneSpacing: "<<sceneSpacing<<std::endl;
	//add a few objects to the scene
	vnl_vector<double> sphereParams(4);
clock_t t0=clock();
	sphereParams(0) = 10; sphereParams(1) = 10; sphereParams(2) = 0; sphereParams(3) = 2;   sphere->SetParameters(sphereParams); scene3D->AddObject(sphere);
	sphereParams(0) = 11; sphereParams(1) = 11; sphereParams(2) = 0; sphereParams(3) = 1.5; sphere->SetParameters(sphereParams); scene3D->AddObject(sphere);
	sphereParams(0) = 5; sphereParams(1) = 12.5; sphereParams(2) = 0; sphereParams(3) = 2;   sphere->SetParameters(sphereParams); scene3D->AddObject(sphere);

	typedef SimpleSEMSensor<Scene3DType, Image2DFloatType> SensorType;
	SensorType::Pointer sensor3D = SensorType::New();
	sensor3D->SetScene(scene3D);
	sensor3D->SetObservationDirectionType(HORIZONTAL);
	vnl_vector<double> appearanceParams(2); appearanceParams(0)=250;appearanceParams(1)=1.0/10.0; sensor3D->SetAppearanceParameters(appearanceParams);


	//GENERATE A REFERENCE IMAGE... with possibly different resolution, ...
	typedef SensedSceneToImageMeanSquareErrorCostFunction<Scene3DType, Image2DUCharType, Image2DFloatType> WholeImageCostFunctionType;
	WholeImageCostFunctionType::Pointer costFunction = WholeImageCostFunctionType::New();

	costFunction->SetScene(scene3D);
	costFunction->SetReferenceImage(referenceImage);
	costFunction->SetSensor(sensor3D);

	std::cout<<"measuring the cost..."<<std::endl;
	std::cout<<"value = "<<costFunction->GetValue()<<std::endl;
std::cout<<"  time to set 3 objects and get the value: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
t0=clock();
	sphereParams(0) = 6; sphereParams(1) = 2; sphereParams(2) = 0; sphereParams(3) = 2; sphere->SetParameters(sphereParams); scene3D->AddObject(sphere);
	sphereParams(0) = 12; sphereParams(1) = 2; sphereParams(2) = 0; sphereParams(3) = 4; sphere->SetParameters(sphereParams); scene3D->AddObject(sphere);
	std::cout<<"measuring the cost again, with a worst state..."<<std::endl;
	std::cout<<"value = "<<costFunction->GetValue()<<std::endl;
std::cout<<"  time to add 2 objects and get the value again: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;		
	Write2DGreyLevelRescaledImageToFile<Image2DFloatType>("testImage.png", sensor3D->GetOutputImage());


	//image-based term
	//typedef SensedSingleObjectInOutContrast<Scene3DType, Image2DUCharType, Image2DFloatType> SingleObjectCostFunctionType;
	typedef SensedSingleObjectToImageMeanSquareErrorCostFunction<Scene3DType, Image2DUCharType, Image2DFloatType> SingleObjectCostFunctionType;
	SingleObjectCostFunctionType::Pointer sooCostFunction = SingleObjectCostFunctionType::New();
	sooCostFunction->SetScene(scene3D);
	sooCostFunction->SetReferenceImage(referenceImage);
	sooCostFunction->SetSensor(sensor3D);
	sooCostFunction->SelectOffContextComputation();
	//sooCostFunction->SelectInContextComputation();
	sooCostFunction->SelectObject(3);

	double val;
	//
	std::cout<<"loop modifying back and forth the object... just to check for memory leak..."<<std::endl;
	for (unsigned i=0 ; i<1000 ; i++) {
		sphereParams(0) = 5; sphereParams(1) = 12.5; sphereParams(2) = 0; sphereParams(3) = 6;
		if (!scene3D->ModifyObjectParameters(3, sphereParams)) std::cout<<"pb modifying the object"<<std::endl;
		val = costFunction->GetValue();
		val = sooCostFunction->GetValue();
		if (i==0) {
			std::cout<<"single object metric: "<<val<<std::endl;
			std::cout<<"1-params of object 3: "<<scene3D->GetParametersOfObject(3)<<std::endl;
			Write2DGreyLevelRescaledImageToFile<Image2DFloatType>("testImage_1.png", sensor3D->GetOutputImage());
			Write2DGreyLevelRescaledImageToFile<WholeImageCostFunctionType::ReferenceImageType>("testImage_ref1.png", costFunction->GetInterpolatedReferenceImage());
		}

		sphereParams(0) = 5; sphereParams(1) = 12.5; sphereParams(2) = 0; sphereParams(3) = 2;
		if (!scene3D->ModifyObjectParameters(3, sphereParams)) std::cout<<"pb modifying the object"<<std::endl;
		val = costFunction->GetValue();
		val = sooCostFunction->GetValue();
		if (i==0) {
			std::cout<<"single object metric: "<<val<<std::endl;
			std::cout<<"2-params of object 3: "<<scene3D->GetParametersOfObject(3)<<std::endl;
			Write2DGreyLevelRescaledImageToFile<Image2DFloatType>("testImage_2.png", sensor3D->GetOutputImage());
			Write2DGreyLevelRescaledImageToFile<WholeImageCostFunctionType::ReferenceImageType>("testImage_ref2.png", costFunction->GetInterpolatedReferenceImage());
		}
		if (i==10) std::cout<<"10 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
		if (i==50) std::cout<<"50 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;	
		if (i==100) std::cout<<"100 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
		if (i==500) std::cout<<"500 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
	}
	//TODO: check for 2D, check also that modifications of object parameters also work.



	std::cout<<"\n\n TestMeanSquareSensedSceneToImageCostFunction: completed successfully "<<std::endl;
}




int main(int argc, char** argv) {

	try {
		TestMeanSquareSensedSceneToImageCostFunction();
	}
	catch (DeformableModelException& e) {
		std::cout << "Exception occured while running the test cases" << std::endl;
		std::cout << e.what() << std::endl;
	}

}
