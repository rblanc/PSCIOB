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
#include "shape3DCuboid.h"
#include "Translation3DTransform.h"
#include "shape2DRectangle.h"
#include "Rigid2DTransform.h"
#include "Similarity2DTransform.h"
#include "TranslationScale3DTransform.h"
#include "Direct3DSphere.h"
#include "VTKScene.h"

#include "itkLabelMapToBinaryImageFilter.h"
#include "itkLabelMapToLabelImageFilter.h"

#include "SimpleUniformSensor.h"
#include "SimpleSEMSensor.h"
#include "SimplePixelBasedSEMSensor.h"
#include "PixelSetObjectOverlap.h"

using namespace psciob;

void TestSimple2D2DSensor() {
	std::cout<<"\n\n TestSimple2D2DSensor: draw a few rectangles, use a SimpleUniformSensor to generate an image, add an object, test updating only the necessary image part..."<<std::endl;

	shape2DRectangle::Pointer rectangleShape = shape2DRectangle::New();
	Similarity2DTransform::Pointer transform = Similarity2DTransform::New();
	PoseTransformedBinaryShape<2>::Pointer rectangle = PoseTransformedBinaryShape<2>::New();

	vnl_vector<double> rectangleRadius(1);	rectangleRadius(0) = 2;	rectangleShape->SetParameters(rectangleRadius);
	vnl_vector<double> pose2D(4); pose2D(0) = 2.1; pose2D(1) = 1.7; pose2D(2) = PI/3.0; pose2D(3) = 5; transform->SetParameters(pose2D);
	rectangle->SetShapeAndTransform(rectangleShape, transform);

	typedef PoseTransformedBinaryShape<2>::BaseClass Base2DObjectType;	

	typedef VTKScene<2> SceneType;
	//typedef itk::Image<unsigned char, 2> Image2DType;	typedef itk::Image<unsigned char, 3> Image3DType;

	SceneType::Pointer scene = SceneType::New();
	//define the characteristics of the scene
	vnl_vector<double> sceneSpacing(2);	sceneSpacing.fill(0.5);
	vnl_vector<double> sceneBBox(4); for (unsigned i=0 ; i<2 ; i++) { sceneBBox(2*i) = -10;	sceneBBox(2*i+1) = 10; }
	scene->SetPhysicalDimensions(sceneBBox, sceneSpacing);

	//add a few objects to the scene
	vnl_vector<double> rectangleParams(5);
	rectangleParams(0) = 0;	rectangleParams(1) = 0;	rectangleParams(2) = PI/3.0; rectangleParams(3) = 3 ; rectangleParams(4) = 1; rectangle->SetParameters(rectangleParams); scene->AddObject(rectangle);
	rectangleParams(0) = 4;	rectangleParams(1) = 2;	rectangleParams(2) =-PI/5.0; rectangleParams(3) = 5; rectangleParams(4) = 2; rectangle->SetParameters(rectangleParams); scene->AddObject(rectangle);
	rectangleParams(0) =-5; rectangleParams(1) = 4; rectangleParams(2) = PI/8.0; rectangleParams(3) = 4 ; rectangleParams(4) =1.5;rectangle->SetParameters(rectangleParams); scene->AddObject(rectangle);

	//observing the scene through a simple sensor
	typedef SimpleUniformSensor<SceneType> SensorType;
	SensorType::Pointer sensor = SensorType::New();
	sensor->SetScene(scene);
	vnl_vector<double> appearanceParams(1); appearanceParams(0)=200; sensor->SetAppearanceParameters(appearanceParams);
	//by default, with a HORIZONTAL view, and a exponentially decreasing appearance function, with max = 250 (decreases with the distance to the sensor)

	Write2DGreyLevelRescaledImageToFile<SensorType::OutputImageType>("Simple2D2DSensedScene_1.png", sensor->GetOutput());

	std::cout<<"add an object and update the sensor"<<std::endl;
	rectangleParams(0) =-6; rectangleParams(1) =-5; rectangleParams(2) =-PI/4.0; rectangleParams(3) = 3 ; rectangleParams(4) =4; rectangle->SetParameters(rectangleParams); scene->AddObject(rectangle);

	Write2DGreyLevelRescaledImageToFile<SensorType::OutputImageType>("Simple2D2DSensedScene_2.png", sensor->GetOutput());

	std::cout<<"\n\n TestSimple2D2DSensor: completed successfully "<<std::endl;
}

void TestSimple3D3DSensor() {
	std::cout<<"\n\n TestSimple3D3DSensor: draw a few spheres, use a SimpleUniformSensor to capture an image, save it, test GetOffContextPixelSet"<<std::endl;

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

	//add a few objects to the scene
	vnl_vector<double> sphereParams(4);
	sphereParams(0) = 0;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = 1;	sphereParams(1) = 1;	sphereParams(2) = 0;	sphereParams(3) = 1.5;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = -4.2;	sphereParams(1) = 2.3;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);


	//observing the scene through a 3D/3D sensor
	typedef SimpleUniformSensor<Scene3DType> SensorType;
	SensorType::Pointer sensor3D = SensorType::New();
	sensor3D->SetScene(scene3D);
	vnl_vector<double> appearanceParams(1); appearanceParams(0)=200; sensor3D->SetAppearanceParameters(appearanceParams);
	//by default, with a HORIZONTAL view, and a exponentially decreasing appearance function, with max = 250 (decreases with the distance to the sensor)

	WriteITKImageToFile<SensorType::OutputImageType>("Simple3D3DSensedScene.nii", sensor3D->GetOutput());

	std::cout<<"\n\n TestSimple3D3DSensor: completed successfully "<<std::endl;
}



void TestSimpleSEMSensor() {
	std::cout<<"\n\n TestSimpleSEMSensor: draw a few spheres, use a SimpleSEMSensor to capture an image, save it, test GetOffContextPixelSet"<<std::endl;

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

	//add a few objects to the scene
	vnl_vector<double> sphereParams(4);
	sphereParams(0) = 0;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = 1;	sphereParams(1) = 1;	sphereParams(2) = 0;	sphereParams(3) = 1.5;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = -4.2;	sphereParams(1) = 2.3;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = 3.2;	sphereParams(1) =-2.3;	sphereParams(2) = 0;	sphereParams(3) = 0.8;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) =-1.2;	sphereParams(1) =-4.3;	sphereParams(2) = 0;	sphereParams(3) = 4;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);

	WriteMirrorPolyDataToFile("SEMSensor_VTKScene.vtk", scene3D->GetSceneAsVTKPolyData());

	//observing the scene through a 3D/2D sensor
	typedef itk::Image<float, 2> Image2DType;
	typedef itk::Image<float, 3> Image3DType;
	Image3DType::Pointer img3D = Image3DType::New();
	typedef itk::Image<unsigned short, 2> UShort2DImageType;
	typedef itk::Image<unsigned short, 3> UShort3DImageType;
	UShort3DImageType::Pointer img3DUS = UShort3DImageType::New();
	typedef SimpleSEMSensor<Scene3DType, Image2DType> SensorType;
	typedef itk::LabelMapToLabelImageFilter<SensorType::OutputLabelMapType, UShort2DImageType> LabelMapToLabelImageFilterType;
	LabelMapToLabelImageFilterType::Pointer filter = LabelMapToLabelImageFilterType::New();
	
	SensorType::Pointer sensor3D = SensorType::New();
	sensor3D->SetScene(scene3D);
	sensor3D->SetObservationDirectionType(HORIZONTAL);
	vnl_vector<double> appearanceParams(2); appearanceParams(0)=250; appearanceParams(0)=1.0/40.0;
	sensor3D->SetAppearanceParameters(appearanceParams);
	//by default, with a HORIZONTAL view, and a exponentially decreasing appearance function, with max = 250 (decreases with the distance to the sensor)

	//Write2DGreyLevelRescaledImageToFile<Image2DType>("SimpleSEMSensedScene.png", sensor3D->GetOutput());
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(4));
	WriteITKImageToFile<Image3DType>("SEMSensor_Hor.nii", img3D);
	filter->SetInput( sensor3D->GetLabelOutput() ); filter->Update();
	Convert2DITKImageToFlat3D<unsigned short, unsigned short>(filter->GetOutput(), img3DUS, sensor3D->GetObservationDirectionType(), sceneBBox(4));
	std::cout<<"number of objects in horizontal label map: "<<sensor3D->GetLabelOutput()->GetNumberOfLabelObjects()<<std::endl;
	WriteITKImageToFile<UShort3DImageType>("SEMSensor_HorLabels.nii", img3DUS);

	sensor3D->SetObservationDirectionType(SAGITTAL);
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	WriteITKImageToFile<Image3DType>("SEMSensor_Sag.nii", img3D);
	filter->SetInput( sensor3D->GetLabelOutput() ); filter->Update();
	Convert2DITKImageToFlat3D<unsigned short, unsigned short>(filter->GetOutput(), img3DUS, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	std::cout<<"number of objects in sagittal label map: "<<sensor3D->GetLabelOutput()->GetNumberOfLabelObjects()<<std::endl;
	WriteITKImageToFile<UShort3DImageType>("SEMSensor_SagLabels.nii", img3DUS);

	sensor3D->SetObservationDirectionType(CORONAL);
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(2));
	WriteITKImageToFile<Image3DType>("SEMSensor_Cor.nii", img3D);
	filter->SetInput( sensor3D->GetLabelOutput() ); filter->Update();
	Convert2DITKImageToFlat3D<unsigned short, unsigned short>(filter->GetOutput(), img3DUS, sensor3D->GetObservationDirectionType(), sceneBBox(2));
	std::cout<<"number of objects in coronal label map: "<<sensor3D->GetLabelOutput()->GetNumberOfLabelObjects()<<std::endl;
	WriteITKImageToFile<UShort3DImageType>("SEMSensor_CorLabels.nii", img3DUS);

	//
	std::cout<<"\nGetting offContext image of object 2"<<std::endl;
	SensorType::OutputImageType::Pointer singleObjectImage = SensorType::OutputImageType::New();
	SensorType::OutputLabelMapType::Pointer singleObjectLabelMap = SensorType::OutputLabelMapType::New();
	sensor3D->GetOffContextObjectImage( scene3D->GetObject(2), singleObjectImage, singleObjectLabelMap );
	std::cout<<"number of object in singleObjectLabelMap: "<<singleObjectLabelMap->GetNumberOfLabelObjects()<<std::endl;
	Convert2DITKImageToFlat3D<float, float>(singleObjectImage, img3D, sensor3D->GetObservationDirectionType(), sceneBBox(2));
	WriteITKImageToFile<Image3DType>("SEMSensor_Cor_Single.nii", img3D);


	//
	std::cout<<"Removing first object and updating the sensor"<<std::endl;
	scene3D->RemoveObject(1);

	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(2));
	WriteITKImageToFile<Image3DType>("SEMSensor_Cor2.nii", img3D);
	filter->SetInput( sensor3D->GetLabelOutput() ); filter->Update();
	Convert2DITKImageToFlat3D<unsigned short, unsigned short>(filter->GetOutput(), img3DUS, sensor3D->GetObservationDirectionType(), sceneBBox(2));
	std::cout<<"number of objects in coronal label map: "<<sensor3D->GetLabelOutput()->GetNumberOfLabelObjects()<<std::endl;
	WriteITKImageToFile<UShort3DImageType>("SEMSensor_CorLabels2.nii", img3DUS);

	SensorType::OutputImageType::Pointer outImg = SensorType::OutputImageType::New();
	SensorType::OutputLabelMapType::Pointer outLblImg = SensorType::OutputLabelMapType::New();

	////
	//std::cout<<"loop modifying back and forth the object... just to check for memory leak..."<<std::endl;
	//clock_t t0 = clock();
	//for (unsigned i=0 ; i<1001 ; i++) {
	//	sphereParams(0) += 0.001; sphereParams(3) += 0.001;
	//	if (!scene3D->ModifyObjectParameters(2, sphereParams)) std::cout<<"error modifying params"<<std::endl;

	//	if (!sensor3D->GetOffContextObjectImage( scene3D->GetObject(2), outImg, outLblImg )) std::cout<<"pb with getting the offcontext image..."<<std::endl;
	//	sensor3D->GetOutput();

	//	if (i==100) std::cout<<"100 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
	//	if (i==500) std::cout<<"500 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
	//	if (i==1000) std::cout<<"1000 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
	//}

	std::cout<<"\n\n TestSimpleSEMSensor: completed successfully "<<std::endl;
}

//
#include "LabelMapScene.h"
#include "SimplePixelBasedSEMSensor.h"
//
void TestPixelSetBasedSEMSensor() {
	std::cout<<"\n\n TestPixelSetBasedSEMSensor: draw a few spheres, use a SimplePixelBasedSEMSensor to capture an image, save it, test GetOffContextPixelSet"<<std::endl;

	Direct3DSphere::Pointer sphere = Direct3DSphere::New();

	typedef LabelMapScene<3> Scene3DType;

	Scene3DType::Pointer scene3D = Scene3DType::New();
	//define the characteristics of the scene
	vnl_vector<double> sceneSpacing(3);	sceneSpacing.fill(0.5);
	vnl_vector<double> sceneBBox(6);	for (unsigned i=0 ; i<3 ; i++) { sceneBBox(2*i) = -50;	sceneBBox(2*i+1) = 50; }
	scene3D->SetPhysicalDimensions(sceneBBox, sceneSpacing);
	std::cout<<"number of pixels in the scene: "<<scene3D->GetSceneImageRegion().GetSize(0)<<", "<<scene3D->GetSceneImageRegion().GetSize(1)<<", "<<scene3D->GetSceneImageRegion().GetSize(2)<<std::endl;
	//add a few objects to the scene
	vnl_vector<double> sphereParams(4);
	sphereParams(0) = 0;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = 1;	sphereParams(1) = 1;	sphereParams(2) = 0;	sphereParams(3) = 1.5;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = -4.2;	sphereParams(1) = 2.3;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = 3.2;	sphereParams(1) =-2.3;	sphereParams(2) = 0;	sphereParams(3) = 0.8;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) =-1.2;	sphereParams(1) =-4.3;	sphereParams(2) = 0;	sphereParams(3) = 4.21;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = 2;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = 3;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);

	WriteMirrorPolyDataToFile("PixelSEMSensor_VTKScene.vtk", scene3D->GetSceneAsVTKPolyData());

	//observing the scene through a 3D/2D sensor
	typedef itk::Image<float, 2> Image2DType;
	typedef itk::Image<float, 3> Image3DType;
	Image3DType::Pointer img3D = Image3DType::New();
	typedef itk::Image<unsigned short, 2> UShort2DImageType;
	typedef itk::Image<unsigned short, 3> UShort3DImageType;
	UShort3DImageType::Pointer img3DUS = UShort3DImageType::New();
	typedef SimplePixelBasedSEMSensor<Scene3DType, Image2DType> SensorType;
	typedef itk::LabelMapToLabelImageFilter<SensorType::OutputLabelMapType, UShort2DImageType> LabelMapToLabelImageFilterType;
	LabelMapToLabelImageFilterType::Pointer filter = LabelMapToLabelImageFilterType::New();
	
	SensorType::Pointer sensor3D = SensorType::New();
	sensor3D->SetScene(scene3D);
	sensor3D->SetObservationDirectionType(HORIZONTAL);
	vnl_vector<double> appearanceParams(2); 
	appearanceParams(0)=250;appearanceParams(1)=1.0/100.0;
	sensor3D->SetAppearanceParameters(appearanceParams);
	
	//by default, with a HORIZONTAL view, and a exponentially decreasing appearance function, with max = 250 (decreases with the distance to the sensor)
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(4));
	WriteITKImageToFile<Image3DType>("PixelSEMSensor_Hor.nii", img3D);
	filter->SetInput( sensor3D->GetLabelOutput() ); filter->Update();
	Convert2DITKImageToFlat3D<unsigned short, unsigned short>(filter->GetOutput(), img3DUS, sensor3D->GetObservationDirectionType(), sceneBBox(4));
	std::cout<<"number of objects in horizontal label map: "<<sensor3D->GetLabelOutput()->GetNumberOfLabelObjects()<<std::endl;
	WriteITKImageToFile<UShort3DImageType>("PixelSEMSensor_HorLabels.nii", img3DUS);

	SensorType::OutputImageType::Pointer singleObjectImage = SensorType::OutputImageType::New();
	SensorType::OutputLabelMapType::Pointer singleObjectLabelMap = SensorType::OutputLabelMapType::New();
	sensor3D->GetOffContextObjectImage( scene3D->GetObject(5), singleObjectImage, singleObjectLabelMap );
	std::cout<<"number of object in singleObjectLabelMap: "<<singleObjectLabelMap->GetNumberOfLabelObjects()<<std::endl;
	Convert2DITKImageToFlat3D<float, float>(singleObjectImage, img3D, sensor3D->GetObservationDirectionType(), sceneBBox(4));
	WriteITKImageToFile<Image3DType>("PixelSEMSensor_Hor_Single.nii", img3D);



	sensor3D->SetObservationDirectionType(CORONAL);
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(2));
	WriteITKImageToFile<Image3DType>("PixelSEMSensor_Cor.nii", img3D);
	filter->SetInput( sensor3D->GetLabelOutput() ); filter->Update();
	Convert2DITKImageToFlat3D<unsigned short, unsigned short>(filter->GetOutput(), img3DUS, sensor3D->GetObservationDirectionType(), sceneBBox(2));
	std::cout<<"number of objects in coronal label map: "<<sensor3D->GetLabelOutput()->GetNumberOfLabelObjects()<<std::endl;
	WriteITKImageToFile<UShort3DImageType>("PixelSEMSensor_CorLabels.nii", img3DUS);

	sensor3D->GetOffContextObjectImage( scene3D->GetObject(5), singleObjectImage, singleObjectLabelMap );
	Convert2DITKImageToFlat3D<float, float>(singleObjectImage, img3D, sensor3D->GetObservationDirectionType(), sceneBBox(2));
	WriteITKImageToFile<Image3DType>("PixelSEMSensor_Cor_Single.nii", img3D);



	sensor3D->SetObservationDirectionType(SAGITTAL);
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	WriteITKImageToFile<Image3DType>("PixelSEMSensor_Sag.nii", img3D);
	filter->SetInput( sensor3D->GetLabelOutput() ); filter->Update();
	Convert2DITKImageToFlat3D<unsigned short, unsigned short>(filter->GetOutput(), img3DUS, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	std::cout<<"number of objects in sagittal label map: "<<sensor3D->GetLabelOutput()->GetNumberOfLabelObjects()<<std::endl;
	WriteITKImageToFile<UShort3DImageType>("PixelSEMSensor_SagLabels.nii", img3DUS);

	sensor3D->GetOffContextObjectImage( scene3D->GetObject(5), singleObjectImage, singleObjectLabelMap );
	Convert2DITKImageToFlat3D<float, float>(singleObjectImage, img3D, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	WriteITKImageToFile<Image3DType>("PixelSEMSensor_Sag_Single.nii", img3D);



	//
	std::cout<<"Modifying object 5 and updating the sensor"<<std::endl;
	sphereParams = scene3D->GetParametersOfObject(5);
	sphereParams(1)+=0.3;
	scene3D->ModifyObjectParameters(5, sphereParams);
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	WriteITKImageToFile<Image3DType>("PixelSEMSensor_Sag1.nii", img3D);

	sphereParams(1)+=0.3;
	scene3D->ModifyObjectParameters(5, sphereParams);
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	WriteITKImageToFile<Image3DType>("PixelSEMSensor_Sag2.nii", img3D);

	sphereParams(1)+=0.3;
	scene3D->ModifyObjectParameters(5, sphereParams);
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	WriteITKImageToFile<Image3DType>("PixelSEMSensor_Sag3.nii", img3D);
	
	sphereParams(1)+=0.3;
	scene3D->ModifyObjectParameters(5, sphereParams);
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	WriteITKImageToFile<Image3DType>("PixelSEMSensor_Sag4.nii", img3D);
	
	sphereParams(1)+=0.3;
	scene3D->ModifyObjectParameters(5, sphereParams);
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	WriteITKImageToFile<Image3DType>("PixelSEMSensor_Sag5.nii", img3D);

	sphereParams(1)-=0.3;
	scene3D->ModifyObjectParameters(5, sphereParams);
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	WriteITKImageToFile<Image3DType>("PixelSEMSensor_Sag6.nii", img3D);

	sphereParams(1)-=0.3;
	scene3D->ModifyObjectParameters(5, sphereParams);
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	WriteITKImageToFile<Image3DType>("PixelSEMSensor_Sag7.nii", img3D);

	//SensorType::OutputImageType::Pointer outImg = SensorType::OutputImageType::New();
	//SensorType::OutputLabelMapType::Pointer outLblImg = SensorType::OutputLabelMapType::New();

	////
	//std::cout<<"loop modifying back and forth the object . CORONAL direction... just to check for memory leak..."<<std::endl;
	//clock_t t0 = clock();
	//for (unsigned i=0 ; i<1001 ; i++) {
	//	//sphereParams(0) += 0.001; sphereParams(3) += 0.001;
	//	//if (!scene3D->ModifyObjectParameters(2, sphereParams)) std::cout<<"error modifying params"<<std::endl;
	//	//if (!sensor3D->GetOffContextObjectImage( scene3D->GetObject(2), outImg, outLblImg )) std::cout<<"pb with getting the offcontext image..."<<std::endl;
	//	//sensor3D->GetOutput();

	//	sphereParams(0) = 0; sphereParams(1) = 0; sphereParams(2) = 0; sphereParams(3) = 2; sphere->SetParameters(sphereParams); 
	//	Scene3DType::IDType id = scene3D->AddObject(sphere);
	//	if (!sensor3D->GetOffContextObjectImage( scene3D->GetObject(2), outImg, outLblImg )) std::cout<<"pb with getting the offcontext image..."<<std::endl;
	//	sensor3D->GetOutput();
	//	scene3D->RemoveObject(id);
	//	if (!sensor3D->GetOffContextObjectImage( scene3D->GetObject(2), outImg, outLblImg )) std::cout<<"pb with getting the offcontext image..."<<std::endl;
	//	sensor3D->GetOutput();


	//	if (i==100) std::cout<<"100 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
	//	if (i==500) std::cout<<"500 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
	//	if (i==1000) std::cout<<"1000 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
	//}


	////
	//std::cout<<"loop modifying back and forth the object . SAGITTAL direction... just to check for memory leak..."<<std::endl;
	//sensor3D->SetObservationDirectionType(SAGITTAL);
	//t0 = clock();
	//unsigned n=1, nmax = scene3D->GetNumberOfObjects();
	//for (unsigned i=0 ; i<1001 ; i++) {
	//	n++;
	//	sphereParams = scene3D->GetParametersOfObject(n);
	//	sphereParams(0) +=0.001; sphereParams(3) += 0.001;
	//	if (!scene3D->ModifyObjectParameters(n, sphereParams)) std::cout<<"error modifying params"<<std::endl;
	//	if (!sensor3D->GetOffContextObjectImage( scene3D->GetObject(n), outImg, outLblImg )) std::cout<<"pb with getting the offcontext image..."<<std::endl;
	//	sensor3D->GetOutput();
	//	if (n==nmax) n=1;
	//	if (i==100) std::cout<<"100 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
	//	if (i==500) std::cout<<"500 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
	//	if (i==1000) std::cout<<"1000 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
	//}
	std::cout<<"\n\n TestPixelSetBasedSEMSensor: completed successfully "<<std::endl;
}

//
#include "LabelImageScene.h"
//
void TestPixelSetBasedSEMSensorWithLabelImage() {
	std::cout<<"\n\n TestPixelSetBasedSEMSensorWithLabelImage: draw a few spheres, use a SimplePixelBasedSEMSensor to capture an image, save it, test GetOffContextPixelSet"<<std::endl;

	//shape3DSphere::Pointer sphereShape = shape3DSphere::New();
	//Translation3DTransform::Pointer translation3D = Translation3DTransform::New();
	//PoseTransformedBinaryShape<3>::Pointer sphere = PoseTransformedBinaryShape<3>::New();
	//vnl_vector<double> sphereRadius(1);	sphereRadius(0) = 2.215;	sphereShape->SetParameters(sphereRadius);
	//vnl_vector<double> trans3D(3);		trans3D.fill(0);			translation3D->SetParameters(trans3D);
	//sphere->SetShapeAndTransform(sphereShape, translation3D);
	Direct3DSphere::Pointer sphere = Direct3DSphere::New();

	typedef PoseTransformedBinaryShape<3>::BaseClass Base3DObjectType;	

	typedef LabelImageScene<3> Scene3DType;
	//typedef itk::Image<unsigned char, 2> Image2DType;	typedef itk::Image<unsigned char, 3> Image3DType;

	Scene3DType::Pointer scene3D = Scene3DType::New();
	//define the characteristics of the scene
	vnl_vector<double> sceneSpacing(3);	sceneSpacing.fill(0.5);
	vnl_vector<double> sceneBBox(6);	for (unsigned i=0 ; i<3 ; i++) { sceneBBox(2*i) = -50;	sceneBBox(2*i+1) = 50; }
	scene3D->SetPhysicalDimensions(sceneBBox, sceneSpacing);
	std::cout<<"number of pixels in the scene: "<<scene3D->GetSceneImageRegion().GetSize(0)<<", "<<scene3D->GetSceneImageRegion().GetSize(1)<<", "<<scene3D->GetSceneImageRegion().GetSize(2)<<std::endl;
	//add a few objects to the scene
	vnl_vector<double> sphereParams(4);
	sphereParams(0) = 0;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = 1;	sphereParams(1) = 1;	sphereParams(2) = 0;	sphereParams(3) = 1.5;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = -4.2;	sphereParams(1) = 2.3;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = 3.2;	sphereParams(1) =-2.3;	sphereParams(2) = 0;	sphereParams(3) = 0.8;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) =-1.2;	sphereParams(1) =-4.3;	sphereParams(2) = 0;	sphereParams(3) = 4;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = 2;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = 3;	sphereParams(1) = 0;	sphereParams(2) = 0;	sphereParams(3) = 2;	sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);

	WriteMirrorPolyDataToFile("PixelSEMSensorLabImg_VTKScene.vtk", scene3D->GetSceneAsVTKPolyData());

	//observing the scene through a 3D/2D sensor
	typedef itk::Image<float, 2> Image2DType;
	typedef itk::Image<float, 3> Image3DType;
	Image3DType::Pointer img3D = Image3DType::New();
	typedef itk::Image<unsigned short, 2> UShort2DImageType;
	typedef itk::Image<unsigned short, 3> UShort3DImageType;
	UShort3DImageType::Pointer img3DUS = UShort3DImageType::New();
	typedef SimplePixelBasedSEMSensor<Scene3DType, Image2DType> SensorType;
	typedef itk::LabelMapToLabelImageFilter<SensorType::OutputLabelMapType, UShort2DImageType> LabelMapToLabelImageFilterType;
	LabelMapToLabelImageFilterType::Pointer filter = LabelMapToLabelImageFilterType::New();
	
	SensorType::Pointer sensor3D = SensorType::New();
	sensor3D->SetScene(scene3D);
	sensor3D->SetObservationDirectionType(HORIZONTAL);
	vnl_vector<double> appearanceParams(1); appearanceParams(0)=200; sensor3D->SetAppearanceParameters(appearanceParams);
	
	//by default, with a HORIZONTAL view, and a exponentially decreasing appearance function, with max = 250 (decreases with the distance to the sensor)
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(4));
	WriteITKImageToFile<Image3DType>("PixelSEMSensorLabImg_Hor.nii", img3D);
	filter->SetInput( sensor3D->GetLabelOutput() ); filter->Update();
	Convert2DITKImageToFlat3D<unsigned short, unsigned short>(filter->GetOutput(), img3DUS, sensor3D->GetObservationDirectionType(), sceneBBox(4));
	std::cout<<"number of objects in horizontal label map: "<<sensor3D->GetLabelOutput()->GetNumberOfLabelObjects()<<std::endl;
	WriteITKImageToFile<UShort3DImageType>("PixelSEMSensorLabImg_HorLabels.nii", img3DUS);

	SensorType::OutputImageType::Pointer singleObjectImage = SensorType::OutputImageType::New();
	SensorType::OutputLabelMapType::Pointer singleObjectLabelMap = SensorType::OutputLabelMapType::New();
	sensor3D->GetOffContextObjectImage( scene3D->GetObject(5), singleObjectImage, singleObjectLabelMap );
	std::cout<<"number of object in singleObjectLabelMap: "<<singleObjectLabelMap->GetNumberOfLabelObjects()<<std::endl;
	Convert2DITKImageToFlat3D<float, float>(singleObjectImage, img3D, sensor3D->GetObservationDirectionType(), sceneBBox(4));
	WriteITKImageToFile<Image3DType>("PixelSEMSensorLabImg_Hor_Single.nii", img3D);


	sensor3D->SetObservationDirectionType(SAGITTAL);
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	WriteITKImageToFile<Image3DType>("PixelSEMSensorLabImg_Sag.nii", img3D);
	filter->SetInput( sensor3D->GetLabelOutput() ); filter->Update();
	Convert2DITKImageToFlat3D<unsigned short, unsigned short>(filter->GetOutput(), img3DUS, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	std::cout<<"number of objects in sagittal label map: "<<sensor3D->GetLabelOutput()->GetNumberOfLabelObjects()<<std::endl;
	WriteITKImageToFile<UShort3DImageType>("PixelSEMSensorLabImg_SagLabels.nii", img3DUS);

	sensor3D->GetOffContextObjectImage( scene3D->GetObject(5), singleObjectImage, singleObjectLabelMap );
	Convert2DITKImageToFlat3D<float, float>(singleObjectImage, img3D, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	WriteITKImageToFile<Image3DType>("PixelSEMSensorLabImg_Sag_Single.nii", img3D);

	sensor3D->SetObservationDirectionType(CORONAL);
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(2));
	WriteITKImageToFile<Image3DType>("PixelSEMSensorLabImg_Cor.nii", img3D);
	filter->SetInput( sensor3D->GetLabelOutput() ); filter->Update();
	Convert2DITKImageToFlat3D<unsigned short, unsigned short>(filter->GetOutput(), img3DUS, sensor3D->GetObservationDirectionType(), sceneBBox(2));
	std::cout<<"number of objects in coronal label map: "<<sensor3D->GetLabelOutput()->GetNumberOfLabelObjects()<<std::endl;
	WriteITKImageToFile<UShort3DImageType>("PixelSEMSensorLabImg_CorLabels.nii", img3DUS);

	sensor3D->GetOffContextObjectImage( scene3D->GetObject(5), singleObjectImage, singleObjectLabelMap );
	Convert2DITKImageToFlat3D<float, float>(singleObjectImage, img3D, sensor3D->GetObservationDirectionType(), sceneBBox(2));
	WriteITKImageToFile<Image3DType>("PixelSEMSensorLabImg_Cor_Single.nii", img3D);


	//
	std::cout<<"Removing first object and updating the sensor"<<std::endl;
	scene3D->RemoveObject(1);

	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(2));
	WriteITKImageToFile<Image3DType>("PixelSEMSensorLabImg_Cor2.nii", img3D);
	filter->SetInput( sensor3D->GetLabelOutput() ); filter->Update();
	Convert2DITKImageToFlat3D<unsigned short, unsigned short>(filter->GetOutput(), img3DUS, sensor3D->GetObservationDirectionType(), sceneBBox(2));
	std::cout<<"number of objects in coronal label map: "<<sensor3D->GetLabelOutput()->GetNumberOfLabelObjects()<<std::endl;
	WriteITKImageToFile<UShort3DImageType>("PixelSEMSensorLabImg_CorLabels2.nii", img3DUS);

	SensorType::OutputImageType::Pointer outImg = SensorType::OutputImageType::New();
	SensorType::OutputLabelMapType::Pointer outLblImg = SensorType::OutputLabelMapType::New();

	//
	std::cout<<"loop modifying back and forth the object . CORONAL direction... just to check for memory leak..."<<std::endl;
	clock_t t0 = clock();
	for (unsigned i=0 ; i<1001 ; i++) {
		//sphereParams(0) += 0.001; sphereParams(3) += 0.001;
		//if (!scene3D->ModifyObjectParameters(2, sphereParams)) std::cout<<"error modifying params"<<std::endl;
		//if (!sensor3D->GetOffContextObjectImage( scene3D->GetObject(2), outImg, outLblImg )) std::cout<<"pb with getting the offcontext image..."<<std::endl;
		//sensor3D->GetOutput();
		sphereParams(0) = 0; sphereParams(1) = 0; sphereParams(2) = 0; sphereParams(3) = 2; sphere->SetParameters(sphereParams); 
		Scene3DType::IDType id = scene3D->AddObject(sphere);
		if (!sensor3D->GetOffContextObjectImage( scene3D->GetObject(2), outImg, outLblImg )) std::cout<<"pb with getting the offcontext image..."<<std::endl;
		sensor3D->GetOutput();
		scene3D->RemoveObject(id);
		if (!sensor3D->GetOffContextObjectImage( scene3D->GetObject(2), outImg, outLblImg )) std::cout<<"pb with getting the offcontext image..."<<std::endl;
		sensor3D->GetOutput();


		if (i==100) std::cout<<"100 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
		if (i==500) std::cout<<"500 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
		if (i==1000) std::cout<<"1000 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
	}


	////
	//std::cout<<"loop modifying back and forth the object . SAGITTAL direction... just to check for memory leak..."<<std::endl;
	//sensor3D->SetObservationDirectionType(SAGITTAL);
	//t0 = clock();
	//unsigned n=1, nmax = scene3D->GetNumberOfObjects();
	//for (unsigned i=0 ; i<1001 ; i++) {
	//	n++;
	//	sphereParams = scene3D->GetParametersOfObject(n);
	//	sphereParams(0) +=0.001; sphereParams(3) += 0.001;
	//	if (!scene3D->ModifyObjectParameters(n, sphereParams)) std::cout<<"error modifying params"<<std::endl;
	//	if (!sensor3D->GetOffContextObjectImage( scene3D->GetObject(n), outImg, outLblImg )) std::cout<<"pb with getting the offcontext image..."<<std::endl;
	//	sensor3D->GetOutput();
	//	if (n==nmax) n=1;
	//	if (i==100) std::cout<<"100 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
	//	if (i==500) std::cout<<"500 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
	//	if (i==1000) std::cout<<"1000 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;
	//}
	std::cout<<"\n\n TestPixelSetBasedSEMSensorWithLabelImage: completed successfully "<<std::endl;
}


//
void TestOverlaps() {
	std::cout<<"\n\n TestOverlaps: testing the LabelImageScene and PixelSetBasedSEMSensor with respect to overlaps..."<<std::endl;

	//shape3DSphere::Pointer sphereShape = shape3DSphere::New();
	//Translation3DTransform::Pointer translation3D = Translation3DTransform::New();
	//PoseTransformedBinaryShape<3>::Pointer sphere = PoseTransformedBinaryShape<3>::New();
	//vnl_vector<double> sphereRadius(1);	sphereRadius(0) = 2.215;	sphereShape->SetParameters(sphereRadius);
	//vnl_vector<double> trans3D(3);		trans3D.fill(0);			translation3D->SetParameters(trans3D);
	//sphere->SetShapeAndTransform(sphereShape, translation3D);
	Direct3DSphere::Pointer sphere = Direct3DSphere::New();

	typedef ParametricObject<3, unsigned char>::BaseClass Base3DObjectType;	

//both version are apparently working well.!
	typedef LabelImageScene<3> Scene3DType;
//	typedef PixelSetScene<Base3DObjectType, unsigned short, PriorCostAndPixelSetContainer, PixelSetIntersectionContainer> Scene3DType;
//	typedef PixelSetObjectOverlap<Scene3DType> InteractionManagerType;
//	InteractionManagerType::Pointer interactionManager = InteractionManagerType::New();

	Scene3DType::Pointer scene3D = Scene3DType::New();
//	scene3D->SetInteractionManager( (Scene3DType::ObjectInteractionManagerType *)interactionManager.GetPointer() );


	//PairWiseOverlapCoefficient::Pointer pairwiseIntersectionCostFunction = PairWiseOverlapCoefficient::New();
	TransformedPairWiseOverlapMeasure< GPF_ScaledExpFunction<double, double>, PairWiseOverlapCoefficient >::Pointer pairwiseIntersectionCostFunction = TransformedPairWiseOverlapMeasure< GPF_ScaledExpFunction<double, double>, PairWiseOverlapCoefficient >::New();
	vnl_vector<double> fctParams(1); fctParams(0) = 10; pairwiseIntersectionCostFunction->SetParameters(fctParams);
	scene3D->GetInteractionManager()->SetIntersectionCostFunction( pairwiseIntersectionCostFunction );

	//define the characteristics of the scene
	vnl_vector<double> sceneSpacing(3);	sceneSpacing.fill(0.5);
	vnl_vector<double> sceneBBox(6);	for (unsigned i=0 ; i<3 ; i++) { sceneBBox(2*i) = -50;	sceneBBox(2*i+1) = 50; }
	scene3D->SetPhysicalDimensions(sceneBBox, sceneSpacing);
	SceneObjectIterator<Scene3DType> it(scene3D); //define an iterator for the scene objects
	std::cout<<"number of pixels in the scene: "<<scene3D->GetSceneImageRegion().GetSize(0)<<", "<<scene3D->GetSceneImageRegion().GetSize(1)<<", "<<scene3D->GetSceneImageRegion().GetSize(2)<<std::endl;
	
	//define the sensor
	typedef itk::Image<float, 2> Image2DType;
	typedef itk::Image<float, 3> Image3DType; Image3DType::Pointer img3D = Image3DType::New();
	typedef SimplePixelBasedSEMSensor<Scene3DType, Image2DType> SensorType;
	SensorType::Pointer sensor3D = SensorType::New();
	sensor3D->SetScene(scene3D);
	sensor3D->SetObservationDirectionType(SAGITTAL); //this is the fastest, with respect to the representation of objects as labelObjects
	vnl_vector<double> appearanceParams(1); appearanceParams(0)=200; sensor3D->SetAppearanceParameters(appearanceParams);
	
	//add a few objects to the scene
	vnl_vector<double> sphereParams(4);
	
	//
	std::cout<<"\n0: Adding two overlapping spheres"<<std::endl;
	sphereParams(0) = 0; sphereParams(1) = 0; sphereParams(2) = 0; sphereParams(3) = 5; sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	sphereParams(0) = 0; sphereParams(1) = 0; sphereParams(2)=7.5; sphereParams(3) = 5; sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);

	WriteITKImageToFile<Scene3DType::LabelImageType>("TestOverlaps_0_lab.nii", scene3D->GetSceneAsLabelImage());
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	WriteITKImageToFile<Image3DType>("TestOverlaps_0_sensed.nii", img3D);
	std::cout<<"browsing all objects in the scene, and checking for interactions"<<std::endl;
	for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
		for (Scene3DType::ObjectInteractionIterator iit = it.GetObjectInScene()->interactionData.begin() ; iit!=it.GetObjectInScene()->interactionData.end() ; iit++) {
			if (it.GetID()>iit->first) std::cout<<"  interaction between object: "<<it.GetID()<<" and "<<iit->first<<", interaction cost: "<<iit->second.interactionCost<<", nb of overlapping pixels: "<<iit->second.nbOverlappingPixels<<std::endl;
		}
	}

	//
	std::cout<<"\n1: Adding a third overlapping sphere, which 'hides' the overlap between spheres 1 and 2"<<std::endl;
	sphereParams(0) = 0; sphereParams(1) = 0; sphereParams(2) = 5; sphereParams(3) = 5; sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	WriteITKImageToFile<Scene3DType::LabelImageType>("TestOverlaps_1_lab.nii", scene3D->GetSceneAsLabelImage());
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	WriteITKImageToFile<Image3DType>("TestOverlaps_1_sensed.nii", img3D);
	std::cout<<"browsing all objects in the scene, and checking for interactions"<<std::endl;
	for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
		for (Scene3DType::ObjectInteractionIterator iit = it.GetObjectInScene()->interactionData.begin() ; iit!=it.GetObjectInScene()->interactionData.end() ; iit++) {
			if (it.GetID()>iit->first) std::cout<<"  interaction between object: "<<it.GetID()<<" and "<<iit->first<<", interaction cost: "<<iit->second.interactionCost<<", nb of overlapping pixels: "<<iit->second.nbOverlappingPixels<<std::endl;
		}
	}
	//
	std::cout<<"\n2: Removing sphere 2"<<std::endl;
	scene3D->RemoveObject(2);
	WriteITKImageToFile<Scene3DType::LabelImageType>("TestOverlaps_2_lab.nii", scene3D->GetSceneAsLabelImage());
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	WriteITKImageToFile<Image3DType>("TestOverlaps_2_sensed.nii", img3D);
	std::cout<<"browsing all objects in the scene, and checking for interactions"<<std::endl;
	for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
		for (Scene3DType::ObjectInteractionIterator iit = it.GetObjectInScene()->interactionData.begin() ; iit!=it.GetObjectInScene()->interactionData.end() ; iit++) {
			if (it.GetID()>iit->first) std::cout<<"  interaction between object: "<<it.GetID()<<" and "<<iit->first<<", interaction cost: "<<iit->second.interactionCost<<", nb of overlapping pixels: "<<iit->second.nbOverlappingPixels<<std::endl;
		}
	}
	//
	std::cout<<"\n3: Re-Adding the removed sphere, the overlap between this sphere and sphere 1 is shadowed by sphere 3"<<std::endl;
	sphereParams(0) = 0; sphereParams(1) = 0; sphereParams(2)=7.5; sphereParams(3) = 5; sphere->SetParameters(sphereParams);	scene3D->AddObject(sphere);
	WriteITKImageToFile<Scene3DType::LabelImageType>("TestOverlaps_3_lab.nii", scene3D->GetSceneAsLabelImage());
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	WriteITKImageToFile<Image3DType>("TestOverlaps_3_sensed.nii", img3D);
	std::cout<<"browsing all objects in the scene, and checking for interactions"<<std::endl;
	for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
		for (Scene3DType::ObjectInteractionIterator iit = it.GetObjectInScene()->interactionData.begin() ; iit!=it.GetObjectInScene()->interactionData.end() ; iit++) {
			if (it.GetID()>iit->first) std::cout<<"  interaction between object: "<<it.GetID()<<" and "<<iit->first<<", interaction cost: "<<iit->second.interactionCost<<", nb of overlapping pixels: "<<iit->second.nbOverlappingPixels<<std::endl;
		}
	}

	//Modify slighlty one sphere
	std::cout<<"\n4: Modifying sphere2, such that its overlap with sphere 1 is still shadowed by sphere 3"<<std::endl;
	sphereParams(0) = 0; sphereParams(1) = 0; sphereParams(2)=8; sphereParams(3) = 5; scene3D->ModifyObjectParameters(2, sphereParams);
	WriteITKImageToFile<Scene3DType::LabelImageType>("TestOverlaps_4_lab.nii", scene3D->GetSceneAsLabelImage());
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	WriteITKImageToFile<Image3DType>("TestOverlaps_4_sensed.nii", img3D);
	std::cout<<"browsing all objects in the scene, and checking for interactions"<<std::endl;
	for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
		for (Scene3DType::ObjectInteractionIterator iit = it.GetObjectInScene()->interactionData.begin() ; iit!=it.GetObjectInScene()->interactionData.end() ; iit++) {
			if (it.GetID()>iit->first) std::cout<<"  interaction between object: "<<it.GetID()<<" and "<<iit->first<<", interaction cost: "<<iit->second.interactionCost<<", nb of overlapping pixels: "<<iit->second.nbOverlappingPixels<<std::endl;
		}
	}
	
	//Modify one sphere
	std::cout<<"\n5: Modifying sphere2, such that it no longer overlaps with sphere 1"<<std::endl;
	sphereParams(0) = 0; sphereParams(1) = 0; sphereParams(2)=11; sphereParams(3) = 5; scene3D->ModifyObjectParameters(2, sphereParams);
	WriteITKImageToFile<Scene3DType::LabelImageType>("TestOverlaps_5_lab.nii", scene3D->GetSceneAsLabelImage());
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	WriteITKImageToFile<Image3DType>("TestOverlaps_5_sensed.nii", img3D);
	std::cout<<"browsing all objects in the scene, and checking for interactions"<<std::endl;
	for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
		for (Scene3DType::ObjectInteractionIterator iit = it.GetObjectInScene()->interactionData.begin() ; iit!=it.GetObjectInScene()->interactionData.end() ; iit++) {
			if (it.GetID()>iit->first) std::cout<<"  interaction between object: "<<it.GetID()<<" and "<<iit->first<<", interaction cost: "<<iit->second.interactionCost<<", nb of overlapping pixels: "<<iit->second.nbOverlappingPixels<<std::endl;
		}
	}	

	//Modify one sphere
	std::cout<<"\n6: Modifying sphere1, such that it touches again sphere 2, but this contact is shadowed by sphere3"<<std::endl;
	sphereParams(0) = 0; sphereParams(1) = 0; sphereParams(2)=3; sphereParams(3) = 5; scene3D->ModifyObjectParameters(1, sphereParams);
	WriteITKImageToFile<Scene3DType::LabelImageType>("TestOverlaps_6_lab.nii", scene3D->GetSceneAsLabelImage());
	Convert2DITKImageToFlat3D<float, float>(sensor3D->GetOutput(), img3D, sensor3D->GetObservationDirectionType(), sceneBBox(0));
	WriteITKImageToFile<Image3DType>("TestOverlaps_6_sensed.nii", img3D);
	std::cout<<"browsing all objects in the scene, and checking for interactions"<<std::endl;
	for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
		for (Scene3DType::ObjectInteractionIterator iit = it.GetObjectInScene()->interactionData.begin() ; iit!=it.GetObjectInScene()->interactionData.end() ; iit++) {
			if (it.GetID()>iit->first) std::cout<<"  interaction between object: "<<it.GetID()<<" and "<<iit->first<<", interaction cost: "<<iit->second.interactionCost<<", nb of overlapping pixels: "<<iit->second.nbOverlappingPixels<<std::endl;
		}
	}	

	std::cout<<"\n\n TestOverlaps: completed successfully "<<std::endl;
}

//
int main(int argc, char** argv) {

	try {
		//TestSimple2D2DSensor();
		//TestSimple3D3DSensor();
		//TestSimpleSEMSensor();

		TestPixelSetBasedSEMSensor();
		//TestPixelSetBasedSEMSensorWithLabelImage();

		//TestOverlaps();
	}
	catch (DeformableModelException& e) {
		std::cout << "Exception occured while running the test cases" << std::endl;
		std::cout << e.what() << std::endl;
	}

}
