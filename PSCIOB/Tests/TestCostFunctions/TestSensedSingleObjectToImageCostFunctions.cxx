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
#include "LabelMapScene.h"

#include "SimpleUniformSensor.h"
#include "SimpleSEMSensor.h"
#include "SimplePixelBasedSEMSensor.h"
#include "shape2DEllipse.h"
#include "Rigid2DTransform.h"

#include "SensedSingleObjectToImageMeanSquareErrorCostFunction.h"
#include "SensedSingleObjectInOutCoherence.h"
#include "SensedSingleObjectInOutContrast.h"

#include "SingleObjectEnergy.h"
#include "VTKObjectOverlap.h"
#include "LabelObjectOverlap.h"

#include "AverageObjectToImageCostFunction.h"
#include "SceneEnergy.h"

using namespace psciob;

void GenerateReferenceEllipsesImage() {

	shape2DEllipse::Pointer diskShape = shape2DEllipse::New();
	Rigid2DTransform::Pointer translation2D = Rigid2DTransform::New();
	PoseTransformedBinaryShape<2>::Pointer disk = PoseTransformedBinaryShape<2>::New();

	vnl_vector<double> diskRadius(2); diskRadius(0) = 2.1; diskRadius(1) = 1.2; diskShape->SetParameters(diskRadius);
	vnl_vector<double> trans2D(3);    trans2D.fill(0); trans2D(2) = PI/3.0;     translation2D->SetParameters(trans2D);
	disk->SetShapeAndTransform(diskShape, translation2D);

	typedef PoseTransformedBinaryShape<2>::BaseClass Base2DObjectType;

	typedef LabelMapScene<2> Scene2DType;

	Scene2DType::Pointer scene2D = Scene2DType::New();
	//define the characteristics of the scene
	vnl_vector<double> sceneSpacing(2);	sceneSpacing.fill(0.5);
	vnl_vector<double> sceneBBox(4);	for (unsigned i=0 ; i<2 ; i++) { sceneBBox(2*i) = -10;	sceneBBox(2*i+1) = 10; }
	scene2D->SetPhysicalDimensions(sceneBBox, sceneSpacing);

	//add a few objects to the scene
	vnl_vector<double> diskParams(5);
	diskParams(0) = 3;   diskParams(1) = -5;diskParams(2) = -PI/3.0; diskParams(3) = 1.5; diskParams(4) = 2;  disk->SetParameters(diskParams);	scene2D->AddObject(disk);
	diskParams(0) = 1;   diskParams(1) = 3; diskParams(2) = PI/8.0; diskParams(3) = 1.8; diskParams(4) = 1.5; disk->SetParameters(diskParams);	scene2D->AddObject(disk);
	diskParams(0) =-4.2; diskParams(1) = 0; diskParams(2) = 5.0*PI/12.0; diskParams(3) = 1.2; diskParams(4) = 1.2;  disk->SetParameters(diskParams);	scene2D->AddObject(disk);

	typedef itk::Image<unsigned char, 2> Image2DType;
	typedef SimpleUniformSensor<Scene2DType, Image2DType> SensorType;
	SensorType::Pointer sensor2D = SensorType::New();
	sensor2D->SetScene(scene2D);

	Write2DGreyLevelRescaledImageToFile<Image2DType>("referenceEllipsesImage.png", sensor2D->GetOutput());
	
}

//MSE
//
void TestSensedSingleObjectToImage_MSE() {
	std::cout<<"\n\n TestSensedSingleObjectToImage_MSE: draw a few spheres, use a SimplePixelBasedSEMSensor to capture images ; save that image ; create a second "<<std::endl;

	GenerateReferenceEllipsesImage();
	typedef itk::Image<unsigned char, 2> Image2DUCharType;
	typedef itk::Image<float, 2> Image2DFloatType;
	Image2DUCharType::Pointer referenceImage = ReadITKImageFromFile<Image2DUCharType>("referenceEllipsesImage.png", 0.5 );


	shape3DSphere::Pointer sphereShape = shape3DSphere::New();
	Translation3DTransform::Pointer translation3D = Translation3DTransform::New();
	PoseTransformedBinaryShape<3>::Pointer sphere = PoseTransformedBinaryShape<3>::New();

	vnl_vector<double> sphereRadius(1);	sphereRadius(0) = 2.215;	sphereShape->SetParameters(sphereRadius);
	vnl_vector<double> trans3D(3);		trans3D.fill(0);			translation3D->SetParameters(trans3D);
	sphere->SetShapeAndTransform(sphereShape, translation3D);

	typedef ParametricObject<3> Base3DObjectType;	
	typedef LabelMapScene<3> Scene3DType;
	typedef itk::Image<unsigned char, 3> Image3DType;
	typedef SimplePixelBasedSEMSensor<Scene3DType, Image2DFloatType> SEMSensorType;


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


	std::cout<<"added "<<scene3D->GetNumberOfObjects()<<" spheres to the scene"<<std::endl;
	std::cout<<"writing 3D representations of the scene to files: test3DScene.vtk and test3DScene_Labels.nii"<<std::endl;
	WriteMirrorPolyDataToFile("test3DScene.vtk", scene3D->GetSceneAsVTKPolyData());
	std::cout<<"writing 3D label image"<<std::endl;
	Scene3DType::LabelMapType::Pointer labelMap = scene3D->GetSceneAsLabelMap();
	typedef itk::LabelMapToLabelImageFilter< Scene3DType::LabelMapType, Scene3DType::LabelImageType> LabelMapToLabelImageFilterType;
	LabelMapToLabelImageFilterType::Pointer m_labelMapToLabelImageFilter = LabelMapToLabelImageFilterType::New();
	m_labelMapToLabelImageFilter->SetInput(labelMap);
	m_labelMapToLabelImageFilter->Update();
	WriteITKImageToFile<Scene3DType::LabelImageType>("test3DScene_Labels.nii", m_labelMapToLabelImageFilter->GetOutput());
	std::cout<<"written 3D label image"<<std::endl;

	//observing the scene through a 3D/2D sensor
	SEMSensorType::Pointer sensor3D = SEMSensorType::New();
	sensor3D->SetScene(scene3D);
	//vnl_vector<double> appearanceParams(1);appearanceParams(0)=20000; sensor3D->SetAppearanceParameters(appearanceParams);
	//by default, with a HORIZONTAL view, and a exponentially decreasing appearance function, with max = 250 (decreases with the distance to the sensor)

	std::cout<<"writing the sensed image: testSensed3DScene.png"<<std::endl;
	Write2DGreyLevelRescaledImageToFile<Image2DFloatType>("testSensed3DScene.png", sensor3D->GetOutput());
	typedef itk::LabelMapToLabelImageFilter< SEMSensorType::OutputLabelMapType, SEMSensorType::OutputLabelImageType> LabelMapTo2DLabelImageFilterType;
	LabelMapTo2DLabelImageFilterType::Pointer m_labelMapTo2DLabelImageFilter = LabelMapTo2DLabelImageFilterType::New();
	m_labelMapTo2DLabelImageFilter->SetInput( sensor3D->GetLabelOutput() );
	m_labelMapTo2DLabelImageFilter->Update();
	Write2DGreyLevelRescaledImageToFile<SEMSensorType::OutputLabelImageType>("testSensed3DScene_LabelImage.png", m_labelMapTo2DLabelImageFilter->GetOutput());

	typedef SensedSingleObjectToImageMeanSquareErrorCostFunction<Scene3DType, Image2DUCharType, Image2DFloatType> SingleObjectCostFunctionType;
	SingleObjectCostFunctionType::Pointer costFunction = SingleObjectCostFunctionType::New();
	costFunction->SetScene(scene3D);
	costFunction->SetReferenceImage(referenceImage);
	costFunction->SetSensor(sensor3D);

	std::cout<<"MEASURING the cost for object 1"<<std::endl;
	costFunction->SelectObject(1);
	costFunction->SelectOffContextComputation();
	std::cout<<"off context value = "<<costFunction->GetValue()<<std::endl;
	costFunction->SelectInContextComputation();
	std::cout<<"in context value = "<<costFunction->GetValue()<<std::endl;

	std::cout<<"measuring the cost for object 2"<<std::endl;
	costFunction->SelectObject(2);
	costFunction->SelectOffContextComputation();
	std::cout<<"off context value = "<<costFunction->GetValue()<<std::endl;
	costFunction->SelectInContextComputation();
	std::cout<<"in context value = "<<costFunction->GetValue()<<std::endl;
	costFunction->SelectOffContextComputation();
	std::cout<<"off context value = "<<costFunction->GetValue()<<std::endl;
	costFunction->SelectInContextComputation();
	std::cout<<"in context value = "<<costFunction->GetValue()<<std::endl;

	std::cout<<"measuring the cost for object 3"<<std::endl;
	costFunction->SelectObject(3);
	costFunction->SelectOffContextComputation();
	std::cout<<"off context value = "<<costFunction->GetValue()<<std::endl;
	costFunction->SelectInContextComputation();
	std::cout<<"in context value = "<<costFunction->GetValue()<<std::endl;
	costFunction->SelectOffContextComputation();
	std::cout<<"off context value = "<<costFunction->GetValue()<<std::endl;
	costFunction->SelectInContextComputation();
	std::cout<<"in context value = "<<costFunction->GetValue()<<std::endl;


	//diskParams(0) = -5;	diskParams(1) = 5;	diskParams(2) = 1;	disk->SetParameters(diskParams);	scene2D->ModifyObjectParameters( 1, diskParams );
	//std::cout<<"\nmeasuring the cost again, with a worst state..."<<std::endl;
	//costFunction->SelectOffContextComputation();
	//std::cout<<"off context value = "<<costFunction->GetValue()<<std::endl;
	//costFunction->SelectInContextComputation();
	//std::cout<<"in context value = "<<costFunction->GetValue()<<std::endl;


	std::cout<<"\n\n TestSensedSingleObjectToImage_MSE: completed successfully "<<std::endl;
}

//
//InOutCoherence
//
void TestSensedSingleObjectToImage_InOutContrast() {
	std::cout<<"\n\n TestSensedSingleObjectToImage_InOutContrast: draw a few spheres, use a SimplePixelBasedSEMSensor to capture images ; save that image ; create a second "<<std::endl;
	GenerateReferenceEllipsesImage();
	typedef itk::Image<unsigned char, 2> Image2DUCharType;
	typedef itk::Image<float, 2> Image2DFloatType;
	Image2DUCharType::Pointer referenceImage = ReadITKImageFromFile<Image2DUCharType>("referenceEllipsesImage.png", 0.5 );


	shape3DSphere::Pointer sphereShape = shape3DSphere::New();
	Translation3DTransform::Pointer translation3D = Translation3DTransform::New();
	PoseTransformedBinaryShape<3>::Pointer sphere = PoseTransformedBinaryShape<3>::New();

	vnl_vector<double> sphereRadius(1);	sphereRadius(0) = 2.215;	sphereShape->SetParameters(sphereRadius);
	vnl_vector<double> trans3D(3);		trans3D.fill(0);			translation3D->SetParameters(trans3D);
	sphere->SetShapeAndTransform(sphereShape, translation3D);

	typedef ParametricObject<3> Base3DObjectType;	
	typedef LabelMapScene<3> Scene3DType;
	typedef itk::Image<unsigned char, 3> Image3DType;
	typedef SimplePixelBasedSEMSensor<Scene3DType, Image2DFloatType> SEMSensorType;


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


	std::cout<<"added "<<scene3D->GetNumberOfObjects()<<" spheres to the scene"<<std::endl;
	std::cout<<"writing 3D representations of the scene to files: test3DScene.vtk and test3DScene_Labels.nii"<<std::endl;
	WriteMirrorPolyDataToFile("test3DScene.vtk", scene3D->GetSceneAsVTKPolyData());
	std::cout<<"writing 3D label image"<<std::endl;
	Scene3DType::LabelMapType::Pointer labelMap = scene3D->GetSceneAsLabelMap();
	typedef itk::LabelMapToLabelImageFilter< Scene3DType::LabelMapType, Scene3DType::LabelImageType> LabelMapToLabelImageFilterType;
	LabelMapToLabelImageFilterType::Pointer m_labelMapToLabelImageFilter = LabelMapToLabelImageFilterType::New();
	m_labelMapToLabelImageFilter->SetInput(labelMap);
	m_labelMapToLabelImageFilter->Update();
	WriteITKImageToFile<Scene3DType::LabelImageType>("test3DScene_Labels.nii", m_labelMapToLabelImageFilter->GetOutput());
	std::cout<<"written 3D label image"<<std::endl;

	//observing the scene through a 3D/2D sensor
	SEMSensorType::Pointer sensor3D = SEMSensorType::New();
	sensor3D->SetScene(scene3D);
	vnl_vector<double> appearanceParams(1);appearanceParams(0)=20000; sensor3D->SetAppearanceParameters(appearanceParams);
	//by default, with a HORIZONTAL view, and a exponentially decreasing appearance function, with max = 250 (decreases with the distance to the sensor)

	std::cout<<"writing the sensed image"<<std::endl;
	Write2DGreyLevelRescaledImageToFile<Image2DFloatType>("testSensed3DScene.png", sensor3D->GetOutput());


	//typedef SensedSingleObjectInOutCoherence<Scene3DType, Image2DUCharType, Image2DFloatType> SingleObjectCostFunctionType;
	typedef SensedSingleObjectInOutContrast<Scene3DType, Image2DUCharType, Image2DFloatType> SingleObjectCostFunctionType;
	
	SingleObjectCostFunctionType::Pointer costFunction = SingleObjectCostFunctionType::New();
	costFunction->SetScene(scene3D);
	costFunction->SetReferenceImage(referenceImage);
	costFunction->SetSensor(sensor3D);

	std::cout<<"MEASURING the cost for object 1"<<std::endl;
	costFunction->SelectObject(1);
	costFunction->SelectOffContextComputation();
	std::cout<<"off context value = "<<costFunction->GetValue()<<std::endl;
	costFunction->SelectInContextComputation();
	std::cout<<"in context value = "<<costFunction->GetValue()<<std::endl;

	std::cout<<"measuring the cost for object 2"<<std::endl;
	costFunction->SelectObject(2);
	costFunction->SelectOffContextComputation();
	std::cout<<"off context value = "<<costFunction->GetValue()<<std::endl;
	costFunction->SelectInContextComputation();
	std::cout<<"in context value = "<<costFunction->GetValue()<<std::endl;
	costFunction->SelectOffContextComputation();
	std::cout<<"off context value = "<<costFunction->GetValue()<<std::endl;
	costFunction->SelectInContextComputation();
	std::cout<<"in context value = "<<costFunction->GetValue()<<std::endl;

	std::cout<<"measuring the cost for object 3"<<std::endl;
	costFunction->SelectObject(3);
	costFunction->SelectOffContextComputation();
	std::cout<<"off context value = "<<costFunction->GetValue()<<std::endl;
	costFunction->SelectInContextComputation();
	std::cout<<"in context value = "<<costFunction->GetValue()<<std::endl;
	costFunction->SelectOffContextComputation();
	std::cout<<"off context value = "<<costFunction->GetValue()<<std::endl;
	costFunction->SelectInContextComputation();
	std::cout<<"in context value = "<<costFunction->GetValue()<<std::endl;


	std::cout<<"\n\n TestSensedSingleObjectToImage_InOutContrast: completed successfully "<<std::endl;
}


void TestSensedSingleObjectEnergy() {
	std::cout<<"\n\n TestSensedSingleObjectEnergy: draw a few spheres, use a SimplePixelBasedSEMSensor to capture images ; save that image ; create a second "<<std::endl;
	GenerateReferenceEllipsesImage();
	typedef itk::Image<unsigned char, 2> Image2DUCharType;
	typedef itk::Image<float, 2> Image2DFloatType;
	Image2DUCharType::Pointer referenceImage = ReadITKImageFromFile<Image2DUCharType>("referenceEllipsesImage.png", 0.5 );


	shape3DSphere::Pointer sphereShape = shape3DSphere::New();
	Translation3DTransform::Pointer translation3D = Translation3DTransform::New();
	PoseTransformedBinaryShape<3>::Pointer sphere = PoseTransformedBinaryShape<3>::New();

	vnl_vector<double> sphereRadius(1);	sphereRadius(0) = 2.215;	sphereShape->SetParameters(sphereRadius);
	vnl_vector<double> trans3D(3);		trans3D.fill(0);			translation3D->SetParameters(trans3D);
	sphere->SetShapeAndTransform(sphereShape, translation3D);

	typedef ParametricObject<3> Base3DObjectType;	
	typedef LabelMapScene<3> Scene3DType;
	typedef itk::Image<unsigned char, 3> Image3DType;
	typedef SimplePixelBasedSEMSensor<Scene3DType, Image2DFloatType> SEMSensorType;


	Scene3DType::Pointer scene3D = Scene3DType::New();
	//define the characteristics of the scene
	vnl_vector<double> sceneSpacing(3);
	sceneSpacing(0) = referenceImage->GetSpacing()[0]; sceneSpacing(1) = referenceImage->GetSpacing()[1]; sceneSpacing(2) = 1;
	vnl_vector<double> sceneBBox(6);	
	sceneBBox(0) = 0; sceneBBox(1) = referenceImage->GetLargestPossibleRegion().GetSize()[0]*sceneSpacing(0);
	sceneBBox(2) = 0; sceneBBox(3) = referenceImage->GetLargestPossibleRegion().GetSize()[1]*sceneSpacing(1); //
	sceneBBox(4) =-5; sceneBBox(5) = 5;
	scene3D->SetPhysicalDimensions(sceneBBox, sceneSpacing);

	//penalize object intersection
	typedef LabelObjectOverlap<Scene3DType> InteractionManagerType;
	InteractionManagerType::Pointer interactionManager = InteractionManagerType::New();
	scene3D->SetInteractionManager( (Scene3DType::ObjectInteractionManagerType *)interactionManager.GetPointer() );

	//observing the scene through a 3D/2D sensor
	SEMSensorType::Pointer sensor3D = SEMSensorType::New();
	sensor3D->SetScene(scene3D);
	vnl_vector<double> appearanceParams(1);appearanceParams(0)=20000; sensor3D->SetAppearanceParameters(appearanceParams);
	//by default, with a HORIZONTAL view, and a exponentially decreasing appearance function, with max = 250 (decreases with the distance to the sensor)


	//add a few objects to the scene
	vnl_vector<double> sphereParams(4);
clock_t t0=clock();
	sphereParams(0) = 10; sphereParams(1) = 10; sphereParams(2) = 0; sphereParams(3) = 2;   sphere->SetParameters(sphereParams); scene3D->AddObject(sphere);
	sphereParams(0) = 11; sphereParams(1) = 11; sphereParams(2) = 0; sphereParams(3) = 1.5; sphere->SetParameters(sphereParams); scene3D->AddObject(sphere);
	sphereParams(0) = 5; sphereParams(1) = 12.5; sphereParams(2) = 0; sphereParams(3) = 2.2;   sphere->SetParameters(sphereParams); scene3D->AddObject(sphere);
	

	std::cout<<"writing the sensed image"<<std::endl;
	Write2DGreyLevelRescaledImageToFile<Image2DFloatType>("testSensed3DScene.png", sensor3D->GetOutput());

	//parameters prior
	UniformBoxPDF::Pointer uniformTransPDF = UniformBoxPDF::New(); uniformTransPDF->SetBox(sceneBBox);
	TriangularPDF::Pointer radius_prior = TriangularPDF::New(); radius_prior->SetParameters(1, 2, 3);
	IndependentPDFs::Pointer diskLikelihoodPDF = IndependentPDFs::New(); diskLikelihoodPDF->AddMultivariatePDF(uniformTransPDF);diskLikelihoodPDF->AddUnivariatePDF(radius_prior);	
	scene3D->GetObjectTypesLibrary()->SetObjectPDF( scene3D->GetObject(1)->objectTypeId, PDF_OBJECTGENERATIONPRIOR, diskLikelihoodPDF );
	scene3D->GetObjectTypesLibrary()->SetObjectPDF( scene3D->GetObject(1)->objectTypeId, PDF_OBJECTLIKELIHOOD, diskLikelihoodPDF );

	//image-based term
	typedef SensedSingleObjectInOutContrast<Scene3DType, Image2DUCharType, Image2DFloatType> SingleObjectCostFunctionType;
	SingleObjectCostFunctionType::Pointer costFunction = SingleObjectCostFunctionType::New();
	costFunction->SetScene(scene3D);
	costFunction->SetReferenceImage(referenceImage);
	costFunction->SetSensor(sensor3D);

	//single object energy
	typedef SingleObjectEnergy<Scene3DType, Image2DUCharType> SingleObjectEnergyType;
	SingleObjectEnergyType::Pointer energyPotential = SingleObjectEnergyType::New();
	energyPotential->SetScene(scene3D);
	energyPotential->SetSingleObjectSceneToImageCostFunction(costFunction);
	energyPotential->UseObjectInteractionPrior(true);
	energyPotential->UseObjectPrior(true);
	
	//print energy of the different objects:
	energyPotential->PrintDetailsForObject(1);
	energyPotential->PrintDetailsForObject(2);
	energyPotential->PrintDetailsForObject(3);

	energyPotential->SelectObject(1);
	std::cout<<"value for object 1: "<<energyPotential->GetValue()<<std::endl;

	energyPotential->SelectObject(3);
	std::cout<<"value for object 3: "<<energyPotential->GetValue()<<std::endl;

	std::cout<<"\n\n TestSensedSingleObjectEnergy: completed successfully "<<std::endl;
}

void TestSceneEnergy() {
	std::cout<<"\n\n TestSceneEnergy -- test AverageObjectToImageCostFunction and SceneEnergy..."<<std::endl;
	
	shape2DEllipse::Pointer diskShape = shape2DEllipse::New();
	Rigid2DTransform::Pointer translation2D = Rigid2DTransform::New();
	PoseTransformedBinaryShape<2>::Pointer disk = PoseTransformedBinaryShape<2>::New();

	vnl_vector<double> diskRadius(2); diskRadius(0) = 2.1; diskRadius(1) = 1.2; diskShape->SetParameters(diskRadius);
	vnl_vector<double> trans2D(3);    trans2D.fill(0); trans2D(2) = PI/3.0;     translation2D->SetParameters(trans2D);
	disk->SetShapeAndTransform(diskShape, translation2D);

	typedef PoseTransformedBinaryShape<2>::BaseClass Base2DObjectType;

	typedef LabelMapScene<2> Scene2DType;

	Scene2DType::Pointer scene2D = Scene2DType::New();
	//define the characteristics of the scene
	vnl_vector<double> sceneSpacing(2);	sceneSpacing.fill(0.5);
	vnl_vector<double> sceneBBox(4);	for (unsigned i=0 ; i<2 ; i++) { sceneBBox(2*i) = -10;	sceneBBox(2*i+1) = 10; }
	scene2D->SetPhysicalDimensions(sceneBBox, sceneSpacing);

	//simple uniform sensor
	typedef itk::Image<unsigned char, 2> Image2DType;
	typedef SimpleUniformSensor<Scene2DType, Image2DType> SensorType;
	SensorType::Pointer sensor2D = SensorType::New();
	sensor2D->SetScene(scene2D);

	//penalize object intersection
	typedef LabelObjectOverlap<Scene2DType> InteractionManagerType;
	InteractionManagerType::Pointer interactionManager = InteractionManagerType::New();
	scene2D->SetInteractionManager( (Scene2DType::ObjectInteractionManagerType *)interactionManager.GetPointer() );


	//add a few objects to the scene
	vnl_vector<double> diskParams(5);
	diskParams(0) = 3;   diskParams(1) = -5;diskParams(2) = -PI/3.0; diskParams(3) = 1.5; diskParams(4) = 2; disk->SetParameters(diskParams); scene2D->AddObject(disk);
	diskParams(0) = 1;   diskParams(1) = 3; diskParams(2) = PI/8.0; diskParams(3) = 1.8; diskParams(4) = 1.5; disk->SetParameters(diskParams); scene2D->AddObject(disk);
	diskParams(0) =-4.2; diskParams(1) = 0; diskParams(2) = 5.0*PI/12.0; diskParams(3) = 1.2; diskParams(4) = 1.2; disk->SetParameters(diskParams); scene2D->AddObject(disk);
	diskParams(0) = 0; diskParams(1) = 0; diskParams(2) = 2.0*PI/3.0; diskParams(3) = 2.5; diskParams(4) = 2;  disk->SetParameters(diskParams); scene2D->AddObject(disk);


	//parameters prior
	UniformBoxPDF::Pointer uniformTransPDF = UniformBoxPDF::New(); uniformTransPDF->SetBox(sceneBBox);
	UniformPDF::Pointer uniformAnglePDF = UniformPDF::New(); uniformAnglePDF->SetParameters(-2.0*PI, 2.0*PI);
	TriangularPDF::Pointer radius_prior = TriangularPDF::New(); radius_prior->SetParameters(1, 2, 3);
	NormalPDF::Pointer elongationPrior = NormalPDF::New(); elongationPrior->SetParameters(1.5, 0.4);
	IndependentPDFs::Pointer diskLikelihoodPDF = IndependentPDFs::New(); diskLikelihoodPDF->AddMultivariatePDF(uniformTransPDF);diskLikelihoodPDF->AddUnivariatePDF(uniformAnglePDF);	
	diskLikelihoodPDF->AddUnivariatePDF(radius_prior);diskLikelihoodPDF->AddUnivariatePDF(elongationPrior);	
	scene2D->GetObjectTypesLibrary()->SetObjectPDF( scene2D->GetObject(1)->objectTypeId, PDF_OBJECTGENERATIONPRIOR, diskLikelihoodPDF );
	scene2D->GetObjectTypesLibrary()->SetObjectPDF( scene2D->GetObject(1)->objectTypeId, PDF_OBJECTLIKELIHOOD, diskLikelihoodPDF );

	//image-based term
	typedef SensedSingleObjectInOutContrast<Scene2DType, Image2DType, Image2DType> SingleObjectCostFunctionType;
	SingleObjectCostFunctionType::Pointer costFunction = SingleObjectCostFunctionType::New();
	costFunction->SetScene(scene2D);
	costFunction->SetReferenceImage(sensor2D->GetOutput());
	costFunction->SetSensor(sensor2D);
	costFunction->SelectOffContextComputation();

	//single object energy
	typedef SingleObjectEnergy<Scene2DType, Image2DType> SingleObjectEnergyType;
	SingleObjectEnergyType::Pointer energyPotential = SingleObjectEnergyType::New();
	energyPotential->SetScene(scene2D);
	energyPotential->SetSingleObjectSceneToImageCostFunction(costFunction);
	energyPotential->UseObjectInteractionPrior(true);
	energyPotential->UseObjectPrior(true);
	
	//Average energy...
	typedef AverageObjectToImageCostFunction<Scene2DType, Image2DType> AverageObjectMetricType;
	AverageObjectMetricType::Pointer averageMetric = AverageObjectMetricType::New();
	averageMetric->SetScene(scene2D);
	averageMetric->SetSingleObjectMetric( (AverageObjectMetricType::SOOCostFunctionType *)energyPotential.GetPointer() );

	//SceneEnergy
	typedef SceneEnergy<Scene2DType, Image2DType> SceneEnergyType;
	SceneEnergyType::Pointer sceneEnergy = SceneEnergyType::New();
	sceneEnergy->SetScene(scene2D);
	sceneEnergy->SetSingleObjectSceneToImageCostFunction(costFunction);
	sceneEnergy->UseGlobalScenePrior(true);
	sceneEnergy->UseObjectInteractionPrior(true);
	sceneEnergy->UseObjectPrior(true);
	sceneEnergy->SetSceneToImageCostFunction(averageMetric);


	//Test single object energy:
	energyPotential->PrintDetailsForObject(1);
	energyPotential->PrintDetailsForObject(2);
	energyPotential->PrintDetailsForObject(3);

	energyPotential->SelectObject(1);
	std::cout<<"energy potential for object 1: "<<energyPotential->GetValue()<<std::endl;

	energyPotential->SelectObject(3);
	std::cout<<"energy potential for object 3: "<<energyPotential->GetValue()<<std::endl;

	//AverageMetric:
	std::cout<<"average metric value: "<<averageMetric->GetValue()<<std::endl;
	//SceneEnergy
	sceneEnergy->PrintDetails();
	std::cout<<"SceneEnergy value: "<<sceneEnergy->GetValue()<<std::endl;


	std::cout<<"\n\n TestSceneEnergy: completed successfully "<<std::endl;
}
//
//main
//
int main(int argc, char** argv) {

	try {
		TestSensedSingleObjectToImage_MSE();
		TestSensedSingleObjectToImage_InOutContrast();

		TestSensedSingleObjectEnergy();

		TestSceneEnergy();
	}
	catch (DeformableModelException& e) {
		std::cout << "Exception occured while running the test cases" << std::endl;
		std::cout << e.what() << std::endl;
	}

}
