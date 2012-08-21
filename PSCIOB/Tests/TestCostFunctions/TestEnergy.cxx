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
#include "LabelMapScene.h"
#include "LabelImageScene.h"
#include "SimplePixelBasedSEMSensor.h"
#include "PixelSetObjectOverlap.h"

#include "SingleObjectEnergy.h"
#include "SceneEnergy.h"

#include "shape2DDisk.h"
#include "Translation2DTransform.h"
#include "SimpleUniformSensor.h"

#include "SensedSingleObjectToImageMeanSquareErrorCostFunction.h"
#include "SensedSingleObjectInOutCoherence.h"
#include "SensedSingleObjectInOutContrast.h"
#include "SensedSceneToImageMeanSquareErrorCostFunction.h"
#include "AverageObjectToImageCostFunction.h"

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

	Write2DGreyLevelRescaledImageToFile<Image2DType>("referenceDiskImage.png", sensor2D->GetOutput());
	
	Scene2DType::LabelMapType::Pointer labelMap = Scene2DType::LabelMapType::New();
	std::cout<<"getting off context labelmap"<<std::endl;
	sensor2D->GetOffContextObjectLabelMap( scene2D->GetObject(1), labelMap );
	std::cout<<"nb of label Objects: "<<labelMap->GetNumberOfLabelObjects()<<std::endl;
}

//
void TestSceneEnergy() {
	std::cout<<"\n\n TestSceneEnergy: draw a few spheres, use a SimpleSEMSensor to capture images ; save that image ; create a second "<<std::endl;

	//Generate a reference image
	GenerateReferenceDiskImage();
	//loads this image
	typedef itk::Image<unsigned char, 2> Image2DUCharType;
	typedef itk::Image<float, 2> Image2DFloatType;
	Image2DUCharType::Pointer referenceImage = ReadITKImageFromFile<Image2DUCharType>("referenceDiskImage.png", 1 );

	//define the characteristics of the scene with respect to the reference image.
	typedef ParametricObject<3,unsigned char>::BaseClass Base3DObjectType;	
	vnl_vector<double> sceneSpacing(3);
	sceneSpacing(0) = referenceImage->GetSpacing()[0];
	sceneSpacing(1) = referenceImage->GetSpacing()[1];
	sceneSpacing(2) = 1;
	vnl_vector<double> sceneBBox(6);	
	sceneBBox(0) = 0; sceneBBox(1) = referenceImage->GetLargestPossibleRegion().GetSize()[0]*sceneSpacing(0);
	sceneBBox(2) = 0; sceneBBox(3) = referenceImage->GetLargestPossibleRegion().GetSize()[1]*sceneSpacing(1); //
	sceneBBox(4) =-10; sceneBBox(5) = 10;

	//define the type of object we assume present in the reference image
	//shape3DSphere::Pointer sphereShape = shape3DSphere::New();
	//Translation3DTransform::Pointer translation3D = Translation3DTransform::New();
	//PoseTransformedBinaryShape<3>::Pointer sphere = PoseTransformedBinaryShape<3>::New();
	//sphere->SetShapeAndTransform(sphereShape, translation3D);
	Direct3DSphere::Pointer sphere = Direct3DSphere::New();

	//Set likelihoods pdf for sphere parameters:
	UniformBoxPDF::Pointer uniformTransPDF = UniformBoxPDF::New(); uniformTransPDF->SetBox(sceneBBox);
	TriangularPDF::Pointer radiusGeneratePrior = TriangularPDF::New(); radiusGeneratePrior->SetParameters(3, 5, 7);
	TrapezoidalPDF::Pointer radiusLikelihoodPrior = TrapezoidalPDF::New(); radiusLikelihoodPrior->SetParameters(3, 4, 6, 7);
	IndependentPDFs::Pointer sphereGeneratePDF = IndependentPDFs::New(); sphereGeneratePDF->AddMultivariatePDF(uniformTransPDF);sphereGeneratePDF->AddUnivariatePDF(radiusGeneratePrior);
	IndependentPDFs::Pointer sphereLikelihoodPDF = IndependentPDFs::New(); sphereLikelihoodPDF->AddMultivariatePDF(uniformTransPDF);sphereLikelihoodPDF->AddUnivariatePDF(radiusLikelihoodPrior);

	//Define the scene and helper classes
//VERSION WITH PIXELSET SCENE
//	typedef PixelSetScene<Base3DObjectType, unsigned short, CostsAndPixelSetContainer, PixelSetIntersectionContainer> Scene3DType;
//	typedef PixelSetObjectOverlap<Scene3DType> InteractionManagerType;
//	Scene3DType::Pointer scene3D = Scene3DType::New();
//	InteractionManagerType::Pointer interactionManager = InteractionManagerType::New();
//	//set the overlap interaction manager
//	scene3D->SetInteractionManager( (Scene3DType::ObjectInteractionManagerType *)interactionManager.GetPointer() );
//VERSION WITH LABELIMAGE SCENE	
	typedef LabelImageScene<3> Scene3DType;
	Scene3DType::Pointer scene3D = Scene3DType::New();


	//associate PDFs with the object types
	unsigned int objectTypeID = scene3D->GetObjectTypesLibrary()->RegisterObjectType(sphere);
	scene3D->GetObjectTypesLibrary()->SetObjectPDF(objectTypeID, PDF_OBJECTGENERATIONPRIOR, sphereGeneratePDF);
	scene3D->GetObjectTypesLibrary()->SetObjectPDF(objectTypeID, PDF_OBJECTLIKELIHOOD, sphereLikelihoodPDF);

	//Define the sensor
	typedef SimplePixelBasedSEMSensor<Scene3DType, Image2DFloatType> SensorType;
	SensorType::Pointer sensor3D = SensorType::New();
	sensor3D->SetScene(scene3D);
	sensor3D->SetObservationDirectionType(HORIZONTAL);
	vnl_vector<double> appearanceParams(2); appearanceParams(0)=250; appearanceParams(1)=1.0/20.0;
	sensor3D->SetAppearanceParameters(appearanceParams);


	//
	scene3D->SetPhysicalDimensions(sceneBBox, sceneSpacing);
	std::cout<<"sceneBBox: "<<sceneBBox<<std::endl;
	std::cout<<"sceneSpacing: "<<sceneSpacing<<std::endl;
	//add a few objects to the scene
	vnl_vector<double> sphereParams(4);

	sphereParams(0) = 6; sphereParams(1) = 12; sphereParams(2) = 0; sphereParams(3) = 4; sphere->SetParameters(sphereParams); 
	Scene3DType::IDType id = scene3D->AddObject(sphere);
	sphereParams(0) = 3; sphereParams(1) = 13; sphereParams(2) = 0; sphereParams(3) = 3.5; sphere->SetParameters(sphereParams); scene3D->AddObject(sphere);
	sphereParams(0) = 15; sphereParams(1) = 5; sphereParams(2) = 0; sphereParams(3) = 4.5; sphere->SetParameters(sphereParams); scene3D->AddObject(sphere);
	std::cout<<"added a few objects: "<<scene3D->GetNumberOfObjects()<<std::endl;

	//Define single-object-based image similarity measures
	typedef SensedSingleObjectToImageMeanSquareErrorCostFunction<Scene3DType, Image2DUCharType, Image2DFloatType> SOOCF1;
	typedef SensedSingleObjectInOutCoherence<Scene3DType, Image2DUCharType, Image2DFloatType> SOOCF2;
	typedef SensedSingleObjectInOutContrast<Scene3DType, Image2DUCharType, Image2DFloatType> SOOCF3;
	SOOCF1::Pointer soocf1 = SOOCF1::New(); soocf1->SetReferenceImage(referenceImage); soocf1->SetScene(scene3D); soocf1->SetSensor(sensor3D);
	SOOCF2::Pointer soocf2 = SOOCF2::New(); soocf2->SetReferenceImage(referenceImage); soocf2->SetScene(scene3D); soocf2->SetSensor(sensor3D);
	SOOCF3::Pointer soocf3 = SOOCF3::New(); soocf3->SetReferenceImage(referenceImage); soocf3->SetScene(scene3D); soocf3->SetSensor(sensor3D);

	//test 1st metric
	soocf1->SelectObject(1); std::cout<<"selected object1... computing its cost"<<std::endl;
	soocf1->SelectInContextComputation(); 
	std::cout<<"selecting incontext computation ; object parameters: "<<scene3D->GetParametersOfObject(1);
	std::cout<<"1st cost for object 1, in context : "<<soocf1->GetValue()<<std::endl;
	soocf1->SelectOffContextComputation();std::cout<<"1st cost for object 1, off context: "<<soocf1->GetValue()<<std::endl;
//	scene3D->InvalidateObjectDataCosts(1);
	//test 2nd metric
	soocf2->SelectObject(1); 
	soocf2->SelectInContextComputation(); std::cout<<"1st cost for object 1, in context : "<<soocf2->GetValue()<<std::endl;
	soocf2->SelectOffContextComputation();std::cout<<"1st cost for object 1, off context: "<<soocf2->GetValue()<<std::endl;
//	scene3D->InvalidateObjectDataCosts(1);
	//test 3rd metric
	soocf3->SelectObject(1); 
	soocf3->SelectInContextComputation(); std::cout<<"1st cost for object 1, in context : "<<soocf3->GetValue()<<std::endl;
	soocf3->SelectOffContextComputation();std::cout<<"1st cost for object 1, off context: "<<soocf3->GetValue()<<std::endl;
//	scene3D->InvalidateObjectDataCosts(1);

	//test average soo metric
	typedef AverageObjectToImageCostFunction<Scene3DType, Image2DUCharType> AvSOOCF;
	AvSOOCF::Pointer avsoocf = AvSOOCF::New();
	avsoocf->SetScene(scene3D);avsoocf->SetSingleObjectMetric(soocf3);
	std::cout<<"average soo metric using the 3rd..."<<avsoocf->GetValue()<<std::endl;
//	scene3D->InvalidateObjectDataCosts();

	//test whole image metric
	typedef SensedSceneToImageMeanSquareErrorCostFunction<Scene3DType, Image2DUCharType, Image2DFloatType> WholeImageCostFunctionType;
	WholeImageCostFunctionType::Pointer costFunction = WholeImageCostFunctionType::New();
	costFunction->SetScene(scene3D); costFunction->SetReferenceImage(referenceImage); costFunction->SetSensor(sensor3D);
	std::cout<<"Whole image metric: "<<costFunction->GetValue()<<std::endl;

	//Single Object Energy
	typedef SingleObjectEnergy<Scene3DType, Image2DUCharType> SOOENERGY;
	SOOENERGY::Pointer sooEnergy = SOOENERGY::New(); sooEnergy->SetScene(scene3D); 
	sooEnergy->SetSingleObjectSceneToImageCostFunction(soocf1);
	sooEnergy->UseObjectInteractionPrior(true);
	sooEnergy->UseObjectPrior(true);
	sooEnergy->UseSingleObjectSceneToImageCostFunction(true);

	std::cout<<"browsing all objects in the scene, and printing energy details"<<std::endl;
	SceneObjectIterator<Scene3DType> it(scene3D);
	for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
		sooEnergy->PrintDetailsForObject(it.GetID());
	}
//	scene3D->InvalidateObjectDataCosts();


	//Now, test the scene energy:
	typedef SceneEnergy<Scene3DType, Image2DUCharType> EnergyType;
	EnergyType::Pointer sceneEnergy = EnergyType::New();
	sceneEnergy->SetScene(scene3D);
	sceneEnergy->SetSingleObjectSceneToImageCostFunction(soocf2);
	sceneEnergy->SetSceneToImageCostFunction(costFunction);

	std::cout<<"scene energy: "<<sceneEnergy->GetValue()<<std::endl;
	for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
		sceneEnergy->PrintDetailsForObject(it.GetID());
	}

	std::cout<<"\n\nAdd a random object to the scene and compute the energy"<<std::endl;
	Base3DObjectType::Pointer object = scene3D->GetObjectTypesLibrary()->GenerateNewRandomObject(objectTypeID);
	std::cout<<"toto"<<std::endl;
	std::cout<<"parameters of the generated object: "<<object->GetParameters()<<std::endl;
	id = scene3D->AddObject(object);
	std::cout<<"attributed id: "<<id<<", number of interactions: "<<scene3D->GetObject(id)->interactionData.size()<<std::endl;
	std::cout<<"Energy value: "<<sceneEnergy->GetValue()<<std::endl;
	for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
		sceneEnergy->PrintDetailsForObject(it.GetID());
	}

	vnl_vector<double> params;
	std::cout<<"\n\nModify a parameter of that random object and re-compute the energy"<<std::endl;
	params = scene3D->GetParametersOfObject(id);
	params(1)+=0.33;
	scene3D->ModifyObjectParameters(id, params);
	std::cout<<"new parameters of the generated object: "<<object->GetParameters()<<std::endl;
	std::cout<<"Energy value: "<<sceneEnergy->GetValue()<<std::endl;
	for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
		sceneEnergy->PrintDetailsForObject(it.GetID());
	}

	std::cout<<"\n\nRemoving that object from the scene and re-compute the energy"<<std::endl;
	scene3D->RemoveObject(id);
	std::cout<<"Energy value: "<<sceneEnergy->GetValue()<<std::endl;
	for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) {
		sceneEnergy->PrintDetailsForObject(it.GetID());
	}


	std::cout<<"\nAdding a random object compute the energy, modify it, remove it, recompute ; and repeat to check for computation time , memory leaks, ..."<<std::endl;
	clock_t t0 = clock();
	
	for (unsigned i=0 ; i<1001 ; i++) {
		id = scene3D->AddObject( scene3D->GetObjectTypesLibrary()->GenerateNewRandomObject(objectTypeID) );
		sceneEnergy->GetValue();
		params = scene3D->GetParametersOfObject(id);
		params(1)+=0.33;
		scene3D->ModifyObjectParameters(id, params);
		sceneEnergy->GetValue();
		scene3D->RemoveObject(id);
		sceneEnergy->GetValue();

		if (i==100) std::cout<<"100 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s. - id = "<<id<<std::endl;
		if (i==500) std::cout<<"500 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s. - id = "<<id<<std::endl;
		if (i==1000) std::cout<<"1000 iterations, after "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s. - id = "<<id<<std::endl;
	}


	std::cout<<"\n\n TestSceneEnergy: completed successfully "<<std::endl;
}




int main(int argc, char** argv) {

	try {
		TestSceneEnergy();
	}
	catch (DeformableModelException& e) {
		std::cout << "Exception occured while running the test cases" << std::endl;
		std::cout << e.what() << std::endl;
	}

}
