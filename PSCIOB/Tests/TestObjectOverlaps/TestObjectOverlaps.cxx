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

#include <GeneralUtils.h>

#include "LabelImageScene.h"
#include "Fast2DEllipse.h"

#include "LabelMapScene.h"
#include "PixelSetObjectOverlap.h"
#include "ObjectOverlap_Pow2DTree.h"
#include "LabelObjectOverlap.h"

#include <SimpleUniformSensor.h>

using namespace psciob;

//
// Test adding objects, and exploiting the quadtree structure...
void TestOverlap_PixelSet_QuadTree() {

	//Scene                                                ObjectCostsAndPixelSetContainer  PixelSetIntersectionContainer
	//typedef LabelMapScene<2,unsigned char, unsigned short, ObjectCostsAndPixelSetContainer, PixelSetIntersectionContainer> SceneType;
	typedef LabelMapScene<2,unsigned char, unsigned short, ObjectCostsContainer, LabelObjectIntersectionContainer<unsigned short, 2>> SceneType;
	SceneType::Pointer scene = SceneType::New();
	vnl_vector<double> bbox(4); bbox.fill(0); bbox(1) = 100; bbox(3) = 100;
	scene->SetPhysicalDimensions(bbox, 1);
	std::cout<<"The scene is defined"<<std::endl; 

	//interaction manager
	//typedef ObjectsWithoutInteraction<SceneType> ObjectInteractionManagerType;
	//typedef PixelSetObjectOverlap<SceneType> ObjectInteractionManagerType;
	//typedef ObjectOverlap_Pow2DTree<SceneType::BaseClass, PixelSetObjectOverlap<SceneType::BaseClass>> ObjectInteractionManagerType;
	typedef ObjectOverlap_Pow2DTree<SceneType::BaseClass, LabelObjectOverlap<SceneType::BaseClass>> ObjectInteractionManagerType;
	ObjectInteractionManagerType::Pointer interactionManager = ObjectInteractionManagerType::New();
	scene->SetInteractionManager( (SceneType::ObjectInteractionManagerType *)interactionManager.GetPointer() );
	std::cout<<"The interaction manager is set"<<std::endl; 

	//sensor
	typedef SimpleUniformSensor<SceneType> SensorType;
	SensorType::Pointer sensor = SensorType::New();
	sensor->SetScene(scene);
	vnl_vector<double> sensorAppearanceParam(1); sensorAppearanceParam(0) = 200;
	sensor->SetAppearanceParameters(sensorAppearanceParam);
	std::cout<<"sensor is defined"<<std::endl;
	
	//Now, add, modify and remove objects from the scene, to check how the tree works, and intersection are detected...
	unsigned nb=0;std::string name; 
	Fast2DEllipse::Pointer object = Fast2DEllipse::New(); vnl_vector<double> params(5);

	//prior for generating ellipses
	unsigned int typeCode = scene->GetObjectTypesLibrary()->RegisterObjectType(object);
	UniformBoxPDF::Pointer uniformTransPDF = UniformBoxPDF::New(); uniformTransPDF->SetBox(scene->GetSceneBoundingBox());
	UniformPDF::Pointer    uniformRadiusPDF = UniformPDF::New();   uniformRadiusPDF->SetParameters(2, 10);
	UniformPDF::Pointer    uniformElongPDF = UniformPDF::New();    uniformElongPDF->SetParameters(0.45, 1);
	UniformPDF::Pointer    uniformAnglePDF = UniformPDF::New();    uniformAnglePDF->SetParameters(0, PI);
	IndependentPDFs::Pointer ellipseGenerationPDF = IndependentPDFs::New();
	ellipseGenerationPDF->AddMultivariatePDF(uniformTransPDF); ellipseGenerationPDF->AddUnivariatePDF(uniformRadiusPDF);
	ellipseGenerationPDF->AddUnivariatePDF(uniformElongPDF);   ellipseGenerationPDF->AddUnivariatePDF(uniformAnglePDF);

	scene->GetObjectTypesLibrary()->SetObjectPDF(typeCode, PDF_OBJECTGENERATIONPRIOR, ellipseGenerationPDF);

	//pdf for random walk
	vnl_vector<double> meanRW(5); meanRW.fill(0); vnl_matrix<double> covRW(5,5); covRW.fill(0);
	covRW(0,0) = 1; covRW(1,1) = 1; //translation
	covRW(2,2) = 0.5*0.5; covRW(3,3) = 0.02*0.02; covRW(4,4) = (5.0*PI/180.0)*(5.0*PI/180.0);
	MVN_PDF::Pointer ellipseRWPDF = MVN_PDF::New();  ellipseRWPDF->SetParameters(meanRW, covRW);
	scene->GetObjectTypesLibrary()->SetObjectPDF(typeCode, PDF_RANDOMWALK, ellipseRWPDF);

	//util variables
	SceneType::IDType id; unsigned int requestedIndex; SceneObjectIterator<SceneType> objectIt(scene);
	RandomVariableGenerator::Pointer rndgen = RandomVariableGenerator::New();
	SensorType::OutputImageType::Pointer sensorImage;
	//start!
	std::cout<<"Start adding objects..."<<std::endl;
	params(0) = 39.0295; params(1) =  13.8283; params(2) = 6.03666; params(3) =  0.500162; params(4) = 0.915307; object->SetParameters(params); 
	
	scene->AddObject(object);
	std::cout<<"Added first object to the scene..."<<std::endl;
	nb++;name = "overlap_" + stringify(nb) + ".png"; //Write2DGreyLevelRescaledImageToFile<SensorType::OutputImageType>(name, sensor->GetOutput());
	params(0) = 42.0589; params(1) =  49.9891; params(2) = 6.16854; params(3) =  0.836838; params(4) = 2.1312  ; object->SetParameters(params); scene->AddObject(object);
	std::cout<<"Added second object to the scene..."<<std::endl;
	nb++;name = "overlap_" + stringify(nb) + ".png"; //Write2DGreyLevelRescaledImageToFile<SensorType::OutputImageType>(name, sensor->GetOutput());
	std::cout<<"Added second object to the scene..."<<std::endl;
	params(0) = 14.0115; params(1) =  14.6685; params(2) = 4.39277; params(3) =  0.588911; params(4) = 0.799745; 	std::cout<<"Added second object to the scene..."<<std::endl;
object->SetParameters(params); 	std::cout<<"Adding 3rd object to the scene..."<<std::endl;
scene->AddObject(object);
	std::cout<<"Added third object to the scene..."<<std::endl;
	nb++;name = "overlap_" + stringify(nb) + ".png"; //Write2DGreyLevelRescaledImageToFile<SensorType::OutputImageType>(name, sensor->GetOutput());
	params(0) = 18.0507; params(1) =  35.0293; params(2) = 4.00183; params(3) =  0.728091; params(4) = 1.56986 ; object->SetParameters(params); scene->AddObject(object);
	std::cout<<"Added fourth object to the scene..."<<std::endl;
	nb++;name = "overlap_" + stringify(nb) + ".png"; Write2DGreyLevelRescaledImageToFile<SensorType::OutputImageType>(name, sensor->GetOutput());
	params(0) = 26.0634; params(1) =  4.9733 ; params(2) = 4.97155; params(3) =  0.633991; params(4) = 1.53275 ; object->SetParameters(params); scene->AddObject(object);
	std::cout<<"Added fifth object to the scene..."<<std::endl;
	sensorImage = sensor->GetOutput();
	std::cout<<"..toto"<<std::endl;
	std::cout<<"size of sensor image: "<<sensorImage->GetLargestPossibleRegion().GetSize(0)<<std::endl;
	std::cout<<"writing image..."<<std::endl;
	nb++;name = "overlap_" + stringify(nb) + ".png"; Write2DGreyLevelRescaledImageToFile<SensorType::OutputImageType>(name, sensorImage);
	params(0) = 9.99796; params(1) =  35.083 ; params(2) = 4.03536; params(3) =  0.708747; params(4) = 1.57316 ; object->SetParameters(params); scene->AddObject(object);
	std::cout<<"Added 6 object to the scene..."<<std::endl;
	nb++;name = "overlap_" + stringify(nb) + ".png"; Write2DGreyLevelRescaledImageToFile<SensorType::OutputImageType>(name, sensor->GetOutput());
	params(0) = 72.72847; params(1) =  10.0644; params(2) = 7.73396; params(3) =  0.50603 ; params(4) = 0.856896; object->SetParameters(params); scene->AddObject(object);
	std::cout<<"Added 7 object to the scene..."<<std::endl;
	nb++;name = "overlap_" + stringify(nb) + ".png"; Write2DGreyLevelRescaledImageToFile<SensorType::OutputImageType>(name, sensor->GetOutput());
	params(0) = 88.7036; params(1) =  63.1676; params(2) = 3.66358; params(3) =  0.510954; params(4) = 1.83045 ; object->SetParameters(params); scene->AddObject(object);
	std::cout<<"Added 8 object to the scene..."<<std::endl;
	nb++;name = "overlap_" + stringify(nb) + ".png"; Write2DGreyLevelRescaledImageToFile<SensorType::OutputImageType>(name, sensor->GetOutput());
	params(0) = 20.3914; params(1) =  49.5284; params(2) = 5.17163; params(3) =  0.721595; params(4) = 0.889653; object->SetParameters(params); scene->AddObject(object);
	std::cout<<"Added 9 object to the scene..."<<std::endl;
	nb++;name = "overlap_" + stringify(nb) + ".png"; Write2DGreyLevelRescaledImageToFile<SensorType::OutputImageType>(name, sensor->GetOutput());
	params(0) = 90.3914; params(1) =  22.5284; params(2) = 5.17163; params(3) =  0.721595; params(4) = 0.889653; object->SetParameters(params); scene->AddObject(object);
	std::cout<<"Added 10 object to the scene..."<<std::endl;
	nb++;name = "overlap_" + stringify(nb) + ".png"; Write2DGreyLevelRescaledImageToFile<SensorType::OutputImageType>(name, sensor->GetOutput());

	std::cout<<"browsing all objects in the scene, and checking for interactions"<<std::endl;
	for (objectIt.GoToBegin() ; !objectIt.IsAtEnd() ; ++objectIt) {
		for (SceneType::ObjectInteractionIterator iit = objectIt.GetObjectInScene()->interactionData.begin() ; iit!=objectIt.GetObjectInScene()->interactionData.end() ; iit++) {
			if (objectIt.GetID()>iit->first) std::cout<<"  interaction between object: "<<objectIt.GetID()<<" and "<<iit->first<<", interaction cost: "<<iit->second.interactionCost<<", nb of overlapping pixels: "<<iit->second.nbOverlappingPixels<<std::endl;
		}
	}	


	std::cout<<"\n\nRemoving randomly 9 objects"<<std::endl;
	for (unsigned i=0 ; i<9 ; i++) {
		requestedIndex = rndgen->GetIntegerVariate(scene->GetNumberOfObjects()-1);
		objectIt.GoToBegin(); objectIt.Advance(requestedIndex); id = objectIt.GetID();
		scene->RemoveObject(id);
		nb++;name = "overlap_" + stringify(nb) + ".png"; Write2DGreyLevelRescaledImageToFile<SensorType::OutputImageType>(name, sensor->GetOutput());
	}

	std::cout<<"\n\nAdding 29 random objects..."<<std::endl;
	//add 25 more objects.
	for (unsigned i=0 ; i<29 ; i++) {
		params = scene->GetObjectTypesLibrary()->GetObjectPDF(typeCode, PDF_OBJECTGENERATIONPRIOR)->DrawSample();
		object->SetParameters(params);
		scene->AddObject(object);
		nb++;name = "overlap_" + stringify(nb) + ".png"; Write2DGreyLevelRescaledImageToFile<SensorType::OutputImageType>(name, sensor->GetOutput());
	}

	std::cout<<"\n\nModifying randomly 50 objects..."<<std::endl;
	//modify a random object randomly, 50 times.
	for (unsigned i=0 ; i<50 ; i++) {
		requestedIndex = rndgen->GetIntegerVariate(scene->GetNumberOfObjects()-1);
		objectIt.GoToBegin(); objectIt.Advance(requestedIndex); id = objectIt.GetID();
		params = scene->GetParametersOfObject(id);
		params += scene->GetObjectTypesLibrary()->GetObjectPDF(typeCode, PDF_RANDOMWALK)->DrawSample();
		scene->ModifyObjectParameters(id, params);
		nb++;name = "overlap_" + stringify(nb) + ".png"; Write2DGreyLevelRescaledImageToFile<SensorType::OutputImageType>(name, sensor->GetOutput());
	}

	std::cout<<"\n\nRemoving randomly 20 objects"<<std::endl;
	for (unsigned i=0 ; i<20 ; i++) {
		requestedIndex = rndgen->GetIntegerVariate(scene->GetNumberOfObjects()-1);
		objectIt.GoToBegin(); objectIt.Advance(requestedIndex); id = objectIt.GetID();
		scene->RemoveObject(id);
		nb++;name = "overlap_" + stringify(nb) + ".png"; Write2DGreyLevelRescaledImageToFile<SensorType::OutputImageType>(name, sensor->GetOutput());
	}

}


//
int main(int argc, char** argv) {

	try {
		TestOverlap_PixelSet_QuadTree();
	}
	catch (DeformableModelException& e) {
		std::cout << "Exception occured while running the test cases" << std::endl;
		std::cout << e.what() << std::endl;
	}

}
