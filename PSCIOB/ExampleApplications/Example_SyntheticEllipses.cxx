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
#include "Example_Ellipses2D_Synthese.h"

#include "LabelMapScene.h"
#include "PixelSetObjectOverlap.h"
#include "ObjectOverlap_Pow2DTree.h"
#include "LabelObjectOverlap.h"

#include <SimpleUniformSensor.h>

#include <SceneEnergy.h>
#include <SensedSingleObjectInOutMMahalanobisDistance.h>
//#include <SensedSingleObjectToImageMeanAbsoluteDifferenceCostFunction.h>

#include <SceneStateOptimizationManager.h>
#include <RandomJumpSAOptimizer.h>

using namespace psciob;

//
// In this example, we synthesize a 2D image representing ellipses
// and perform their detection using a simple RJ-MCMC chain 
//
//
int main(int argc, char** argv) {

	bool verbose = true;
	try{
		/*
		* GENERATE THE REFERENCE IMAGE
		*/
		typedef itk::Image<unsigned char, 2> ReferenceImageType;

		unsigned int sceneCode = 1;	double       blurSize  = 0;	double       noisePower= 0;	int          rndSeed   = 0;

		Example_Ellipses2D_Synthese sceneGenerator;
		sceneGenerator.SelectSceneGenerationMethod(1);
		switch(sceneCode) {
			case 1: sceneGenerator.GenerateScene(); break;
			default: throw DeformableModelException("unknown scene code..."); break;
		}
		sceneGenerator.SetBlurStd(blurSize);
		sceneGenerator.SetNoiseStd(noisePower);
		sceneGenerator.SetNoiseSeed(rndSeed);

		Write2DGreyLevelRescaledImageToFile<Example_Ellipses2D_Synthese::ImageType>("ReferenceImage.png", sceneGenerator.GetImage());
return 1;
		/*
		* LOAD THE INPUT IMAGE
		*/
		ReferenceImageType::Pointer referenceImage = ReadITKImageFromFile<ReferenceImageType>("ReferenceImage.png", 1 );

		/*
		* DETECTION STEPS
		*/

		/* 
		* Define the scene 
		*/
		typedef LabelMapScene<2,unsigned char, unsigned short, ObjectCostsContainer, LabelObjectIntersectionContainer<unsigned short, 2>> SceneType;

		SceneType::Pointer scene = SceneType::New();
		vnl_vector<double> bbox = BoundingBoxFromITKImage<ReferenceImageType>(referenceImage);
		scene->SetPhysicalDimensions(bbox, referenceImage->GetSpacing().GetVnlVector());
		if (verbose) std::cout<<"The scene is defined, bbox: "<<scene->GetSceneBoundingBox()<<std::endl; 

		//typedef ObjectOverlap_Pow2DTree<SceneType::BaseClass, PixelSetObjectOverlap<SceneType::BaseClass>> ObjectInteractionManagerType;
		typedef ObjectOverlap_Pow2DTree<SceneType::BaseClass, LabelObjectOverlap<SceneType::BaseClass>> ObjectInteractionManagerType;
		ObjectInteractionManagerType::Pointer interactionManager = ObjectInteractionManagerType::New();
		scene->SetInteractionManager(interactionManager);

		/* 
		* Define a generative distribution for ellipse birth...
		*/ 
		Fast2DEllipse::Pointer sampleEllipse = Fast2DEllipse::New();
		//sampleEllipse->SetVTKPolyDataResolution(24);		

		unsigned int typeCode = scene->GetObjectTypesLibrary()->RegisterObjectType(sampleEllipse);

		UniformBoxPDF::Pointer uniformTransPDF = UniformBoxPDF::New(); uniformTransPDF->SetBox(scene->GetSceneBoundingBox());
		UniformPDF::Pointer    uniformRadiusPDF = UniformPDF::New();   uniformRadiusPDF->SetParameters(3, 10);
		UniformPDF::Pointer    uniformElongPDF = UniformPDF::New();    uniformElongPDF->SetParameters(0.25, 1);
		UniformPDF::Pointer    uniformAnglePDF = UniformPDF::New();    uniformAnglePDF->SetParameters(0, PI);

		IndependentPDFs::Pointer ellipseGenerationPDF = IndependentPDFs::New();
		ellipseGenerationPDF->AddMultivariatePDF(uniformTransPDF); ellipseGenerationPDF->AddUnivariatePDF(uniformRadiusPDF);
		ellipseGenerationPDF->AddUnivariatePDF(uniformElongPDF);   ellipseGenerationPDF->AddUnivariatePDF(uniformAnglePDF);

		scene->GetObjectTypesLibrary()->SetObjectPDF(typeCode, PDF_OBJECTGENERATIONPRIOR, ellipseGenerationPDF);
		if (verbose) std::cout<<"defined the generative pdf for ellipses, test sample: "<<ellipseGenerationPDF->DrawSample()<<std::endl; 

		/*
		* Define a proposal distribution for random modifications of the parameters of an object.
		*/

		vnl_vector<double> meanRW(5); meanRW.fill(0); vnl_matrix<double> covRW(5,5); covRW.fill(0);
		covRW(0,0) = 2; covRW(1,1) = 2; //translation
		covRW(2,2) = 0.5*0.5; covRW(3,3) = 0.02*0.02; covRW(4,4) = (5.0*PI/180.0)*(5.0*PI/180.0);
		MVN_PDF::Pointer ellipseRWPDF = MVN_PDF::New();  ellipseRWPDF->SetParameters(meanRW, covRW);
		
		scene->GetObjectTypesLibrary()->SetObjectPDF(typeCode, PDF_RANDOMWALK, ellipseRWPDF);
		if (verbose) std::cout<<"defined the random walk pdf for ellipses, test sample: "<<ellipseRWPDF->DrawSample()<<std::endl; 

		/* 
		* Define a prior distribution for assessing the likelihood of an object
		* and a normalization function so that the good objects have ~0 cost (rather in [0 e], with e small)
		* and unlikely object have a larger, positive cost
		*/
		std::cout<<"scene bbox: "<<scene->GetSceneBoundingBox()<<std::endl;
		UniformBoxPDF::Pointer priorTransPDF = UniformBoxPDF::New();    priorTransPDF->SetBox(scene->GetSceneBoundingBox());
		TrapezoidalPDF::Pointer priorRadiusPDF = TrapezoidalPDF::New(); priorRadiusPDF->SetParameters(1.5,3,10,10.5);
		TrapezoidalPDF::Pointer priorElongPDF = TrapezoidalPDF::New();  priorElongPDF->SetParameters(0.2,0.25,0.99,1);
		TrapezoidalPDF::Pointer priorAnglePDF = TrapezoidalPDF::New();  priorAnglePDF->SetParameters(-PI/2.0,0,PI,3.0*PI/2.0);
		
		IndependentPDFs::Pointer priorEllipsePDF = IndependentPDFs::New();
		priorEllipsePDF->AddMultivariatePDF(priorTransPDF); priorEllipsePDF->AddUnivariatePDF(priorRadiusPDF);
		priorEllipsePDF->AddUnivariatePDF(priorElongPDF);   priorEllipsePDF->AddUnivariatePDF(priorAnglePDF);

		scene->GetObjectTypesLibrary()->SetObjectPDF(typeCode, PDF_OBJECTLIKELIHOOD, priorEllipsePDF);

		if (verbose) std::cout<<"defined the prior pdf for ellipses, test sample: "<<priorEllipsePDF->DrawSample()<<std::endl; 
		if (verbose) std::cout<<"defined the prior pdf for ellipses, test sample: "<<priorEllipsePDF->DrawSample()<<std::endl; 
		if (verbose) std::cout<<"defined the prior pdf for ellipses, test sample: "<<priorEllipsePDF->DrawSample()<<std::endl; 
		if (verbose) std::cout<<"defined the prior pdf for ellipses, test sample: "<<priorEllipsePDF->DrawSample()<<std::endl; 
		if (verbose) std::cout<<"defined the prior pdf for ellipses, test sample: "<<priorEllipsePDF->DrawSample()<<std::endl; 


		GPF_LinearRescaling<double, double>::Pointer priorNormalization = GPF_LinearRescaling<double, double>::New();
		vnl_vector<double> priorNormFctParams(2); priorNormFctParams(1) = -1; priorNormFctParams(0) = - priorNormFctParams(1)*priorEllipsePDF->GetMaximumLogLikelihoodValue();
		priorNormalization->SetParameters(priorNormFctParams);

		scene->SetObjectPriorNormalizationFunction(priorNormalization);

		if (verbose) {
			vnl_vector<double> tmpparams;
			tmpparams = ellipseGenerationPDF->DrawSample();
			std::cout<<"random sample: "<<tmpparams<<", likelihood: "<<priorNormalization->Evaluate( priorEllipsePDF->GetLogLikelihood(tmpparams))<<std::endl;
			tmpparams = ellipseGenerationPDF->DrawSample();
			std::cout<<"random sample: "<<tmpparams<<", likelihood: "<<priorNormalization->Evaluate( priorEllipsePDF->GetLogLikelihood(tmpparams))<<std::endl;
			tmpparams = ellipseGenerationPDF->DrawSample();
			std::cout<<"random sample: "<<tmpparams<<", likelihood: "<<priorNormalization->Evaluate( priorEllipsePDF->GetLogLikelihood(tmpparams))<<std::endl;
			tmpparams = ellipseGenerationPDF->DrawSample();
			std::cout<<"random sample: "<<tmpparams<<", likelihood: "<<priorNormalization->Evaluate( priorEllipsePDF->GetLogLikelihood(tmpparams))<<std::endl;
			tmpparams = ellipseGenerationPDF->DrawSample();
			std::cout<<"random sample: "<<tmpparams<<", likelihood: "<<priorNormalization->Evaluate( priorEllipsePDF->GetLogLikelihood(tmpparams))<<std::endl;
			tmpparams = ellipseGenerationPDF->DrawSample();
			std::cout<<"random sample: "<<tmpparams<<", likelihood: "<<priorNormalization->Evaluate( priorEllipsePDF->GetLogLikelihood(tmpparams))<<std::endl;
		}

		/* 
		* Sensor 
		*/
		typedef SimpleUniformSensor<SceneType, ReferenceImageType> SensorType;
		SensorType::Pointer sensor = SensorType::New();
		sensor->SetScene(scene);
		vnl_vector<double> sensorAppearanceParam(1); sensorAppearanceParam(0) = 200;
		sensor->SetAppearanceParameters(sensorAppearanceParam);
		if (verbose) std::cout<<"defined the sensor"<<std::endl; 
		sensor->GetScene();

		/*
		* Cost Function
		*/
		//Data Cost
		typedef SensedSingleObjectInOutMMahalanobisDistance<SceneType, ReferenceImageType, SensorType::OutputImageType> SingleObjectCostFunctionType;
		//typedef SensedSingleObjectToImageMeanAbsoluteDifferenceCostFunction<SceneType, ReferenceImageType, SensorType::OutputImageType> SingleObjectCostFunctionType;
		if (verbose) std::cout<<"created cost function type"<<std::endl; 

		SingleObjectCostFunctionType::Pointer singleObjectCostFunction = SingleObjectCostFunctionType::New();
		if (verbose) std::cout<<"created cost function object"<<std::endl; 
		singleObjectCostFunction->SetReferenceImage( referenceImage );
		if (verbose) std::cout<<"  reference image is set"<<std::endl; 
		singleObjectCostFunction->SetSensor(sensor);
		if (verbose) std::cout<<"  sensor is set"<<std::endl; 
		singleObjectCostFunction->SelectOffContextComputation();
		singleObjectCostFunction->SetSurroundingRadius(2);
		if (verbose) std::cout<<"set inputs of the cost function"<<std::endl; 
		//normalization of the cost function
		GPF_LinearRescaling<double, double>::Pointer normalizationFunction = GPF_LinearRescaling<double, double>::New();
		vnl_vector<double> normFParams(2); normFParams(0) = -5.5; normFParams(1)=1;
		normFParams(0) = -0.1; //try a more conservative value when the image is noisy.		
		//vnl_vector<double> normFParams(2); normFParams(0) = -0.4; normFParams(1)=2; //actually, the MAD metric would need a more discriminative rescaling...
		normalizationFunction->SetParameters(normFParams);
		singleObjectCostFunction->SetNormalizationFunction(normalizationFunction);
		if (verbose) std::cout<<"set normalization function"<<std::endl; 

		//Interaction Cost
		typedef TransformedPairWiseOverlapMeasure<GPF_ScaledExpPlusConstantFunction<double, double>, PairWiseOverlapCoefficient> IntersectionCostFunctionType;
		IntersectionCostFunctionType::Pointer intersectionCostFunction = IntersectionCostFunctionType::New();
		vnl_vector<double> intersectionCostFunctionParams(2); intersectionCostFunctionParams(0) = -1; intersectionCostFunctionParams(1) = 100;
		intersectionCostFunction->SetParameters(intersectionCostFunctionParams);
		scene->GetInteractionManager()->SetIntersectionCostFunction(intersectionCostFunction);
		if (verbose) std::cout<<"set the interaction cost function"<<std::endl; 

		//Energy
		typedef SceneEnergy<SceneType, ReferenceImageType> EnergyType;
		EnergyType::Pointer sceneEnergy = EnergyType::New();
		sceneEnergy->SetScene(scene);
		if (verbose) std::cout<<"instanciated energy"<<std::endl; 

		sceneEnergy->UseGlobalScenePrior(false);
		sceneEnergy->UseObjectPrior(true);
		sceneEnergy->UseObjectInteractionPrior(true); sceneEnergy->SetInteractionPriorWeight(1);
		sceneEnergy->SetSingleObjectSceneToImageCostFunction(singleObjectCostFunction, 1);
		sceneEnergy->UseSceneToImageCostFunction(false);

		if (verbose) std::cout<<"defined the energy"<<std::endl; 

		/*
		* sanity check... testing the computation of the metric...
		*/
		if (verbose) {
			std::cout<<"computing the energy on the empty scene..."<<std::endl;
			std::cout<<"  value = "<<sceneEnergy->GetValue()<<std::endl;

			std::cout<<"generating a random ellipse from the pdf..."<<std::endl;
			SceneType::DeformableObjectType::Pointer sampleObject = scene->GetObjectTypesLibrary()->GenerateNewRandomObject(typeCode);
			std::cout<<"ok"<<std::endl;
			std::cout<<"  parameters = "<<sampleObject->GetParameters()<<std::endl;
			std::cout<<"ok!, now adding it to the scene..."<<std::endl;
			scene->AddObject(sampleObject);
			std::cout<<"  VTK RESOLUTION: "<<scene->GetObject(1)->obj->GetObjectAsVTKPolyData()->GetNumberOfPoints()<<std::endl;
		
			std::cout<<"computing the prior cost for this object..."<<std::endl;
			std::cout<<"value = "<<scene->GetObjectPriorCost(1)<<std::endl;

			std::cout<<"adding a second ellipse..."<<std::endl;
			sampleObject = scene->GetObjectTypesLibrary()->GenerateNewRandomObject(typeCode);
			scene->AddObject(sampleObject);

			std::cout<<"ok!!! recomputing the energy---"<<std::endl;
			std::cout<<"  value = "<<sceneEnergy->GetValue()<<std::endl;
			std::cout<<"writing the scene image... "<<std::endl;
			SceneType::LabelImageType *tmpLabelImage = scene->GetSceneAsLabelImage();
			Write2DGreyLevelRescaledImageToFile<SceneType::LabelImageType>("labelImage.png", tmpLabelImage);

			std::cout<<"return to initial state"<<std::endl;
			scene->RemoveAllObjects();
			std::cout<<"ok, number of objects in scene: "<<scene->GetNumberOfObjects()<<std::endl;
			std::cout<<"\ndone with the sanity checks ; continuing with object detection and scene optimization"<<std::endl;
		}

		/*
		* alright. now proceed with the optimization...
		* in this test program, use only birth and death kernels.
		*/

		SceneStateOptimizationManager<SceneType>::Pointer sceneStateOptimizationManager = SceneStateOptimizationManager<SceneType>::New();
		sceneStateOptimizationManager->SetScene(scene);
		sceneStateOptimizationManager->SetCostFunction(sceneEnergy);

		RandomJumpSAOptimizer<SceneType>::Pointer rjsaOptimizer = RandomJumpSAOptimizer<SceneType>::New();
		//rjsaOptimizer->SetScene(scene);
		ScalarFunction_Geometric::Pointer coolingFunction = ScalarFunction_Geometric::New();
		coolingFunction->SetFactor(0.995); //0.999 //0.9995
		rjsaOptimizer->SetCoolingFunction(coolingFunction);
		rjsaOptimizer->SetInitialTemperature(1);
		rjsaOptimizer->SetMaximumNumberOfAttemptsAtT(2000); //2000 //5000
		rjsaOptimizer->SetMaximumNumberOfConsecutiveFailures(500); //8000 //500
		rjsaOptimizer->SetMaximumNumberOfConsecutiveSuccessesAtT(100); //

		BirthKernel<SceneType>::Pointer birthKernel = BirthKernel<SceneType>::New(); birthKernel->SetScene(scene); rjsaOptimizer->AddKernel(birthKernel, 1);
		DeathKernel<SceneType>::Pointer deathKernel = DeathKernel<SceneType>::New(); deathKernel->SetScene(scene); rjsaOptimizer->AddKernel(deathKernel, 1);
		RandomWalkOneRandomObjectKernel<SceneType>::Pointer rwKer = RandomWalkOneRandomObjectKernel<SceneType>::New(); rwKer->SetScene(scene); rjsaOptimizer->AddKernel(rwKer, 100);

		sceneStateOptimizationManager->SetOptimizer(rjsaOptimizer);
		std::cout<<"configured the optimizer and optimization manager..., now optimize!"<<std::endl;
		std::cout<<"initial configuration: "<<scene->GetNumberOfObjects()<<", energy: "<<sceneStateOptimizationManager->GetValue()<<std::endl;
		clock_t t0=clock();
		sceneStateOptimizationManager->Optimize();
		std::cout<<"final configuration: "<<scene->GetNumberOfObjects()<<", energy: "<<sceneStateOptimizationManager->GetValue()<<std::endl;
		std::cout<<"optimization time: "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s."<<std::endl;

		std::cout<<"total number of objects added: "<<scene->m_addCounter<<std::endl;
		std::cout<<"total number of objects removed: "<<scene->m_remCounter<<std::endl;
		std::cout<<"total number of objects modified: "<<scene->m_modCounter<<std::endl;

		std::cout<<"Final number of objects: "<<scene->GetNumberOfObjects()<<std::endl;

		SceneObjectIterator<SceneType> objectIt(scene);
		objectIt.GoToBegin();
		for (objectIt.GoToBegin(); !objectIt.IsAtEnd() ; ++objectIt) {
			std::cout<<"\nlabel "<<objectIt.GetID()<<", params: "<<scene->GetParametersOfObject(objectIt.GetID())<<", energy = "<<sceneEnergy->GetEnergyForObject(objectIt.GetID())<<std::endl;
			singleObjectCostFunction->SelectObject(objectIt.GetID());singleObjectCostFunction->PrintDetails();
			std::cout<<"  prior cost: "<<scene->GetObjectPriorCost(objectIt.GetID())<<std::endl;
			std::cout<<"  interaction cost: "<<scene->GetObjectTotalInteractionCost(objectIt.GetID())<<std::endl;
		}

		WriteITKImageToFile<SceneType::LabelImageType>("FinalLabelImage.png", scene->GetSceneAsLabelImage());
		typedef itk::BinaryThresholdImageFilter <SceneType::LabelImageType, ReferenceImageType> BinaryThresholdImageFilterType;
		BinaryThresholdImageFilterType::Pointer thresholdFilter = BinaryThresholdImageFilterType::New();
		thresholdFilter->SetInput(scene->GetSceneAsLabelImage());
		thresholdFilter->SetLowerThreshold(1); thresholdFilter->SetInsideValue(200); thresholdFilter->SetOutsideValue(0);
		thresholdFilter->Update();
		Write2DGreyLevelRescaledImageToFile<ReferenceImageType>("FinalBinaryImage.png", thresholdFilter->GetOutput());

	}
	catch (DeformableModelException& e) {
		std::cout << "Exception occured while performing the program..." << std::endl;
		std::cout << e.what() << std::endl;
	}


	return 1;
}

