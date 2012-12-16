/*
 * This file is part of the psciob library.
 *
 * Copyright (c) 2012, Remi Blanc
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

#include "VTKScene.h"
#include "shape2DDisk.h"
#include "TranslationScale2DTransform.h"
#include "PoseTransformedBinaryShape.h"
#include "SimpleUniformSensor.h"

#include "ImageFeaturePointDetector_2DCannyEdges_Orientation.h"
#include "GeneralizedHoughTransformAlgorithm_Test.h"

using namespace psciob;

int main(int argc, char** argv) {

	try {
		//generate a scene with a few disks - same radius
		typedef VTKScene<2> SceneType;

		shape2DDisk::Pointer diskShape = shape2DDisk::New();
		TranslationScale2DTransform::Pointer poseTransform = TranslationScale2DTransform::New();
		PoseTransformedBinaryShape<2>::Pointer disk = PoseTransformedBinaryShape<2>::New();
		disk->SetShapeAndTransform(diskShape, poseTransform);
	
		SceneType::Pointer scene = SceneType::New();
		//define the characteristics of the scene
		vnl_vector<double> sceneSpacing(2);	sceneSpacing.fill(0.1);
		vnl_vector<double> sceneBBox(4);	for (unsigned i=0 ; i<2 ; i++) { sceneBBox(2*i) = -10;	sceneBBox(2*i+1) = 10; }
		scene->SetPhysicalDimensions(sceneBBox, sceneSpacing);

		//add a few disks to the scene, some overlaps, some partially outside, all radii close to 2
		vnl_vector<double> diskParams(3); //center(*2), scale (<=>disk diameter)
		diskParams(0) = 0;   diskParams(1) = 0;   diskParams(2) =2*2;    disk->SetParameters(diskParams);	scene->AddObject(disk);
		diskParams(0) = 1;   diskParams(1) = 1;   diskParams(2) =2*1.8;  disk->SetParameters(diskParams);	scene->AddObject(disk);
		diskParams(0) =-4.2; diskParams(1) = 2.3; diskParams(2) =2*2;    disk->SetParameters(diskParams);	scene->AddObject(disk);
		diskParams(0) =+4.2; diskParams(1) = -4;  diskParams(2) =2*2.1;  disk->SetParameters(diskParams);	scene->AddObject(disk);
		diskParams(0) =+7.65;diskParams(1) =4.142;diskParams(2) =2*1.9;  disk->SetParameters(diskParams);	scene->AddObject(disk);
		diskParams(0) =-2.13;diskParams(1) =-9.34;diskParams(2) =2*2;    disk->SetParameters(diskParams);	scene->AddObject(disk);
		diskParams(0) =-9.13;diskParams(1) =-6.34;diskParams(2) =2*2.02; disk->SetParameters(diskParams);	scene->AddObject(disk);
		diskParams(0) =-8.91;diskParams(1) =+9.24;diskParams(2) =2*1.97; disk->SetParameters(diskParams);	scene->AddObject(disk);

		//use a sensor to make an image out of it -> image on which I'll then try to detect the disks using HT...
		//ideally, the sensor should be able to add some noise to the image... => TODO
		//to do that, just add the noise after the image is generated (overloading GetOutputImage... or perhaps the internal Update...)
		typedef itk::Image<unsigned char, 2> ImageType;
		typedef SimpleUniformSensor<SceneType, ImageType> SensorType;
		SensorType::Pointer sensor = SensorType::New();
		sensor->SetScene(scene);
		vnl_vector<double> appParam(1); appParam(0) = 255; sensor->SetAppearanceParameters( appParam );
			
		Write2DGreyLevelRescaledImageToFile<ImageType>("referenceDiskImage.png", sensor->GetOutputImage());
		//I could also clone the image... but anyway... this is just a test program...
		ImageType::Pointer referenceImage = ReadITKImageFromFile<ImageType>("referenceDiskImage.png", 0.1);
		referenceImage->SetOrigin( sensor->GetOutputImage()->GetOrigin() );

		//test the feature detector (canny edges + orientation)
		typedef ImageFeaturePointDetector_2DCannyEdges_Orientation<unsigned char> FeatureDetectorType;
		FeatureDetectorType::Pointer featureDetector = FeatureDetectorType::New();

		featureDetector->SetInputImage( referenceImage );
		featureDetector->SetCannyVariance( 2.0*2.0*sceneSpacing(0)*sceneSpacing(1) );
		featureDetector->Update();

		std::cout<<"number of feature points detected: "<<featureDetector->GetFeaturePointList().size()<<std::endl;
		Write2DGreyLevelRescaledImageToFile<FeatureDetectorType::FloatImageType>("featurePointsImage.png", featureDetector->GetFeaturePointImage());
		Write2DGreyLevelRescaledImageToFile<FeatureDetectorType::FloatImageType>("gradientOrientationImage.png", featureDetector->GetOrientationImage());
		
		//train a simple hough transform using the same / similar disk radius
		typedef GeneralizedHoughTransformAlgorithm_Test<unsigned char, 2> GHT_AlgoType;
		GHT_AlgoType::Pointer ghtAlgo = GHT_AlgoType::New();
		ghtAlgo->SetInputImage( referenceImage );
		ghtAlgo->SetFeaturePointDetector( featureDetector );
		ghtAlgo->SetSensor( sensor );
		
		//generating a grid of parameters to be considered for detection, and informing the HT about it
		diskParams(2) = 4; disk->SetParameters(diskParams);
		std::vector< std::vector<double> > gridParams; std::vector<double> tmpVect;
		tmpVect.push_back(0); gridParams.push_back(tmpVect); //translation x
		gridParams.push_back(tmpVect); //translation y
		tmpVect.clear(); tmpVect.push_back(3.5); tmpVect.push_back(4); tmpVect.push_back(4.5); gridParams.push_back(tmpVect); //disk diameters
		ghtAlgo->SetSampleObjectAndParameterGrid(disk, gridParams);

		//define bins in the feature space, to reduce the size of the training table
		std::vector< std::vector<double> > featureBins;
		tmpVect.clear(); for (double theta=-PI ; theta<=PI ; theta+=2.0*PI/90) tmpVect.push_back(theta);
		tmpVect.push_back(PI+PI/10.0);//add a high value, just to prevent some size issues...
		tmpVect.push_back(2.0*PI);//add a high value, just to prevent some size issues...
		featureBins.push_back(tmpVect);
		ghtAlgo->SetFeatureBins(featureBins);

		//DO THE TRAINING
		ghtAlgo->Train(true);

		ghtAlgo->PrintFVStructure(std::cout );
		std::cout<<"NOW DETECTING>>>"<<std::endl;
		//detect the disks on the input image...
		ghtAlgo->DetectObjectsOnImage(true);
		ghtAlgo->PrintVotes(std::cout);

		//TODO NOW: GeneralizedHoughTransformAlgorithm_Test 
		// -> cluster votes according to a spatial criterion
		// -> detect objects. (threshold...)

	}
	catch (DeformableModelException& e) {
		std::cout << "Exception occured while running the test cases" << std::endl;
		std::cout << e.what() << std::endl;
	}

}
