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

		//test the feature detector (canny edges + orientation)
		typedef ImageFeaturePointDetector_2DCannyEdges_Orientation<unsigned char> FeatureDetectorType;
		FeatureDetectorType::Pointer featureDetector = FeatureDetectorType::New();

		featureDetector->SetInputImage( sensor->GetOutputImage() );
		featureDetector->SetCannyVariance( 2.0*2.0*sceneSpacing(0)*sceneSpacing(1) );
		featureDetector->Update();

		Write2DGreyLevelRescaledImageToFile<FeatureDetectorType::FloatImageType>("featurePointsImage.png", featureDetector->GetFeaturePointImage());
		Write2DGreyLevelRescaledImageToFile<FeatureDetectorType::FloatImageType>("gradientOrientationImage.png", featureDetector->GetOrientationImage());
		
		//train a simple hough transform using the same / similar disk radius

		//detect the disks on the input image...
	}
	catch (DeformableModelException& e) {
		std::cout << "Exception occured while running the test cases" << std::endl;
		std::cout << e.what() << std::endl;
	}

}
