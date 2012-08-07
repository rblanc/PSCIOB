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

/**
* \file Example_Ellipses2D_Synthese.cxx
* \author Rémi Blanc 
* \date 25. May 2012
* 
*/

#include "Example_Ellipses2D_Synthese.h"

using namespace psciob;

/** Constructor */
Example_Ellipses2D_Synthese::Example_Ellipses2D_Synthese() {
	m_scene = NULL;
	m_image = ImageType::New();	m_imageFlag = false;
	m_blurStd = 0;
	m_noiseStd = 0;
	m_seed = 0;
	
	m_thresholdFilter = BinaryThresholdImageFilterType::New();
	m_thresholdFilter->SetLowerThreshold(1); m_thresholdFilter->SetInsideValue(200); m_thresholdFilter->SetOutsideValue(0);
	
	m_blurFilter = BlurFilterType::New();
	m_noiseImageGenerator = NoiseImageSourceType::New(); m_noiseGenerator = NormalPDF::New();
	m_addFilter = AddImageFilterType::New();

	m_selection = 0;
}


/** Get Pointer to the scene 
* Generates the 1st scene example if necessary
*/
Example_Ellipses2D_Synthese::SceneType* 
Example_Ellipses2D_Synthese::GetScenePointer() {
	if (!m_scene) {	GenerateScene(); }
	return m_scene.GetPointer();
}

/** Get a pointer to the image of the scene 
* updates the image if necessary (e.g. if the noise properties have been modified)
*/
Example_Ellipses2D_Synthese::ImageType* 
Example_Ellipses2D_Synthese::GetImage() {
	if (!m_imageFlag) {
		SceneType::LabelImageType *labelImage = GetScenePointer()->GetSceneAsLabelImage();
		//binarize the image
		m_thresholdFilter->SetInput(labelImage);
		//if necessary: blur it
		if (m_blurStd>0) { //blur the image
			m_blurFilter->SetInput( m_thresholdFilter->GetOutput() );
			m_blurFilter->SetSigma(m_blurStd);
			if (m_noiseStd>0) { //if necessary: add noise
				m_noiseGenerator->SetParameters(0, m_blurStd);
				m_noiseImageGenerator->SetRandomNumberGenerator( m_noiseGenerator );
				
				m_addFilter->SetInput1( m_blurFilter->GetOutput() );
				m_addFilter->SetInput2( m_noiseImageGenerator->GetOutput() );
				
				//output image
				m_addFilter->Update();
				m_image = m_addFilter->GetOutput();
			}
			else {
				//output image = blurred image
				m_blurFilter->Update();
				m_image = m_blurFilter->GetOutput();
			}
		}
		else {
			if (m_noiseStd>0) { //if necessary: add noise
				m_noiseGenerator->SetParameters(0, m_blurStd);
				m_noiseImageGenerator->SetRandomNumberGenerator( m_noiseGenerator );
				
				m_addFilter->SetInput1( m_thresholdFilter->GetOutput() );
				m_addFilter->SetInput2( m_noiseImageGenerator->GetOutput() );
				
				//output image
				m_addFilter->Update();
				m_image = m_addFilter->GetOutput();
			}
			else {
				//output = thresholded scene image.
				m_thresholdFilter->Update();
				m_image = m_thresholdFilter->GetOutput();
			}
		}
		m_imageFlag = true;
	}
	return m_image.GetPointer();
}

/** Set the seed for the noise generator - default seed is 0 */
void Example_Ellipses2D_Synthese::SetNoiseSeed(int seed) {
	m_seed = seed;
	m_imageFlag = false;
}

/** Set the standard deviation of the gaussian noise which is added to the binary image 
* This invalidates the output image
*/
void Example_Ellipses2D_Synthese::SetNoiseStd(double noise) {
	m_noiseStd = max(0.0, noise);
	m_imageFlag = false;
}

/** Get the standard deviation of the gaussian noise which is added to the binary image */
double Example_Ellipses2D_Synthese::GetNoiseStd() {
	return m_noiseStd;
}

/** Set the standard deviation of the gaussian noise which is added to the binary image 
* This invalidates the output image
*/
void Example_Ellipses2D_Synthese::SetBlurStd(double blur) {
	m_blurStd = max(0.0, blur);
	m_imageFlag = false;
}

/** Get the standard deviation of the gaussian noise which is added to the binary image */
double Example_Ellipses2D_Synthese::GetBlurStd() {
	return m_blurStd;
}

//
void Example_Ellipses2D_Synthese::GenerateScene() {
	switch (m_selection) {
		case 0: GenerateScene_0(); break;
		case 1: GenerateScene_1(); break;
	}
}


void Example_Ellipses2D_Synthese::GenerateScene_0() {
	m_scene = SceneType::New();
	vnl_vector<double> sceneBBox(4); 
	sceneBBox(0) = 0; sceneBBox(1) = 100;
	sceneBBox(2) = 0; sceneBBox(3) = 100;

	m_scene->SetPhysicalDimensions(sceneBBox);

	ObjectType::Pointer object = ObjectType::New();
	vnl_vector<double> params(5);

	params(0) = 11; params(1) = 12; params(2) =8.5; params(3) = 0.95; params(4) =4.0*PI/5.0; object->SetParameters(params); m_scene->AddObject(object);
	params(0) = 26; params(1) = 5 ; params(2) = 5 ; params(3) = 0.65; params(4) = PI/2.0;    object->SetParameters(params); m_scene->AddObject(object);
	params(0) = 39; params(1) = 14; params(2) = 6 ; params(3) = 0.5 ; params(4) = PI/3.5;    object->SetParameters(params); m_scene->AddObject(object);

	params(0) = 10; params(1) = 35; params(2) = 4 ; params(3) = 0.7 ; params(4) = PI/2.0;    object->SetParameters(params); m_scene->AddObject(object);
	params(0) = 18; params(1) = 35; params(2) = 4 ; params(3) = 0.7 ; params(4) = PI/2.0;    object->SetParameters(params); m_scene->AddObject(object);

	params(0) = 28; params(1) = 30; params(2) = 3 ; params(3) = 0.75; params(4) =2.0*PI/3.0; object->SetParameters(params); m_scene->AddObject(object);
	params(0) = 30; params(1) = 36; params(2) = 3 ; params(3) = 0.75; params(4) = PI/2.0;    object->SetParameters(params); m_scene->AddObject(object);

	params(0) = 42; params(1) = 40; params(2) = 6 ; params(3) = 0.8 ; params(4) =3.0*PI/5.0; object->SetParameters(params); m_scene->AddObject(object);

	//all examples should end with this code: update the flags, and make sure the memory is allocated for the output image.
	SceneType::LabelImageType *labelImage = GetScenePointer()->GetSceneAsLabelImage();
	m_image->SetRegions(labelImage->GetLargestPossibleRegion());
	m_image->SetOrigin(labelImage->GetOrigin());
	m_image->SetSpacing(labelImage->GetSpacing());
	m_image->Allocate();
	m_imageFlag = false;
}


/** 100*100 2D Scene, 50 random ellipses obtained using a FB algorithm (without rotations)
*/
#include "ForceBiasedAlgorithm.h"

void Example_Ellipses2D_Synthese::GenerateScene_1() {
	m_scene = SceneType::New();
	vnl_vector<double> sceneBBox(4); 
	sceneBBox(0) = 0; sceneBBox(1) = 100;
	sceneBBox(2) = 0; sceneBBox(3) = 100;

	m_scene->SetPhysicalDimensions(sceneBBox);

	ObjectType::Pointer sampleEllipse = ObjectType::New();
	unsigned int typeCode = m_scene->GetObjectTypesLibrary()->RegisterObjectType(sampleEllipse);

	ObjectType::Pointer object = ObjectType::New();
	psciob::UniformBoxPDF::Pointer uniformTransPDF = psciob::UniformBoxPDF::New();
	uniformTransPDF->SetBox(sceneBBox);
	psciob::UniformPDF::Pointer    uniformRadiusPDF= psciob::UniformPDF::New();
	uniformRadiusPDF->SetParameters(3, 10);
	psciob::UniformPDF::Pointer    uniformElongPDF = psciob::UniformPDF::New();
	uniformElongPDF->SetParameters(0.25, 1);
	psciob::UniformPDF::Pointer    uniformAnglePDF = psciob::UniformPDF::New();
	uniformAnglePDF->SetParameters(0, PI);

	psciob::IndependentPDFs::Pointer ellipseGenerationPDF = psciob::IndependentPDFs::New();
	ellipseGenerationPDF->AddMultivariatePDF(uniformTransPDF); ellipseGenerationPDF->AddUnivariatePDF(uniformRadiusPDF);
	ellipseGenerationPDF->AddUnivariatePDF(uniformElongPDF);   ellipseGenerationPDF->AddUnivariatePDF(uniformAnglePDF);

	m_scene->GetObjectTypesLibrary()->SetObjectPDF(typeCode, psciob::PDF_OBJECTGENERATIONPRIOR, ellipseGenerationPDF);

	//50 random ellipses
	for (unsigned i=0 ; i<50 ; i++) {
		m_scene->AddObject(m_scene->GetObjectTypesLibrary()->GenerateNewRandomObject(typeCode));
	}

	//all examples should end with this code: update the flags, and make sure the memory is allocated for the output image.
	SceneType::LabelImageType *labelImage = GetScenePointer()->GetSceneAsLabelImage();
	m_image->SetRegions(labelImage->GetLargestPossibleRegion());
	m_image->SetOrigin(labelImage->GetOrigin());
	m_image->SetSpacing(labelImage->GetSpacing());
	m_image->Allocate();
	m_imageFlag = false;


	psciob::ForceBiasedAlgorithm<SceneType>::Pointer fbAlgo = psciob::ForceBiasedAlgorithm<SceneType>::New();
	fbAlgo->SetScene(m_scene);
	fbAlgo->GetMovementManager()->SetScalingFactor(0.98);
	std::cout<<"Apply FB algo until convergence..."<<std::endl;
	fbAlgo->IterateUntilConvergence();

}
