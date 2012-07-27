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
* \file Example_Ellipses2D_Synthese.h
* \author Rémi Blanc 
* \date 25. May 2012
* 
*/

#ifndef EXAMPLE_ELLIPSES2D_SYNTHESE_H_
#define EXAMPLE_ELLIPSES2D_SYNTHESE_H_


#include "GeneralUtils.h"
#include "LabelImageScene.h"
#include "Fast2DEllipse.h"

#include <itkBinaryThresholdImageFilter.h>
#include <itkRecursiveGaussianImageFilter.h>
#include "WhiteNoiseImageSource.h"
#include "UnivariatePDF.h"
#include <itkAddImageFilter.h>


/** \class Example_Ellipses2D_Synthese
* \brief Example_Ellipses2D_Synthese: helper class to generate a scene of ellipses
* as well as a corresponding image, possibly with some noise...
* 
* The base dynamics of the output image is background = 0 / foreground = 200
*/
class Example_Ellipses2D_Synthese {
public:
	static const unsigned int Dimension = 2;
	typedef double            PixelType;
	typedef unsigned char     IDType;
	typedef psciob::Fast2DEllipse                               ObjectType;
	typedef psciob::LabelImageScene<Dimension, IDType, IDType>  SceneType;
	typedef itk::Image<PixelType, Dimension>            ImageType;
	typedef itk::Image<double, Dimension>               NoiseImageType;

	/** Constructor */
	Example_Ellipses2D_Synthese();

	/** Get Pointer to the scene 
	* Generates the 1st scene example if necessary
	*/
	SceneType* GetScenePointer();

	/** Get a pointer to the image of the scene 
	* updates the image if necessary (e.g. if the noise properties have been modified)
	*/
	ImageType* GetImage();

	/** Set the seed for the noise generator - default seed is 0 */
	void SetNoiseSeed(int seed);

	/** Set the standard deviation of the gaussian noise which is added to the binary image 
	* This invalidates the output image
	*/
	void SetNoiseStd(double noise);

	/** Get the standard deviation of the gaussian noise which is added to the binary image */
	double GetNoiseStd();

	/** Set the standard deviation of the gaussian noise which is added to the binary image 
	* This invalidates the output image
	*/
	void SetBlurStd(double blur);

	/** Get the standard deviation of the gaussian noise which is added to the binary image */
	double GetBlurStd();

	/** */

	/** Generate 1st scene example */
	void GenerateScene_1();


private:

	typedef itk::BinaryThresholdImageFilter <SceneType::LabelImageType, ImageType> BinaryThresholdImageFilterType;
	typedef itk::RecursiveGaussianImageFilter<ImageType, ImageType> BlurFilterType;
	typedef psciob::WhiteNoiseImageSource<NoiseImageType> NoiseImageSourceType;
	typedef itk::AddImageFilter<ImageType, NoiseImageType, ImageType> AddImageFilterType;
	
	BinaryThresholdImageFilterType::Pointer m_thresholdFilter;
	BlurFilterType::Pointer                 m_blurFilter;
	NoiseImageSourceType::Pointer           m_noiseImageGenerator;
	AddImageFilterType::Pointer             m_addFilter;			  
	psciob::NormalPDF::Pointer              m_noiseGenerator;
	
	SceneType::Pointer m_scene; //scene from which the image is generated.
	ImageType::Pointer m_image; //output image, a binary version of the labelimage from the scene, plus some noise / blur if requested.
	bool m_imageFlag;			//flag indicating whether the output image is uptodate
	double m_blurStd, m_noiseStd;
	int m_seed;

};

#endif /* EXAMPLE_ELLIPSES2D_SYNTHESE_H_ */
