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
* \file SimplePixelBasedSEMSensor.h
* \author Rémi Blanc 
* \date 28. September 2011
*/

#ifndef SIMPLEPIXELBASEDSEMSENSOR_H_
#define SIMPLEPIXELBASEDSEMSENSOR_H_

#include "ImageSensor_Base.h"
#include <itkLabelMapToLabelImageFilter.h>
#include <itkLabelImageToLabelMapFilter.h>


//IDEA: should this class maintain a full labelImage at all times?
//it would make a slight overhead, recomputing the LabelMap out of it
//but allow to update only small regions of the sensor instead of the full image all the time...
//elements for this second option are written & commented after the Update_data method

namespace psciob {
/**
* \class SimplePixelBasedSEMSensor
* \brief SimplePixelBasedSEMSensor
* from a 3D image, generates a 2D image => parallel / projective view, etc...
* The field of view is always set to observe the whole scene
*	
* the sensor orientation is either HORIZONTAL (default), SAGITTAL or CORONAL (1st pose param)
* this is a simplified sensor, with a direction always poiting toward the positive direction 
* and with a 'position' stuck at the lowest scene bbox value in the direction (there can be spurious effects for objects 'in front of' the sensor)
*
* the default appearance function is decreasing exponentially (ExpDecrFunction) with initial parameter value 250
*
* \todo add a specific resolution, possibilities for noise, blurriness, etc...
* \todo make another class that properly manages sensor position...
*/

//CONCRETE CLASS
template<class TScene, class TOutputImage = itk::Image<float, 2>>
class SimplePixelBasedSEMSensor : public ImageSensor_Base<TScene, TOutputImage> { 
protected:
	static const unsigned int m_nbPoseParams = 1;	//code for the direction (horizontal/sagittal/coronal)
	static const unsigned int m_nbDistortionParams  = 0;	//no distortion
	static const unsigned int m_nbNoiseParams		= 0;	//no noise
	static const unsigned int m_nbResolutionParams  = 0;	//no resolution params ~> best would be to set the requested resolution, and an interpolator rather than a set of parameters...
public:
	/** Standard class typedefs. */
	typedef SimplePixelBasedSEMSensor      Self;
	typedef ImageSensor_Base               Superclass;
	typedef itk::SmartPointer<Self>        Pointer;
	typedef itk::SmartPointer<const Self>  ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SimplePixelBasedSEMSensor, ImageSensor_Base);
	itkNewMacro(Self);


	/** One orientation parameters */
	unsigned int GetNumberOfPoseParameters() { return m_nbPoseParams; } 
	/** No distortion parameters */
	unsigned int GetNumberOfDistortionParameters()	{ return m_nbDistortionParams; }
	/** No noise parameters */
	unsigned int GetNumberOfNoiseParameters()		{ return m_nbNoiseParams; }
	/** No resolution parameters */
	unsigned int GetNumberOfResolutionParameters()	{ return m_nbResolutionParams; }

	/** Set the direction: Generic interface */ 
	bool SetPoseParameters(const vnl_vector<double> &p);

	/** Set the direction, specific to this class : HORIZONTAL / SAGITTAL / CORONAL */ 
	void SetObservationDirectionType(ObservationDirectionType dir);
	ObservationDirectionType GetObservationDirectionType()			{return m_direction;}	


	/** bounding box of the sensed scene */
	vnl_vector<double> GetSensedBoundingBox(const vnl_vector<double> &bbox);


	/** Off Context image of an object: as if it was alone in the scene 
	* this labelMap is expressed with respect to the full sensor grid
	* returns false if the object is not in the scene
	*/
	bool GetOffContextObjectLabelMap(ObjectInScene *objectPtr, OutputLabelMapType *offContextObjectLabelMap);

	/** Off Context image of an object: as if it was alone in the scene 
	* The image and label maps are expressed relative to the smallest possible region of the sensor frame (to save memory and computation time...)
	* 	OPTIMIZATION: different possibilities for optimization
	*  - caching the images / labelMap for each object, exploiting particular DataContainer for the scene?
	* returns false if the object is not in the scene
	*/
	bool GetOffContextObjectImage(ObjectInScene *objectPtr, OutputImageType* image, OutputLabelMapType *labelMap);

	/** Same as before, but only compute the output image, not the label map. 
	* is it faster?
	* if necessary, I could avoid looping over the whole image again for the CORONAL & HORIZONTAL VIEWS, and directly compute the value...
	*/
	bool GetOffContextObjectImage(ObjectInScene *objectPtr, OutputImageType* image);
	

	/** Get 3D Index for horizontal direction of the output */
	unsigned GetOutputHorizontalDimensionIndex() {return m_dirAxis0;}
	/** Get 3D Index for vertical direction of the output */
	unsigned GetOutputVerticalDimensionIndex() {return m_dirAxis1;}
	/** Get 3D Index for in-depth direction of the output */
	unsigned GetOutputDepthDimensionIndex() {return m_dirAxis2;}

protected:
	SimplePixelBasedSEMSensor();
	~SimplePixelBasedSEMSensor() {};	

	typedef itk::LabelImageToLabelMapFilter< OutputLabelImageType, OutputLabelMapType> LabelImageToLabelMapFilterType;
	
	ObservationDirectionType m_direction;
	unsigned int m_dirAxis0, m_dirAxis1, m_dirAxis2; //maps the sensor image axes with the original scene axes ; see SetObservationDirectionType


	typename OutputLabelImageType::Pointer	m_outputLabelImage;//, m_outputSingleObjectLabelMap;

	/** forces the output image to have the same resolution, dimension, etc... as the scene */
	void AllocateOutputImage();


	/** Update data in the requested region (CLEAN: use itk -> RequestedRegion instead?) */
	void Update_Data(typename InputLabelImageType::RegionType inputRegion);


private:
	SimplePixelBasedSEMSensor(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};

} // namespace psciob

#include "SimplePixelBasedSEMSensor.txx"

#endif /* SIMPLEPIXELBASEDSEMSENSOR_H_ */
