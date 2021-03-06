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
* \file SimpleUniformSensor.h
* \author R�mi Blanc 
* \date 26. October 2011
*/

#ifndef __SIMPLEUNIFORMSENSOR_H_
#define __SIMPLEUNIFORMSENSOR_H_

#include "ImageSensor_Base.h"
#include "itkLabelMapToBinaryImageFilter.h"


namespace psciob {

/**
 * \class SimpleUniformSensor
 * \brief SimpleUniformSensor
 * the output is a binary image, same size & resolution as the scene label image...
 * OPTIMIZATION: another possibility would be to directly ask the scene to generate a binary image - exploiting the set of known occupied pixels for each object, this could be more efficient ; this class would then just set the right pointers.
*/


//CONCRETE CLASS
template<class TScene, class TOutputImage = typename TScene::BinaryImageType>
class SimpleUniformSensor : public ImageSensor_Base<TScene, TOutputImage> {
protected:
	static const unsigned int m_nbPoseParams = 0;	//no orientation
	static const unsigned int m_nbDistortionParams  = 0;	//no distortion
	static const unsigned int m_nbNoiseParams		= 0;	//no noise
	static const unsigned int m_nbResolutionParams  = 0;	//no resolution params
public:
	/** Standard class typedefs. */
	typedef SimpleUniformSensor           Self;
	typedef ImageSensor_Base              Superclass;
	typedef itk::SmartPointer<Self>	      Pointer;
	typedef itk::SmartPointer<const Self> ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SimpleUniformSensor, ImageSensor_Base);
	itkNewMacro(SimpleUniformSensor);

	/** Filter used to generate the sensor uniform output */
	typedef itk::LabelMapToBinaryImageFilter< InputLabelMapType, OutputImageType> LabelMapToBinaryImageFilterType;


	/** No orientation parameters */
	unsigned int GetNumberOfPoseParameters() { return m_nbPoseParams; } 
	/** No distortion parameters */
	unsigned int GetNumberOfDistortionParameters()	{ return m_nbDistortionParams; }
	/** No noise parameters */
	unsigned int GetNumberOfNoiseParameters()		{ return m_nbNoiseParams; }
	/** No resolution parameters */
	unsigned int GetNumberOfResolutionParameters()	{ return m_nbResolutionParams; }

	/** output bounding box equals input bounding box*/
	inline vnl_vector<double> GetSensedBoundingBox(const vnl_vector<double> &bbox) { return bbox; }

	/** Off Context image of an object: as if it was alone in the scene 
	* this labelMap is expressed with respect to the full sensor grid
	* returns false if the object is not in the scene
	*/
	bool GetOffContextObjectLabelMap(ObjectInScene *objectPtr, OutputLabelMapType *offContextObjectLabelMap) {
		//if (!m_scene) throw DeformableModelException("SimpleUniformSensor::GetOffContextObjectLabelMap the scene must be set first!");
		if (objectPtr->id == 0) return false;

		offContextObjectLabelMap->ClearLabels();
		offContextObjectLabelMap->SetSpacing(m_scene->GetSceneSpacing());
		offContextObjectLabelMap->SetOrigin(m_scene->GetSceneOrigin());
		offContextObjectLabelMap->SetRegions(m_scene->GetSceneImageRegion());
		
		if (!objectPtr->sceneOffContextLabelObjectFlag) {
			InsertSingleObjectLabelMapIntoAnother<ObjectLabelMapType, OutputLabelMapType>(objectPtr->obj->GetObjectAsLabelMap(), offContextObjectLabelMap, objectPtr->id);
			objectPtr->sceneOffContextLabelObject = offContextObjectLabelMap->GetNthLabelObject( 0 );
			objectPtr->sceneOffContextLabelObjectFlag = true;
		}
		else { offContextObjectLabelMap->AddLabelObject( objectPtr->sceneOffContextLabelObject ); }
		return true;
	}


	/** Off Context image of an object: as if it was alone in the scene 
	* The image and label maps are expressed relative to the smallest possible region of the sensor frame (to same memory and computation time...)
	* 	OPTIMIZATION: different possibilities for optimization
	*  - caching the images / labelMap for each object, exploiting particular DataContainer for the scene?
	* returns false if the object is not in the scene
	*/
	bool GetOffContextObjectImage(ObjectInScene *objectPtr, OutputImageType* image, OutputLabelMapType *labelMap) {
		if (!m_imageAllocated) AllocateOutputImage();
		if (!objectPtr) return false;
		//compute the bounding box of interest and the corresponding image grid
		vnl_vector<double> roiBBox;
		if (!IntersectionBoundingBoxes( m_scene->GetSceneBoundingBox(), objectPtr->obj->GetPhysicalBoundingBox(), &roiBBox )) return false;
		OutputImageType::PointType origin; 
		OutputImageType::RegionType region;
		ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing<InputDimension>(roiBBox, m_outputSpacing.GetVnlVector(), &origin, &region);
		
		//Compute the label map
		labelMap->SetOrigin( origin ); labelMap->SetSpacing( m_outputSpacing ); labelMap->SetRegions( region );
		CastLabelMapToOtherFrame<ObjectLabelMapType, OutputLabelMapType>(objectPtr->obj->GetObjectAsLabelMap(), labelMap);

		LabelMapToBinaryImageFilterType::Pointer labelMapToBinaryImageFilter = LabelMapToBinaryImageFilterType::New();
		labelMapToBinaryImageFilter->SetInput( labelMap );
		labelMapToBinaryImageFilter->SetForegroundValue( m_function->Evaluate(1) );
		labelMapToBinaryImageFilter->SetBackgroundValue( m_backgroundValue );
		labelMapToBinaryImageFilter->Update();
		image = labelMapToBinaryImageFilter->GetOutput();

		return true;
	}

	/** Off Context image of an object: as if it was alone in the scene 
	* The image and label maps are expressed relative to the smallest possible region of the sensor frame (to same memory and computation time...)
	* 	OPTIMIZATION: different possibilities for optimization
	*  - caching the images / labelMap for each object, exploiting particular DataContainer for the scene?
	* returns false if the object is not in the scene
	*/
	bool GetOffContextObjectImage(ObjectInScene *objectPtr, OutputImageType* image) {
		if (!m_imageAllocated) AllocateOutputImage();
		if (!objectPtr) return false;
		//compute the bounding box of interest and the corresponding image grid
		vnl_vector<double> roiBBox;
		if (!IntersectionBoundingBoxes( m_scene->GetSceneBoundingBox(), objectPtr->obj->GetPhysicalBoundingBox(), &roiBBox )) return false;
		OutputImageType::PointType origin; 
		OutputImageType::RegionType region;
		ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing<InputDimension>(roiBBox, m_outputSpacing.GetVnlVector(), &origin, &region);
		
		//Compute the label map
		OutputLabelMapType::Pointer labelMap = OutputLabelMapType::New();
		labelMap->SetOrigin( origin ); labelMap->SetSpacing( m_outputSpacing ); labelMap->SetRegions( region );
		CastLabelMapToOtherFrame<ObjectLabelMapType, OutputLabelMapType>(objectPtr->obj->GetObjectAsLabelMap(), labelMap);

		LabelMapToBinaryImageFilterType::Pointer labelMapToBinaryImageFilter = LabelMapToBinaryImageFilterType::New();
		labelMapToBinaryImageFilter->SetInput( labelMap );
		labelMapToBinaryImageFilter->SetForegroundValue( m_function->Evaluate(1) );
		labelMapToBinaryImageFilter->SetBackgroundValue( m_backgroundValue );
		labelMapToBinaryImageFilter->Update();
		image = labelMapToBinaryImageFilter->GetOutput();

		return true;
	}

	//	//1: dimension the output image
	//	//Get the bounding box containing the object in the scene
	//	vnl_vector<double> objectInSceneBBox(2*InputDimension);
	//	if (!IntersectionBoundingBoxes( it->second->object->GetPhysicalBoundingBox(), m_scene->GetSceneBoundingBox(), 
	//		&objectInSceneBBox) );
	//	//
	//	OutputImageType::SpacingType spacing = m_scene->GetSceneSpacing();
	//	OutputImageType::PointType origin; OutputImageType::RegionType region; 
	//	ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing<InputDimension>(objectInSceneBBox, spacing.GetVnlVector(), &origin, &region);

	//	OutputImageType::IndexType offsetIndex;
	//	OutputImageType::PointType sceneOrigin = m_scene->GetSceneOrigin();
	//	//WARNING: this integer division is a potential weak point, that could introduce an offset of 1 pixel...
	//	//seems to be working, but more extensive tests wouldn't be a bad idea.
	//	for (unsigned i=0 ; i<TScene::Dimension ; i++) { offsetIndex[i] = (origin[i] - sceneOrigin[i]) / spacing[i]; }

	//	m_outputSingleObjectImage->SetSpacing(spacing);	
	//	m_outputSingleObjectLabelImage->SetSpacing(spacing);
	//	m_outputSingleObjectImage->SetOrigin(origin);	m_outputSingleObjectLabelImage->SetOrigin(origin);
	//	m_outputSingleObjectImage->SetRegions(region);	m_outputSingleObjectLabelImage->SetRegions(region);

	//	SceneType::LabelType label = it->first;
	//	try { m_outputSingleObjectImage->Allocate(); }
	//	catch( itk::ExceptionObject &e) { std::cerr << "Exception raised allocating the offcontext image of object with label: "<<label<<"\n"<< e << std::endl; }
	//	m_outputSingleObjectImage->FillBuffer(0);

	//	try { m_outputSingleObjectLabelImage->Allocate(); }
	//	catch( itk::ExceptionObject &e) { std::cerr << "Exception raised allocating the offcontext label image of object with label: "<<label<<"\n"<< e << std::endl; }
	//	m_outputSingleObjectLabelImage->FillBuffer(0);

	//	//2: fill the image => iterate on it->second->PixelSet
	//	itk::ImageRegionIteratorWithIndex< OutputImageType > pixelIt( m_outputSingleObjectImage, region );
	//	itk::ImageRegionIteratorWithIndex< OutputLabelImageType > labelIt( m_outputSingleObjectLabelImage, region );
	//	SceneType::PixelSetType *set = it->second->object->GetObjectAsLabelMap()->GetPixelSet();
	//	OutputImageType::IndexType pixelIndex;
	//	for (SceneType::PixelSetType::iterator i = set->begin() ; i != set->end() ; i++) {
	//		pixelIndex = m_scene->FlatIndexToIndex( *i );//this is the index with respect to the whole scene image
	//		//-> convert it to indices with respect to the region of interest
	//		for (unsigned i=0 ; i<TScene::Dimension ; i++) { pixelIndex[i] -= offsetIndex[i]; } //maybe check everything goes well here... ???
	//		pixelIt.SetIndex( pixelIndex );	pixelIt.Set( m_function->Evaluate(1) );
	//		labelIt.SetIndex( pixelIndex );	labelIt.Set( label );
	//	}

	//	image->Graft( m_outputSingleObjectImage );
	//	image->SetPixelContainer(m_outputSingleObjectImage->GetPixelContainer());
	//	image_label->Graft( m_outputSingleObjectLabelImage );
	//	image_label->SetPixelContainer(m_outputSingleObjectLabelImage->GetPixelContainer());
	//
	//	return true;
	//}


protected:
	SimpleUniformSensor() : ImageSensor_Base() {
		m_function = static_cast<AppearanceFunctionType*>(GPF_ConstantFunction<double, double>::New().GetPointer());
		m_appearanceFunctionFlag = true;

		m_labelMapToBinaryImageFilter = LabelMapToBinaryImageFilterType::New();
	};
	~SimpleUniformSensor() {};

	typename LabelMapToBinaryImageFilterType::Pointer m_labelMapToBinaryImageFilter;

	void AllocateOutputImage() {	
		if (!m_scene) throw DeformableModelException("SimpleUniformSensor::AllocateOutputImage the scene must be set first!");

		//WARNING: this may change if resolution is authorized to be different than that of the scene, or if distortion is implemented, ...
		m_outputRegion  = m_scene->GetSceneImageRegion();
		m_outputOrigin  = m_scene->GetSceneOrigin();
		m_outputSpacing = m_scene->GetSceneSpacing();
		m_outputSize    = m_outputRegion.GetSize();
		//
		m_outputImage->SetSpacing( m_outputSpacing );
		m_outputImage->SetOrigin( m_outputOrigin );
		m_outputImage->SetRegions( m_outputRegion );
		try { m_outputImage->Allocate(); }
		catch( itk::ExceptionObject &e) { std::cerr << "Exception raised allocating the sensor image: \n"<< e << std::endl; }
		m_outputImage->FillBuffer(0);

		//
		m_outputLabelMap->SetSpacing( m_outputSpacing );
		m_outputLabelMap->SetOrigin( m_outputOrigin );
		m_outputLabelMap->SetRegions( m_outputRegion );
		m_outputLabelMap->ClearLabels();

		m_imageAllocated = true;
	}

	//Update data in the requested region (CLEAN: use itk -> RequestedRegion instead?)
	void Update_Data(typename InputLabelImageType::RegionType inputRegion) {
		m_outputLabelMap = m_scene->GetSceneAsLabelMap();

		//TODO: update only the requested region... this may require rewriting the filter
		// TEST the ~ set requested region... 
		m_labelMapToBinaryImageFilter->SetInput( m_outputLabelMap );
		m_labelMapToBinaryImageFilter->SetForegroundValue( m_function->Evaluate(1) );
		m_labelMapToBinaryImageFilter->SetBackgroundValue( m_backgroundValue );

		m_labelMapToBinaryImageFilter->Update();

		m_outputImage = m_labelMapToBinaryImageFilter->GetOutput();

		//furthermore, add some functionalities with respect to the m_contextRadius


		////select the interesting region in the sensor image
		//InputLabelImageType *sceneLabelImage = m_outputLabelImage;//m_scene->GetSceneAsLabelImage();

		//typedef itk::ImageRegionIterator<OutputImageType> OutputIteratorType;
		//OutputIteratorType outputIt( m_outputImage, inputRegion );
		//
		//typedef itk::ImageRegionIteratorWithIndex<InputLabelImageType>	InputIteratorType;
		//InputIteratorType  inputIt(sceneLabelImage, inputRegion );

		////define a neighborhood iterator, depending on the m_contextRadius => object too close from modified pixels get their dataCost invalidated
		////an iterator will be created at the requested locations, but is not expected to be moved otherwise, hence the region size = 1
		////the intention is to exploit ITK functionalities to deal with 'border effects' 
		//// => the neighborhood checks bounds if it is close to the image border, and doesn't check anything if it is safely inside the image...
		//typedef itk::ConstNeighborhoodIterator< OutputLabelImageType > NeighborhoodIteratorType;
		//NeighborhoodIteratorType::RadiusType radius; 
		//OutputLabelImageType::RegionType neighbRegion; OutputLabelImageType::RegionType neighbIndex; OutputLabelImageType::SizeType neighbSize;
		//for (unsigned i=0 ; i<OutputLabelImageType::ImageDimension ; i++) { radius[i] = m_contextRadius; neighbSize[i]=1; }
		//NeighborhoodIteratorType neighbIt( radius, m_outputLabelImage, inputRegion );
		//neighbRegion.SetSize(neighbSize); 

		////just set the output to the requested value if the input is not 0
		//OutputImageType::PixelType initialPixel, finalPixel;
		//InputLabelImageType::PixelType label;
		//std::set<InputLabelImageType::PixelType> labelObjectsToInvalidate; std::set<InputLabelImageType::PixelType>::iterator setIt;
		//for ( outputIt.GoToBegin(), inputIt.GoToBegin() ; !outputIt.IsAtEnd(); ++outputIt, ++inputIt) {
		//	initialPixel = outputIt.Get();
		//	label = inputIt.Get();
		//	if (label!=0) finalPixel = m_function->Evaluate(1); 
		//	else finalPixel = 0;
		//	outputIt.Set( finalPixel );

		//	//update the object datacost cache if necessary
		//	if (initialPixel!=finalPixel) { //update the cache of the objects around that pixel				
		//		if (m_contextRadius>0) {//look if there are any objects within a radius of m_radiusInvalidatingObjectDataCost ; and invalidate them too
		//			neighbRegion.SetIndex( inputIt.GetIndex() );
		//			neighbIt = NeighborhoodIteratorType( radius, m_outputLabelImage, neighbRegion ); neighbIt.GoToBegin();
		//			for (unsigned int i = 0; i < neighbIt.Size(); ++i) { label = neighbIt.GetPixel(i); if (label!=0) labelObjectsToInvalidate.insert(label); }
		//		}
		//		else { if (label!=0) labelObjectsToInvalidate.insert(label); }
		//	}
		//}
		//
		//for (setIt = labelObjectsToInvalidate.begin() ; setIt!= labelObjectsToInvalidate.end() ; setIt++) {
		//	ObjectInScene *obj = m_scene->GetObjectByLabel(*setIt);
		//	if ( obj !=0 ) { obj->dataCostFlag = false; }
		//}

	}

private:
	SimpleUniformSensor(const Self&);			//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};



} // namespace psciob

#endif /* __SIMPLEUNIFORMSENSOR_H_ */
