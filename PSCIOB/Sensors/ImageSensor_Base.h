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
* \file ImageSensor_Base.h
* \author Rémi Blanc 
* \date 28. September 2011
*/

#ifndef __IMAGESENSORBASE_H_
#define __IMAGESENSORBASE_H_

#include "BaseScene.h"
#include "GenericParametricFunctions.h"
#include "LabelMapUtils.h"

#include "SensorSceneInterface.h"

namespace psciob {

/**
 * \class ImageSensor_Base
 * \brief ImageSensor_Base
 * the input is a scene that produces an image!
 * their are 2 basic outputs: an image, and a LabelMap
 *
 * TODO: add a (generic) mechanism to insert noise (additive or convolutive...) in the image formation
 *
 * TODO: add methods to control the image resolution of the sensor ; should it be tied to the voxel representation of the scene, as it is now? ; or should it be independent?
 *
 * TODO: make it possible to optimize the sensor parameters => need to define a base(virtual) interface here to get/set all tunable parameters...
 * as a first step => brightness & contrast <=> ok
 *
 * types of parameters:
 * camera orientation -> different child classes have different nb of params (1 parameters <-> face), other could be more arbitrary
 * geometric distortion -> perspective distortion, etc...
 * noise parameters -> e.g. Poisson noise ; blur effect, ...
 * resolution ...
 * appearance function...
 * 
 */


//ABSTRACT CLASS
template<class TScene, class TOutputImage>
class ImageSensor_Base : public itk::LightObject {
public:
	/** Standard class typedefs. */
	typedef ImageSensor_Base              Self;
	typedef itk::LightObject              Superclass;
	typedef itk::SmartPointer<Self>	      Pointer;
	typedef itk::SmartPointer<const Self> ConstPointer;
	/** Base Types to which all inherited classes can refer */
	typedef Self                          BaseClass;
	typedef Pointer                       BaseClassPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(ImageSensor_Base, itk::LightObject);

	static const unsigned int InputDimension = TScene::Dimension;

	typedef TScene                             SceneType;
	typedef typename SceneType::ObjectInScene  ObjectInScene;
	typedef typename SceneType::IDType         LabelType;

	typedef typename SceneType::DeformableObjectType::LabelObjectType OriginalLabelObjectType;
	typedef typename SceneType::DeformableObjectType::LabelMapType    OriginalLabelMapType;
	typedef typename SceneType::DeformableObjectType::LabelMapType    ObjectLabelMapType;

	typedef	typename SceneType::LabelObjectType    InputLabelObjectType;
	typedef	typename SceneType::LabelMapType       InputLabelMapType;
	typedef	typename SceneType::LabelImageType     InputLabelImageType;
	typedef	typename SceneType::BinaryImageType    InputBinaryImageType;
	typedef	typename SceneType::TexturedImageType  InputTexturedImageType;

	typedef SensorSceneInterface<typename SceneType::BaseClass> SensorSceneInterfaceType;
	typedef typename SceneType::BinaryImageType::RegionType     SceneRegionType;
	typedef typename std::vector<SceneRegionType>               SceneRegionListType;

	static const unsigned int OutputDimension   = TOutputImage::ImageDimension;

	typedef TOutputImage                                  OutputImageType;
	typedef typename OutputImageType::PixelType           OutputPixelType;
	typedef itk::LabelObject<LabelType, OutputDimension>  OutputLabelObjectType;
	typedef itk::LabelMap<OutputLabelObjectType>          OutputLabelMapType;
	typedef itk::Image<LabelType, OutputDimension>        OutputLabelImageType;

	typedef itk::LabelMapToLabelImageFilter< OutputLabelMapType, OutputLabelImageType> OutputLabelMapToLabelImageFilterType;


	//a priori, I don't care about the pixel value in the scene (should just care if it is empty or not...)
	typedef GenericParametricFunctions<double, double>    AppearanceFunctionType;

	//maybe other classes here that manage different parameters of the sensor (camera orientation, noise, etc...) could be type-defined

	enum UPDATEFLAGS { UPDATEAll, UPDATEAppearanceOnly, UPTODATE };

	/** Create a clone (= an exact, independant copy) of the current sensor 
	* the clone is however generated outdated : it will have to recompute its output if immediately asked for it	
	*/
	//It would not make much sense to make a clone only for the fun of it..., there is a chance that the user wants the same sensor, but working on a different scene, or with different parameters
	//WARNING: if different sensors work on the same scene, there WILL be some issues with the update mechanisms!!!
	virtual BaseClassPointer CreateClone();

	/** Set the scene to observe
	* By default, the sensor is connected to the scene, to get informed of the scene regions that are being modified
	* so it can update itself only on modified regions
	*/
	virtual void SetScene(SceneType *scene);
	/** Get a pointer to the observed scene */
	SceneType* GetScene()           { return m_scene;}

	//the dimension of the parameter vectors should be set in the child classes 
	//see the comments below ; if some of these concepts are handled through external classes, these functions need not be virtual
	//the corresponding vector of parameter needs not be stored as a member, just the pointer to the responsible class
	virtual unsigned int GetNumberOfPoseParameters() = 0;
	virtual unsigned int GetNumberOfDistortionParameters()  = 0;
	virtual unsigned int GetNumberOfNoiseParameters()       = 0;
	virtual unsigned int GetNumberOfResolutionParameters()  = 0;
	
	/** get the bounding box, image of an input bbox observed through the sensor */
	virtual vnl_vector<double> GetSensedBoundingBox(const vnl_vector<double> &bbox) = 0;

	/** Update the sensor */
	void Update();

	//treat separately modifications on the geometry of the image, and modifications that only affect the appearance
	//in the first case, memory should be re-allocated
	void Modified()           { m_scene->InvalidateObjectDataCosts();     m_updateFlag = UPDATEAll; } //here, I certainly need to re-allocate the image!!
	void ModifiedAppearance() { m_scene->InvalidateObjectDataCosts(); if (m_updateFlag!= UPDATEAll)  m_updateFlag = UPDATEAppearanceOnly; }


	//TODO: check validity of the various parameters !!!! the SetXXXParameters should return a bool indicating whether this was performed correctly or not!
	//TAKE CARE THAT ALL THE PARAMETERS OF THIS FUNCTION ARE SUSCEPTIBLE TO MODIFICATION AND OPTIMIZATION...
	void SetAppearanceFunction(AppearanceFunctionType *fct)	{ m_function = fct;	 m_appearanceFunctionFlag = true;  ModifiedAppearance(); }

	bool SetAppearanceParameters(vnl_vector<double> p);
	
	/* Set the pixel 'color' for background pixels (absence of objects) */
	bool SetBackgroundValue(OutputPixelType v = 0) { 
		if (m_backgroundValue!=v) { ModifiedAppearance(); m_backgroundValue=v; }
		return true;
	}

	/* Get the value for background pixels */
	typename OutputPixelType GetBackgroundValue() const {return m_backgroundValue;}

	//IDEA: should (some of) these sets of parameters be managed through generic classes ; as it done for the AppearanceFunction ? or should this be handled through concrete implementation of the sensor ? => handling of OrientationParameters is hard-coded in the Simple3D2DLabelSensor = only 3 possible directions SAGITAL / HORIZONTAL / CORONAL...
	//see comments in the Simple3D2DSensor
	virtual bool SetPoseParameters(const vnl_vector<double> &p);
	bool SetDistortionParameters(const vnl_vector<double> &p);
	bool SetNoiseParameters(const vnl_vector<double> &p);
	bool SetResolutionParameters(const vnl_vector<double> &p);

	unsigned int GetNumberOfAppearanceParameters()              { return m_function->GetNumberOfParameters(); }
	inline vnl_vector<double> GetAppearanceParameters()         { return m_function->GetParameters(); }

	inline const vnl_vector<double>& GetOrientationParameters() { return m_poseParams; }
	inline const vnl_vector<double>& GetDistortionParameters()  { return m_distortionParams; }
	inline const vnl_vector<double>& GetNoiseParameters()       { return m_noiseParams; }
	inline const vnl_vector<double>& GetResolutionParameters()  { return m_resolutionParams; }

	/** Get Image Outputs */
	inline OutputImageType*       GetOutputImage()      { Update(); return m_outputImage.GetPointer(); }
	inline OutputLabelMapType*    GetOutputLabelMap()   { Update(); return m_outputLabelMap.GetPointer(); }
	virtual OutputLabelImageType* GetOutputLabelImage();

	/** Get Information about the output image */
	typename const OutputImageType::RegionType&  GetOutputImageRegion()  { return m_outputRegion; }
	typename const OutputImageType::SizeType&    GetOutputImageSize()    { return m_outputSize; }
	typename const OutputImageType::PointType&   GetOutputImageOrigin()  { return m_outputOrigin; }
	typename const OutputImageType::SpacingType& GetOutputImageSpacing() { return m_outputSpacing; }


	/** Set the radius defining the context around an object
	 * this is used to: - define the pixelset indicating which pixels belong to the context of an object
	 *                  - not implemented yet: invalidate the dataCost if another object comes this close to another one 
	*/
	void SetObjectContextRadius(unsigned int nbPixels);
	/** Get the number of pixels that define the context, or neighborhood, of an object (through dilation by a ball structuring element) */
	unsigned int GetObjectContextRadius() { return m_contextRadius; }

	/** Get the labelMap containing only the requested object (with its correct label), off context 
	* this labelMap is expressed with respect to the full sensor grid
	* returns false if the object is not in the scene
	* \param objectPtr is a pointer to the object of the scene to be processed
	* \param offContextObjectLabelMap is a pointer to be filled by the function
	*/
	virtual bool GetOffContextObjectLabelMap(ObjectInScene *objectPtr, OutputLabelMapType *offContextObjectLabelMap) = 0;
	
	//the function below is useless ; more interesting would be to get the labelmap &/ image of a DeformableObject
	//inline bool GetOffContextObjectLabelMap(LabelType id, OutputLabelMapType *offContextObjectLabelMap) {
	//	if (!m_scene) throw DeformableModelException("SimpleUniformSensor::GetOffContextObjectLabelMap the scene must be set first!");
	//	return GetOffContextObjectLabelMap(m_scene->GetObject(id), offContextObjectLabelMap);
	//}
	
	/** Get an off context image of the object and the corresponding labelmap containing only the requested object (with its correct label)
	* The image and label maps are expressed relative to the smallest possible region of the sensor frame (to same memory and computation time...)
	* IDEA: cache these elements in a specific dataContainer
	*/
	virtual bool GetOffContextObjectImage(ObjectInScene *objectPtr, OutputImageType* image, OutputLabelMapType *labelMap) = 0;
	virtual bool GetOffContextObjectImage(ObjectInScene *objectPtr, OutputImageType* image) = 0;


	/** Get a labelMap containing only the requested off context object, AND its surrounding (with a radius specified to the sensor)
	* The map contains 2 labelObject: the object (with its correct label), and the surroundings (with label+1)
	* this labelMap is expressed with respect to the full sensor grid
	* returns false if the object is not in the scene
	* \param objectPtr is a pointer to the object of the scene to be processed
	* \param offContextLabelMap is a pointer to be filled by the function
	*/
	bool GetOffContextObjectAndSurroundingsLabelMap(ObjectInScene *objectPtr, OutputLabelMapType *labelMap);


	/** Get the labelMap containing only the requested object (with its correct label), off context 
	* this labelMap is expressed with respect to the full sensor grid
	* returns false if the object is not visible on the sensor
	* \param objectPtr is a pointer to the object of the scene to be processed
	* \param offContextObjectLabelMap is a pointer to be filled by the function
	*/
	bool GetInContextObjectLabelMap(ObjectInScene *objectPtr, OutputLabelMapType *inContextObjectLabelMap);


	/** returns a value between 0 and 1 indicating the fraction of observed pixels, compared to the potentially observed pixels
	 * a value less than 1 indicates that an object is 'shadowed' by another, either due to overlapping, or projection effects
	 */
	double GetSingleObjectVisibilityPercentage( LabelType id );


	/** Get a labelMap containing only the requested off context object, AND its surrounding (with a radius specified to the sensor)
	* The map contains 2 labelObject: the object (with its correct label), and the surroundings (with label+1)
	* this labelMap is expressed with respect to the full sensor grid
	* returns false if the object is not visible / not in the scene
	* \param objectPtr is a pointer to the object of the scene to be processed
	* \param offContextLabelMap is a pointer to be filled by the function
	*/
	bool GetInContextObjectAndSurroundingsLabelMap(ObjectInScene *objectPtr, OutputLabelMapType *labelMap);

protected:
	ImageSensor_Base();
	~ImageSensor_Base() {};

	typename SceneType::Pointer	m_scene;
	typename SensorSceneInterfaceType::Pointer m_sceneInterface;

	typename OutputImageType::Pointer		m_outputImage;   //, m_outputSingleObjectImage; 
	typename OutputLabelMapType::Pointer	m_outputLabelMap, m_tmpSingleObjectLabelMap;
	bool m_imageAllocated;

	typename OutputImageType::RegionType  m_outputRegion;
	typename OutputImageType::SizeType    m_outputSize;
	typename OutputImageType::PointType   m_outputOrigin;
	typename OutputImageType::SpacingType m_outputSpacing;

	UPDATEFLAGS m_updateFlag;

	typename AppearanceFunctionType::Pointer m_function; bool m_appearanceFunctionFlag; //TODO: use a function templated against the OutputImageType::PixelType as output (and scalar/vector input(s)...)
	
	OutputPixelType m_backgroundValue;
	vnl_vector<double> m_poseParams, m_distortionParams, m_noiseParams, m_resolutionParams;
	bool			   m_orientationFlag,   m_distortionFlag,   m_noiseFlag,   m_resolutionFlag;

	typename OutputLabelMapToLabelImageFilterType::Pointer m_outputLabelMapToLabelImageFilterType;

	// Allocate the output images
	virtual void AllocateOutputImage() = 0;	//allocate the output image when all dimension characteristic are known
	// Update the specified region of the image...
	virtual void Update_Data(typename InputLabelImageType::RegionType inputRegion = m_scene->m_regionWholeScene) = 0; //update everything by default, but only the part of the scene that changed otherwis
	//virtual void Update_AppearanceData(typename InputImageType::RegionType inputRegion = m_scene->m_regionWholeScene) = 0; //update everything by default, but only the part of the scene that changed otherwise
	//virtual void Update_LabelData(typename InputImageType::RegionType inputRegion = m_scene->m_regionWholeScene) = 0; //update everything by default, but only the part of the scene that changed otherwise

	unsigned int m_contextRadius;
	typename OutputLabelObjectType::Pointer m_StructuringElement;

private:
	ImageSensor_Base(const Self&);			//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};

} // namespace psciob

#include "ImageSensor_Base.txx"


#endif /* __IMAGESENSORBASE_H_ */
