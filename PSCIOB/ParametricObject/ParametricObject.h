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
* \file ParametricObject.h
* \author Rémi Blanc 
* \date 25. July 2011
* 
*/



#ifndef PARAMETRICOBJECT_H_
#define PARAMETRICOBJECT_H_

#include "GeneralUtils.h"
#include "ITKUtils.h"
#include "VTKUtils.h"
#include "vtkPolyDataToITKLabelMapUtils.h"
#include "LabelMapUtils.h"
#include "TransformUtils.h"


//ADD: add the possibility to associate a set of meta-parameters for an object	(e.g. which are not used to described its shape, pose or appearance)
//typically, one could want to tell if that particular spherical object is a star or a planet ; floating in space... using meta-data
// ~> scalar & categorical meta-data...

namespace psciob {

/** \class ParametricObject
* \brief ParametricObject is the base class for all parametric objects 
*		  Parameters can refer to the position, shape, or texture of the object
*
* ParametricObject is an abstract object, templated against the dimensionality 
* and appearance (texture) type, that specifies the base interface for objects
*
* For binary objects, the base mechanism is to generate a vtkPolyData, and then convert it to Pixel Representations
*
* Except for 'Shapes', the first "VDimension" parameters always indicate the position in space ; typically the translation parameters
*/

//ABSTRACT CLASS
template <unsigned int VDimension, class TAppearance = unsigned char>
class ParametricObject : public itk::LightObject {
public:
	/** Standard class typedefs. */
	typedef ParametricObject              Self;
	typedef itk::LightObject              Superclass;
	typedef itk::SmartPointer<Self>       Pointer;
	typedef itk::SmartPointer<const Self> ConstPointer;
	/** Base Types to which all inherited classes can refer */
	typedef Self                          BaseClass;
	typedef Pointer                       BaseClassPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(ParametricObject, itk::LightObject);

	/** Dimension of the space in which the object is defined */
	static const unsigned int Dimension = VDimension;

	/** Types related to the binary image representation */
	typedef unsigned char								BinaryPixelType;
	typedef itk::Image<BinaryPixelType, VDimension>		BinaryImageType;
	typedef typename BinaryImageType::Pointer			BinaryImagePointerType;

	/** Types related to the textured image representation */
	typedef TAppearance									TexturedPixelType;
	typedef itk::Image<TexturedPixelType, VDimension>	TexturedImageType;
	typedef typename TexturedImageType::Pointer			TexturedImagePointerType;

	/** Types related to the LabelMap representation, based on a Run Length Encoding */
	typedef itk::LabelObject<unsigned char, Dimension>	LabelObjectType;
	typedef itk::LabelMap<LabelObjectType>				LabelMapType;

	/** Types of objects that could potentially be available for an object */
	typedef enum {
		MESHFORMAT, 
		TEXTUREDMESHFORMAT,
		BINARYIMAGEFORMAT,
		TEXTUREDIMAGEFORMAT,
		LABELMAPFORMAT
	} ObjectFormatType;


	/** Give Information about Self */
	virtual std::string GetClassName() const = 0;
	virtual void PrintInfo()           const = 0;

	/** Get Number of Parameters */
	virtual unsigned int GetNumberOfParameters()      const = 0;

	/** Get Parameters 
	* not const, because the PoseTransform may need to update the parameters 
	*/
	virtual inline vnl_vector<double> GetParameters() { return m_parameters; }

	/** Get Parameters */
	virtual vnl_vector<double> GetDefaultParameters() const = 0;


	/** Set Parameters - internally check whether the parameters are valid and returns false if not
	* Look for pure integer grid translation
	* If a single parameters is different, then the whole shape needs to be update
	*/
	virtual bool SetParameters(const vnl_vector<double> &params);

	/** Set Default Parameters */
	virtual inline void SetDefaultParameters() { m_parameters = GetDefaultParameters(); Modified(); }

	/** Check if the parameters are valid */
	virtual bool CheckParameters(const vnl_vector<double> &p) const = 0;

	/** Position At the specified location
	* checks if some updates are necessary or if things boil down to a grid-integer translation
	* return false if something goes wrong
	*/
	virtual bool PositionAt(const vnl_vector<double> &pos);

	/** Apply Translation
	* checks if this is a grid-integer translation
	* return false if something goes wrong
	*/
	virtual bool Translate(const vnl_vector<double> &translation);

	/** \param translation is the translation vector to apply 
	* \param params is a vector of object parameters 
	* The function modifies these input parameters such that the new parameters correspond to the translated object
	* \warning: no check are perform to verify the validity of the inputs
	*/
	void ApplyTranslationToParameters(const vnl_vector<double> &translation, vnl_vector<double> &params) {
		for (unsigned i=0 ; i<Dimension ; i++) params(i)+=translation(i);
	}
	
	/** \param scaling to apply 
	* \param params is a vector of object parameters 
	* The function modifies these input parameters such that the new parameters correspond to the scaled object
	* \warning: no check are perform to verify the validity of the inputs
	*/
	virtual void ApplyScalingToParameters(double scaleFactor, vnl_vector<double> &params) = 0;

	/** \param rotation matrix to apply (pre-compose: rotate the object around its center)
	* \param params is a vector of object parameters 
	* The function modifies these input parameters such that the new parameters correspond to the rotated object
	* \warning: no check are perform to verify the validity of the inputs
	*/
	virtual void ApplyRotationToParameters(vnl_matrix<double> rot, vnl_vector<double> &params) = 0;
	
	/** Apply an integer-grid translation, these can be applied very quickly to image representations and avoid recalculating these */
	virtual bool IntegerGridTranslate(const vnl_vector<int> &translation) = 0;

	/** This function creates a new instance of the object, with the same parameters, image spacing, and VTKResolution 
	* However, it does not copies the memory contents (e.g. the vtkPolyData, or associated image)
	*/
	virtual BaseClassPointer CreateCopy() = 0;

	/** This function creates a full copy object, including the vtkPolyData, LabelMap, and itkImage	*/
	virtual BaseClassPointer CreateClone() = 0;

	/** Indicate that the output should be recomputed */
	virtual inline void Modified() {
		m_physicalBBoxUpToDate = false; m_imageBBoxUpToDate = false; 
		m_uptodatePolyData = false; m_uptodateBinaryImage = false; m_uptodateLabelMap = false;
	}

	/** Release memory - may be overloaded by child classes */
	virtual void ReleaseMemory_Light() {}
	virtual void ReleaseMemory_Full() { m_outputPolyData->Initialize(); m_outputBinaryImage->Initialize(); m_outputLabelMap->Initialize(); Modified(); }

	/** Clear memory and resets shape parameters to default */
	void Reset() { ReleaseMemory_Full(); SetDefaultParameters(); }

	/** Get the physical, axis-aligned bounding box
	* The format is [d1_min d1_max d2_min d2_max ...]
	*/
	virtual vnl_vector<double> GetPhysicalBoundingBox()	= 0;

	/** Get the physical, axis-aligned bounding box containing the Image representation
	* It generally differs from the physical bounding box, because it depends on the pixel dimensions (spacing)
	* Furthermore, the convention is that the physical 0 is at the center of a pixel
	* The format is [d1_min d1_max d2_min d2_max ...]
	*/
	vnl_vector<double> GetImageBoundingBox() {
		if (!m_imageBBoxUpToDate) {
			ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing<VDimension>( GetPhysicalBoundingBox(), m_imageSpacing.GetVnlVector(), &m_imageOrigin, &m_imageRegion );
			m_imageBoundingBox = BoundingBoxFromITKImageInformation<BinaryImageType>(m_imageOrigin, m_imageSpacing, m_imageRegion);
			m_imageBBoxUpToDate=true;
		}
		return m_imageBoundingBox;
	}

	/** Get Image Region - it is automatically determined by the physical bounding box of the object and the spacing */
	typename BinaryImageType::RegionType GetImageRegion() const {
		if (!m_imageBBoxUpToDate) {
			ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing<VDimension>( GetPhysicalBoundingBox(), m_imageSpacing.GetVnlVector(), &m_imageOrigin, &m_imageRegion );
			m_imageBoundingBox = BoundingBoxFromITKImageInformation<BinaryImageType>(m_imageOrigin, m_imageSpacing, m_imageRegion);
			m_imageBBoxUpToDate=true;
		}
		return m_imageRegion;
	}

	/** Get Image Origin - it is automatically determined by the physical bounding box of the object and the spacing */
	typename BinaryImageType::PointType GetImageOrigin() const {
		if (!m_imageBBoxUpToDate) {
			ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing<VDimension>( GetPhysicalBoundingBox(), m_imageSpacing.GetVnlVector(), &m_imageOrigin, &m_imageRegion );
			m_imageBoundingBox = BoundingBoxFromITKImageInformation<BinaryImageType>(m_imageOrigin, m_imageSpacing, m_imageRegion);
			m_imageBBoxUpToDate=true;
		}
		return m_imageOrigin;
	}

	/** Get Image Spacing */
	typename BinaryImageType::SpacingType GetImageSpacing()	const { return m_imageSpacing; }

	/** Set the image spacing 
	* Invalidates the current pixel-based representations if the spacing changes
	*/
	virtual void SetImageSpacing(const vnl_vector<double> &sp) {
		if (sp.size()!=VDimension) throw DeformableModelException("SetImageSpacing : unexpected dimension");
		for (unsigned i = 0 ; i<VDimension ; i++) { 
			if (sp[i]<=0) throw DeformableModelException("SetImageSpacing : spacing must be strictly positive");
			if ( fabs(m_imageSpacing[i]-sp[i])>TINY ) { m_uptodateBinaryImage = false; m_uptodateLabelMap = false; m_imageBBoxUpToDate = false; }			
		} 
		m_imageSpacing.SetVnlVector( sp );
	}
	virtual void SetImageSpacing(typename BinaryImageType::SpacingType sp) {
		for (unsigned i = 0 ; i<VDimension ; i++) { 
			//if (sp[i]<=0) throw DeformableModelException("SetImageSpacing : spacing must be strictly positive");
			if ( fabs(m_imageSpacing[i]-sp[i])>TINY ) { m_uptodateBinaryImage = false; m_uptodateLabelMap = false; m_imageBBoxUpToDate = false; }
			m_imageSpacing[i] = sp[i];
		} 		
	}
	virtual void SetImageSpacing(double sp)	{ 
		if (sp<=0) throw DeformableModelException("SetImageSpacing : spacing must be strictly positive");
		for (unsigned i = 0 ; i<VDimension ; i++) { 
			if ( fabs(m_imageSpacing[i]-sp)>TINY ) { m_uptodateBinaryImage = false; m_uptodateLabelMap = false; m_imageBBoxUpToDate = false; }
			m_imageSpacing[i] = sp;
		} 
	}

	/** Set Resolution parameters for the VTK mesh representation 
	* Invalidates the mesh-based representations if these changes, as well as the pixel-based representation deriving from a mesh
	*/
	virtual bool SetVTKResolution(const vnl_vector<double> &res) = 0;
	/** Get VTK Resolution */
	vnl_vector<double> GetVTKResolution() const { return m_vtkResolution; }

	/** Ask if the corresponding object representation is uptodate */
	inline bool IsObjectPhysicalBoundingBoxUpToDate() { return m_physicalBBoxUpToDate; }
	inline bool IsObjectImageBoundingBoxUpToDate()    { return m_imageBBoxUpToDate; }
	inline bool IsObjectPolyDataUpToDate()            { return m_uptodatePolyData; }
	inline bool IsObjectBinaryImageUpToDate()         { return m_uptodateBinaryImage; }
	inline bool IsObjectLabelMapUpToDate()            { return m_uptodateLabelMap; }
	virtual bool IsObjectTexturedPolyDataUpToDate()   = 0;
	virtual bool IsObjectTexturedImageUpToDate()      = 0;

	/** Get the corresponding representation of the object */
	virtual vtkPolyData* GetObjectAsVTKPolyData() = 0;

	/** Get the set of pixel occupied by the object, at the given spacing and with respect to the tightest bounding box containing the object */
	virtual LabelMapType* GetObjectAsLabelMap() {
		if (!m_uptodateLabelMap) {
			//instanciate the LabelMap if necessary
			if(!m_outputLabelMap) m_outputLabelMap = LabelMapType::New();
			//update the labelMap = LabelMap
			VTKPolyDataToLabelMap<LabelMapType>( GetObjectAsVTKPolyData(), m_outputLabelMap, m_imageSpacing );
			m_uptodateLabelMap=true;
		}
		return m_outputLabelMap.GetPointer();
	}

	/** Get the set of pixel occupied by the object, at the given spacing and with respect to the tightest bounding box containing the object */
	virtual BinaryImageType* GetObjectAsBinaryImage() {
		if (!m_uptodateBinaryImage) {
			//instanciate the filter if necessary
			if (!m_labelMapToBinaryImageFilter) m_labelMapToBinaryImageFilter = LabelMapToBinaryImageFilterType::New();
			//generate a binary image out of the pixel set
			m_labelMapToBinaryImageFilter->SetInput( GetObjectAsLabelMap() );
			m_labelMapToBinaryImageFilter->Update();
			//m_outputBinaryImage->Graft( m_labelMapToLabelImageFilter->GetOutput() );
			m_outputBinaryImage = m_labelMapToBinaryImageFilter->GetOutput();
			m_uptodateBinaryImage=true;
		}
		return m_outputBinaryImage.GetPointer();
	}

	//not yet sure how to deal with these...
	virtual TexturedImageType*  GetObjectAsTexturedImage()       = 0;
	virtual vtkPolyData*        GetObjectAsTexturedVTKPolyData() = 0;

protected:
	ParametricObject();
	virtual ~ParametricObject() {}

	vnl_vector<double> m_parameters;

	typename BinaryImageType::SpacingType m_imageSpacing;
	typename BinaryImageType::RegionType  m_imageRegion;
	typename BinaryImageType::PointType   m_imageOrigin;

	vnl_vector<double> m_physicalBoundingBox;              bool m_physicalBBoxUpToDate;
	vnl_vector<double> m_imageBoundingBox;                 bool m_imageBBoxUpToDate; 
	vnl_vector<double> m_vtkResolution;

	vtkSmartPointer<vtkPolyData>      m_outputPolyData;    bool m_uptodatePolyData;
	typename BinaryImageType::Pointer m_outputBinaryImage; bool m_uptodateBinaryImage;
	typename LabelMapType::Pointer    m_outputLabelMap;    bool m_uptodateLabelMap;

	typedef itk::LabelMapToBinaryImageFilter< LabelMapType, BinaryImageType> LabelMapToBinaryImageFilterType;
	typename LabelMapToBinaryImageFilterType::Pointer m_labelMapToBinaryImageFilter;

	// protected internal mechanism for setting the parameters, without checking for their validity or their non-novelty
	virtual void _SetParameters_NoCheck(const vnl_vector<double> &params) { m_parameters = params; Modified(); }

	// protected internal mechanism for setting the parameters, without checking for their validity or their non-novelty
	virtual bool _IntegerGridTranslate_NoCheck(const vnl_vector<int> &translation) {
		double tmp;
		for (unsigned i=0 ; i<Dimension ; i++) {
			if ( translation(i)!=0 ) {
				m_uptodatePolyData = false; 
				tmp = translation(i)*m_imageSpacing[i];
				m_parameters(i) += tmp;
				m_imageOrigin[i] += tmp;
				if (m_physicalBBoxUpToDate) { m_physicalBoundingBox(2*i) += tmp; m_physicalBoundingBox(2*i+1) += tmp; }
				if (m_imageBBoxUpToDate)    { m_imageBoundingBox(2*i)    += tmp; m_imageBoundingBox(2*i+1)    += tmp; }
			}
		}

		if (m_uptodateBinaryImage) m_outputBinaryImage->SetOrigin(m_imageOrigin);
		if (m_uptodateLabelMap)    m_outputLabelMap->SetOrigin(m_imageOrigin);
		return true;
	}


private:
	ParametricObject(const Self&);           //purposely not implemented
	const Self & operator=( const Self & ); //purposely not implemented
};



} // namespace psciob

#include "ParametricObject.txx"

#endif /* PARAMETRICOBJECT_H_ */



