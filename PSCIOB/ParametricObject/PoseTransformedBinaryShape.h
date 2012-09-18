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
* \file PoseTransformedBinaryShape.h
* \author Rémi Blanc 
* \date 2. March 2012
*/

#ifndef POSETRANSFORMEDBINARYSHAPE_H_
#define POSETRANSFORMEDBINARYSHAPE_H_

#include "BinaryDeformableModel.h"
#include "BinaryShape.h"
#include "PoseTransform.h"

namespace psciob {

/** 
* \class PoseTransformedBinaryShape
* \brief PoseTransformedBinaryShape this defines a BinaryDeformableModel applying a Pose Transform to a BinaryShape
*
* \WARNING do not modify the transform directly, it might lead to unexpected behaviors.
* \todo compute analytically the inertia matrices for all shapes, and apply the transform to it (check what happens wrt scale & affine transforms...)
*/

//CONCRETE CLASS
template < unsigned int VDimension > 
class PoseTransformedBinaryShape : public BinaryDeformableModel<VDimension> {
public:
	/** Standard class typedefs. */
	typedef PoseTransformedBinaryShape      Self;
	typedef BinaryDeformableModel           Superclass;
	typedef itk::SmartPointer<Self>         Pointer;
	typedef itk::SmartPointer<const Self>   ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(PoseTransformedBinaryShape, BinaryDeformableModel);
	itkSimpleNewMacro(Self); //SimpleNewMacro is necessary to overload the CreateAnother method

	/** Give Information about Self */
	std::string GetClassName() const { return ("PoseTransformedBinaryShape"); }
	void PrintInfo() const { std::cout<<"PoseTransformedBinaryShape composed of:\n"; m_shape->PrintInfo(); m_transform->PrintInfo(); }

	/** Get Number of Parameters */
	inline unsigned int GetNumberOfParameters() const { 
		return (m_shape->GetNumberOfParameters() + m_transform->GetNumberOfParameters());
	}

	/** Type of the shape to be pose-transformed */
	typedef BinaryShape<Dimension>                      ShapeType;
	/** Type of the pose transform to use */
	typedef PoseTransform<Dimension, TexturedPixelType> TransformType;

	/** Creates another ShapeAndPoseDeformableObject object, with shape and transforms of the same type 
	* the new object has default initialization (doesn't copy the parameters, or any contents)
	*/
	virtual itk::LightObject::Pointer CreateAnother(void) const;

	/** This function creates a full copy object, including the vtkPolyData, LabelMap, and itkImage	*/
	virtual BaseClassPointer CreateCopy() {	
		Pointer clonePtr = Self::New().GetPointer();
		clonePtr->SetShapeAndTransform(static_cast<ShapeType*>(m_shape->CreateCopy().GetPointer()), static_cast<TransformType*>(m_transform->CreateCopy().GetPointer()));
		clonePtr->m_imageRegion   = m_imageRegion;	
		clonePtr->m_imageOrigin   = m_imageOrigin;
		return static_cast<BaseClass*>( clonePtr );
	}

	/** This function creates a full copy object, including the vtkPolyData, LabelMap, and itkImage	*/
	virtual BaseClassPointer CreateClone() {	
		Pointer clonePtr = Self::New().GetPointer();
		clonePtr->SetShapeAndTransform(static_cast<ShapeType*>(m_shape->CreateClone().GetPointer()), static_cast<TransformType*>(m_transform->CreateClone().GetPointer()));
		//clonePtr->m_parameters = m_parameters; //m_parameters are already maintained uptodate by SetShapeAndTransform, in coherence with the parameters of the shape&transform
		//clonePtr->m_imageSpacing  = m_imageSpacing;
		//clonePtr->m_vtkResolution = m_vtkResolution;

		if (m_physicalBBoxUpToDate) {
			clonePtr->m_physicalBoundingBox = m_physicalBoundingBox; 
			clonePtr->m_physicalBBoxUpToDate = true; 
		}

		if (m_imageBBoxUpToDate) { 
			clonePtr->m_imageBoundingBox = m_imageBoundingBox; 
			clonePtr->m_imageRegion   = m_imageRegion;	
			clonePtr->m_imageOrigin   = m_imageOrigin;
			clonePtr->m_imageBBoxUpToDate = true; 
		}

		if (m_uptodatePolyData) { //not really necessary...
			clonePtr->m_outputPolyData = vtkSmartPointer<vtkPolyData>::New();
			clonePtr->m_outputPolyData->DeepCopy( m_outputPolyData );
			clonePtr->m_uptodatePolyData = true; 
		}

		if (m_uptodateBinaryImage) {
			clonePtr->m_outputBinaryImage = BinaryImageType::New();
			CloneITKImage<BinaryImageType>(m_outputBinaryImage, clonePtr->m_outputBinaryImage);
			clonePtr->m_uptodateBinaryImage = true;
		}

		if (m_uptodateLabelMap) {
			clonePtr->m_outputLabelMap = LabelMapType::New();
			clonePtr->m_outputLabelMap->Graft(m_outputLabelMap);
			clonePtr->m_uptodateLabelMap = true;
		}

		if (m_centerFlag)      { clonePtr->m_centerFlag = true; clonePtr->m_center = m_center; }
		if (m_inertiaFlag)     { clonePtr->m_inertiaFlag = true; clonePtr->m_inertia = m_inertia; }
		if (m_eigVInertiaFlag) { clonePtr->m_eigVInertiaFlag = true; clonePtr->m_eigVInertia = m_eigVInertia; }

		return static_cast<BaseClass*>( clonePtr );
	}

	/** Set the binary shape and the transform defining the deformable model */
	void SetShapeAndTransform( ShapeType *shape, TransformType *transf );

	/** Get Default Parameters */
	vnl_vector<double> GetDefaultParameters() const;


	/** Get a pointer to the shape 
	* \todo output should be const to prevent modifications outside this class
	*/
	inline ShapeType* GetShape() { return m_shape.GetPointer(); }

	/** Get a pointer to the transform 
	* \todo output should be const to prevent modifications outside this class
	*/
	inline TransformType* GetTransform() { return m_transform.GetPointer(); }

	///** Get Parameters, in the following order: pose then shape parameters */
	//inline vnl_vector<double> GetParameters() {return m_parameters;}


	/** Indicate that the output should be recomputed */
	virtual inline void Modified() {
		ParametricObject<VDimension, TexturedPixelType>::Modified();
		m_shape->Modified();
		m_transform->Modified();
	}

	/** Set Parameters in the following order: pose then shape parameters
	* internally check whether the parameters are valid and returns false if not
	* Look for pure integer grid translation
	* If a single parameters is different, then the whole shape needs to be update
	*/
	bool SetParameters(const vnl_vector<double> &params);

	/** Set Default Parameters */
	void SetDefaultParameters();

	/** Check if the parameters are valid */
	bool CheckParameters(const vnl_vector<double> &p) const {
		if (p.size() != GetNumberOfParameters()) return false;
		if (!m_transform->CheckParameters( p.extract( m_transform->GetNumberOfParameters(), 0 ) )) { return false; }
		return m_shape->CheckParameters( p.extract( m_shape->GetNumberOfParameters(), p.size() - m_transform->GetNumberOfParameters() ) );
	}

	/** Position At the specified location
	* checks if some updates are necessary or if things boil down to a grid-integer translation
	* return false if something goes wrong
	*/
	bool PositionAt(const vnl_vector<double> &pos);

	/** Apply Translation
	* checks if this is a grid-integer translation
	* return false if something goes wrong
	*/
	bool Translate(const vnl_vector<double> &translation);

	/** Apply an integer-grid translation, these can be applied very quickly to image representations and avoid recalculating these */
	bool IntegerGridTranslate(const vnl_vector<int> &translation);

	/** \param scaling to apply 
	* \param params is a vector of object parameters 
	* The function modifies these input parameters such that the new parameters correspond to the scaled object
	* \warning: no check are perform to verify the validity of the inputs
	*/
	void ApplyScalingToParameters(double scaleFactor, vnl_vector<double> &params) { 
		vnl_vector<double> poseParams = params.extract( m_transform->GetNumberOfParameters(), 0 );
		m_transform->ApplyScalingToParameters(scaleFactor, poseParams);
		for (unsigned i=0 ; i<poseParams.size() ; i++) params(i) = poseParams(i);
	}
	/** \param rotation matrix to apply (pre-compose: rotate the object around its center)
	* \param params is a vector of object parameters 
	* The function modifies these input parameters such that the new parameters correspond to the rotated object
	* \warning: no check are perform to verify the validity of the inputs
	*/
	void ApplyRotationToParameters(vnl_matrix<double> rot, vnl_vector<double> &params) { 
		vnl_vector<double> poseParams = params.extract( m_transform->GetNumberOfParameters(), 0 );
		m_transform->ApplyRotationToParameters(rot, poseParams); 
		for (unsigned i=0 ; i<poseParams.size() ; i++) params(i) = poseParams(i);
	}
	
	
	/** Set Resolution parameters for the VTK mesh representation 
	* Invalidates the mesh-based representations if these changes, as well as the pixel-based representation deriving from a mesh
	*/
	bool SetVTKResolution(const vnl_vector<double> &res) {
		if (!m_shape->SetVTKResolution(res)) return false;
		for (unsigned i=0 ; i<m_vtkResolution.size() ; i++) {
			if ( fabs(m_vtkResolution(i) - res(i))>TINY ) { Modified(); break; }
		}
		m_transform->SetVTKResolution(res);
		m_vtkResolution = res;
		return true;
	}

	/** Set the image spacing 
	* Invalidates the current pixel-based representations if the spacing changes
	*/
	void SetImageSpacing(const vnl_vector<double> &sp) {
		m_shape->SetImageSpacing(sp);
		m_transform->SetImageSpacing(sp);
		m_imageSpacing.SetVnlVector( sp );
		m_imageBBoxUpToDate = m_transform->IsObjectImageBoundingBoxUpToDate();
		if (!m_imageBBoxUpToDate) {m_uptodateBinaryImage = false; m_uptodateLabelMap = false;}
	}
	void SetImageSpacing(typename BinaryImageType::SpacingType sp) {
		m_shape->SetImageSpacing(sp);
		m_transform->SetImageSpacing(sp);
		m_imageSpacing = sp;
		m_imageBBoxUpToDate = m_transform->IsObjectImageBoundingBoxUpToDate();
		if (!m_imageBBoxUpToDate) {m_uptodateBinaryImage = false; m_uptodateLabelMap = false;}
	}
	void SetImageSpacing(double sp)	{ 
		m_shape->SetImageSpacing(sp);
		m_transform->SetImageSpacing(sp);
		m_imageSpacing.Fill(sp);
		m_imageBBoxUpToDate = m_transform->IsObjectImageBoundingBoxUpToDate();
		if (!m_imageBBoxUpToDate) {m_uptodateBinaryImage = false; m_uptodateLabelMap = false;}
	}



	/** Get the physical, axis-aligned bounding box
	* The format is [d1_min d1_max d2_min d2_max ...]
	*/
	inline const vnl_vector<double>& GetPhysicalBoundingBox() { 
		m_physicalBoundingBox = m_transform->GetPhysicalBoundingBox();
		m_physicalBBoxUpToDate = true;
		return m_physicalBoundingBox;
	}

	/** Get the vtkPolyData representation of the object
	* Asks the transform for it 
	*/
	inline vtkPolyData* GetObjectAsVTKPolyData() {
		m_outputPolyData = m_transform->GetObjectAsVTKPolyData();
		return m_outputPolyData.GetPointer();
	}


protected:
	PoseTransformedBinaryShape();
	virtual ~PoseTransformedBinaryShape() {};

	bool m_inputFlag;
	typename ShapeType::Pointer m_shape;
	typename TransformType::Pointer m_transform;

	// Apply an integer-grid translation, these can be applied very quickly to image representations and avoid recalculating these 
	bool _IntegerGridTranslate_NoCheck(const vnl_vector<int> &translation) {
		bool zeroTranslation = true;
		double tmp;
		for (unsigned i=0 ; i<Dimension ; i++) {
			if ( translation(i)!=0 ) {
				zeroTranslation = false;
				tmp = translation(i)*m_imageSpacing[i]; 
				m_parameters(i) += tmp; 
				if (m_centerFlag) m_center(i) += tmp;
				if (m_imageBBoxUpToDate) { m_imageOrigin[i] += tmp; m_imageBoundingBox(2*i) += tmp; m_imageBoundingBox(2*i+1) += tmp; }
				if (m_physicalBBoxUpToDate) { m_physicalBoundingBox(2*i) += tmp; m_physicalBoundingBox(2*i+1) += tmp; }
			}
		}

		if (m_uptodateBinaryImage)   { 
			if (!m_imageBBoxUpToDate) throw DeformableModelException("PoseTransformedBinaryShape::_IntegerGridTranslate_NoCheck : m_imageBBoxUpToDate should always be uptodate when m_uptodateBinaryImage is ; this is not the case");; 
			m_outputBinaryImage->SetOrigin(m_imageOrigin); 
		}
		if (m_uptodateLabelMap)      {
			if (!m_imageBBoxUpToDate) throw DeformableModelException("PoseTransformedBinaryShape::_IntegerGridTranslate_NoCheck : m_imageBBoxUpToDate should always be uptodate when m_uptodateLabelMap is ; this is not the case");; 
			m_outputLabelMap->SetOrigin(m_imageOrigin);
		}

		if (!zeroTranslation) {
			m_uptodatePolyData = false;
			m_transform->_IntegerGridTranslate_NoCheck(translation);
		}

		return true;
	}

	/** Collect Parameters from the shape and transform, in the following order: pose then shape parameters */
	vnl_vector<double> CollectParameters();

	//simply transforms the elements provided by the shape
	virtual void ComputeObjectCenter();
	//virtual void ComputeObjectInertia();
	//virtual void ComputeObjectInertiaEigenVectors(); //in the general case of an affine transform, I need to recover the purely rotational component of the matrix...

private:
	PoseTransformedBinaryShape(const Self&); //purposely not implemented
	const Self & operator=( const Self & );  //purposely not implemented
};


} // namespace psciob

#include "PoseTransformedBinaryShape.txx"

#endif /* POSETRANSFORMEDBINARYSHAPE_H_ */


