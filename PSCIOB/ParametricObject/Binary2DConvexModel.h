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
 * \file Binary2DConvexModel.h
 * \author Rémi Blanc 
 * \date 1. December 2011
*/

#ifndef BINARY2DCONVEXMODEL_H_
#define BINARY2DCONVEXMODEL_H_

#include "BinaryDeformableModel.h"

namespace psciob {


/** 
 * \class Binary2DConvexModel
 * \brief Binary2DConvexModel is a base class for binary convex deformable models
 *
 * Binary2DConvexModel is an abstract object, for fast drawnable objects
 * using a scan line algorithm
*/


//ABSTRACT CLASS
class Binary2DConvexModel : public BinaryDeformableModel<2> {
public:
	/** Standard class typedefs. */
	typedef Binary2DConvexModel				Self;
	typedef BinaryDeformableModel			Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(Binary2DConvexModel,BinaryDeformableModel);

	/** Get the set of pixel occupied by the object, at the given spacing and with respect to the tightest bounding box containing the object 
	 * Use a code specific to 2D convex polygons, requesting a list of points ordered so that each point is connected to the next (+wraparound)
	 * It is a bit more restritive in its input: a single closed contour, with points given in a connectedness order
	 * but maybe faster
	*/
	LabelMapType* GetObjectAsLabelMap() {
		if (!m_uptodateLabelMap) {
			//instanciate the LabelMap if necessary
			if(!m_outputLabelMap) m_outputLabelMap = LabelMapType::New();
			//update the LabelMap
			VTK2DConvexPolyDataToLabelMap<LabelMapType>( GetObjectAsVTKPolyData(), m_outputLabelMap, m_imageSpacing );
			m_uptodateLabelMap=true;
			m_imageBBoxUpToDate=true;
			m_imageOrigin = m_outputLabelMap->GetOrigin();
			m_imageRegion = m_outputLabelMap->GetLargestPossibleRegion();
		}
		return m_outputLabelMap.GetPointer();
	}

protected:
	Binary2DConvexModel() : BinaryDeformableModel() {}
	virtual ~Binary2DConvexModel() {};

private:
	Binary2DConvexModel(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};


} // namespace psciob

#endif /* BINARY2DCONVEXMODEL_H_ */


