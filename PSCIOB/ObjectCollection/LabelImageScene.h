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
 * \file LabelImageScene.h
 * \author Rémi Blanc 
 * \date 29. February 2012
 */

#ifndef LABELIMAGESCENE_H_
#define LABELIMAGESCENE_H_

#include "BaseScene.h"
#include "ObjectPixelSet.h"
#include "PixelSetUtils.h"
#include "LabelImageObjectOverlap.h"

namespace psciob {

/** \class LabelImageScene
 * \brief LabelImageScene
 * 
 * The class maitains at all time a volume of voxels (LabelImage)
 * The main objective is to handle object intersections efficiently.
 *
 * \IMPORTANT: this class requires data structures that store pixel sets, at least containing the fields present in the default structures ObjectCostsAndPixelSetContainer and PixelSetIntersectionContainer
 *             and automatically manages interactions: it is configured to use a LabelImageObjectOverlap as interactionManager, which should not probably never be changed!
 *
*/


//CONCRETE CLASS
template <unsigned int VDimension, class TAppearance = unsigned char, class TObjectId = unsigned short, class TAssociatedData = ObjectCostsContainer, class TInteractionData = typename LabelObjectIntersectionContainer<TObjectId, VDimension>>
class LabelImageScene : public BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData> {
public:
	/** Standard class typedefs. */
	typedef LabelImageScene               Self;
	typedef BaseScene                     Superclass;
	typedef itk::SmartPointer<Self>       Pointer;
	typedef itk::SmartPointer<const Self> ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(LabelImageScene, BaseScene);
	itkNewMacro(Self);

	/** Give Information about Self */
	std::string GetClassName() const { return "LabelImageScene"; }
	void PrintInfo()           const { std::cout<<"LabelImageScene containing "<<GetNumberOfObjects()<<", for a total of "<<GetNumberOfParameters()<<std::endl;}

	/** Set physical bounding Box and pixel spacing
	* the actual bounding box will be slightly modify, to ensure that the origin will be at the center of a pixel (even if outside the actual grid)
	* overloads the base implementation, and allocate memory for the volume
	*/
	using Superclass::SetPhysicalDimensions;
	void SetPhysicalDimensions(const vnl_vector<double> &physicalBoundingBox, const vnl_vector<double> &spacing);

protected:	
	LabelImageScene();
	virtual ~LabelImageScene() {};	

	//Test methods
	SceneObjectOverlapCode TestOverlap_Internal(ObjectInScene *object);
	SceneObjectOverlapCode TestOverlapIgnoringObjectID_Internal(ObjectInScene *object, IDType id);
	bool TestObjectFullyOutside_Internal(ObjectInScene *object); 

	//Draw methods
	void DrawObjectInScene(IDType id);
	void EraseObjectFromScene(IDType id);
	void UpdateObjectInScene(IDType id, ObjectInScene *newObject);

	typename LabelMapType::Pointer m_dummyLabelMap;

	typename LabelObjectType::Pointer m_removedPixels, m_addedPixels, m_redrawPixels;

private:
	LabelImageScene(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};

} // namespace psciob


#include "LabelImageScene.txx"

#endif /* LABELIMAGESCENE_H_ */
