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
 * \file LabelMapScene.h
 * \author Rémi Blanc 
 * \date 29. February 2012
 */

#ifndef LABELMAPSCENE_H_
#define LABELMAPSCENE_H_

#include "BaseScene.h"
#include "ObjectPixelSet.h"
#include "PixelSetUtils.h"

namespace psciob {

/** \class LabelMapScene
 * \brief LabelMapScene
 * 
 * This scene uses the itkLabelObject structure (i.e. a run-length encoded object) to physically represent the objects
 * Internally, when adding an object, it asks the ParametricObject to update its LabelObject representation, and simply cast it into the scene frame
*/


//CONCRETE CLASS
template <unsigned int VDimension, class TAppearance = unsigned char, class TObjectId = unsigned short, class TAssociatedData = ObjectCostsContainer, class TInteractionData = typename LabelObjectIntersectionContainer<TObjectId, VDimension>>
class LabelMapScene : public BaseScene<VDimension, TAppearance, TObjectId, TAssociatedData, TInteractionData> {
public:
	/** Standard class typedefs. */
	typedef LabelMapScene                 Self;
	typedef BaseScene                     Superclass;
	typedef itk::SmartPointer<Self>       Pointer;
	typedef itk::SmartPointer<const Self> ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(LabelMapScene, BaseScene);
	itkNewMacro(Self);

	/** Give Information about Self */
	std::string GetClassName() const { return "LabelMapScene"; }
	void PrintInfo()           const { std::cout<<"LabelMapScene containing "<<GetNumberOfObjects()<<", for a total of "<<GetNumberOfParameters()<<std::endl;}

protected:	
	LabelMapScene();
	virtual ~LabelMapScene() {};	

	void AllocateFromPhysicalDimension();

	//Test methods
	SceneObjectOverlapCode TestOverlap_Internal(ObjectInScene *object);
	SceneObjectOverlapCode TestOverlapIgnoringObjectID_Internal(ObjectInScene *object, IDType id);
	bool TestObjectFullyOutside_Internal(ObjectInScene *object); 

	//Draw methods
	void DrawObjectInScene(IDType id);
	void EraseObjectFromScene(IDType id);
	void UpdateObjectInScene(IDType id, ObjectInScene *newObject);

	typename LabelMapType::Pointer m_dummyLabelMap;
private:
	LabelMapScene(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};


} // namespace psciob

#include "LabelMapScene.txx"

#endif /* LABELMAPSCENE_H_ */
