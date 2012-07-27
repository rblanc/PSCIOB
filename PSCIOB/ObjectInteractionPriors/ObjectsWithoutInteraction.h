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
* \file ObjectsWithoutInteraction.h
* \author R�mi Blanc 
* \date 21. October 2011
*/

#ifndef __OBJECTSWITHOUTINTERACTION_H_
#define __OBJECTSWITHOUTINTERACTION_H_

#include "ObjectInteractionManager.h"

namespace psciob {


/**\class ObjectsWithoutInteraction
* \brief ObjectsWithoutInteraction: Default - objects are never interacting! 
*/

//CONCRETE CLASS
template<class TScene>
class ObjectsWithoutInteraction : public ObjectInteractionManager<TScene> {
public:
	/** Standard class typedefs. */
	typedef ObjectsWithoutInteraction			Self;
	typedef ObjectInteractionManager			Superclass;
	typedef itk::SmartPointer<Self>				Pointer;
	typedef itk::SmartPointer<const Self>		ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(ObjectsWithoutInteraction, ObjectInteractionManager);
	itkNewMacro(ObjectsWithoutInteraction);


	/** NO INTERACTION */
	inline void AddObject(ObjectInScene *objectPtr) {}

	/** NO INTERACTION */
	inline void RemoveObject(ObjectInScene *objectPtr) {}

	/** NO INTERACTION */
	inline void ModifyObjectParameters(ObjectInScene *objectPtr, ObjectInScene *newObject) {}

	/** NO INTERACTION */
	inline InteractionDataType GetPairWiseInteractionData(ObjectInScene *object1, ObjectInScene *object2) { return m_emptyInteraction; }

	/** NO INTERACTION */
	inline std::vector<IDType> IdentifyInteractingObjectsInScene(DeformableObjectType *object, IDType ignoredLabel = 0) { return std::vector<IDType>(); }

	/** NO INTERACTION */
	inline void ComputePairWiseObjectInteractionData(DeformableObjectType *object1, DeformableObjectType *object2, InteractionDataType &interactionData) {}
	/** NO INTERACTION */
	inline void ComputePairWiseObjectInteractionData(ObjectInScene *object1, ObjectInScene *object2, InteractionDataType &interactionData) {}

protected:	
	ObjectsWithoutInteraction() : ObjectInteractionManager() {};
	virtual ~ObjectsWithoutInteraction() {};	

private:
	ObjectsWithoutInteraction(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};

} // namespace psciob

#endif /* __OBJECTSWITHOUTINTERACTION_H_ */
