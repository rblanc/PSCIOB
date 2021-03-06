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
 * \file SceneFlatPrior.h
 * \author R�mi Blanc 
 * \date 27. October 2011
 */

#ifndef SCENEFLATPRIOR_H_
#define SCENEFLATPRIOR_H_

#include "SceneGlobalPrior_Base.h"

namespace psciob {

/**\class SceneFlatPrior 
 * \brief SceneFlatPrior
 * just a default, informationless - returns a constant whatever the scene contents - 
*/



//CONCRETE CLASS
template<class TScene>
class SceneFlatPrior : public SceneGlobalPrior_Base<TScene> {
public:
	/** Standard class typedefs. */
	typedef SceneFlatPrior					Self;
	typedef SceneGlobalPrior_Base<TScene>	Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SceneFlatPrior, SceneGlobalPrior_Base);
	itkNewMacro(Self);


	/** This function should only be called by the scene - use friendship instead of leaving the method public*/
	inline void AddObject(ObjectInScene *objectPtr) {}

	/** This function should only be called by the scene - use friendship instead of leaving the method public*/
	inline void RemoveObject(ObjectInScene *objectPtr) {}

	/** This function should only be called by the scene - use friendship instead of leaving the method public*/
	inline void ModifyObjectParameters(ObjectInScene *objectPtr, ObjectInScene *newObject) {}


protected:	
	SceneFlatPrior() : SceneGlobalPrior_Base() {}
	virtual ~SceneFlatPrior() {};	

	inline double ComputeGlobalPrior_Internal()	{ return 0; }

private:
	SceneFlatPrior(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};



} // namespace psciob

#endif /* SCENEFLATPRIOR_H_ */
