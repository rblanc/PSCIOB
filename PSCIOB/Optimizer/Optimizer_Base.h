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
 * \file Optimizer_Base.h
 * \author Rémi Blanc 
 * \date 22. September 2011
 */



#ifndef OPTIMIZER_BASE_H_
#define OPTIMIZER_BASE_H_

//#include "CommonTypes.h"
#include "GeneralUtils.h"

namespace psciob {

/**\class Optimizer_Base 
 * \brief Optimizer_Base
 * Base class for optimizers - not very useful in itself... 
 * TODO? change the interface to SET the optimization manager, instead of calling ->optimize(manager) ?
*/

class OptimizationManager_Base;

class Optimizer_Base : public itk::LightObject {
public:
	/** Standard class typedefs. */
	typedef Optimizer_Base				Self;
	typedef itk::LightObject			Superclass;
	typedef itk::SmartPointer<Self>		Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(Optimizer_Base,itk::LightObject);

	virtual double Optimize(OptimizationManager_Base *manager) = 0;

protected:
	Optimizer_Base() {};
	virtual ~Optimizer_Base() {};

	unsigned int m_nbParameters;
private:
	Optimizer_Base(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};



} // namespace psciob

#endif /* OPTIMIZATION_BASE_H_ */
