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

/*
 * RandomVariables.h
 * \author Rémi Blanc 
 * \date 28. February 2012
*/


#ifndef RANDOMVARIABLES_H_
#define RANDOMVARIABLES_H_


#include <itkSmartPointer.h>
#include <itkMersenneTwisterRandomVariateGenerator.h>
#include "GeneralUtils.h"

namespace psciob {


/** basically wrap the itk::MersenneTwisterRandomVariateGenerator, adding a couple of functionalities... removing some...
 * By default, new instances are initialized with the same seed.
*/
class RandomVariableGenerator : public itk::LightObject {
public:
	/** Standard class typedefs. */
	typedef RandomVariableGenerator       Self;
	typedef itk::LightObject              Superclass;
	typedef itk::SmartPointer<Self>       Pointer;
	typedef itk::SmartPointer<const Self> ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(RandomVariableGenerator,itk::LightObject);
	itkNewMacro(Self);

	RandomVariableGenerator();

	/** Initialize based on the clock time */
	void Initialize();
	/** Initialize on a fixed seed */
	void Initialize(int seed);

	/** integer in [0 n] n<2^32*/
	inline long GetIntegerVariate(long n) {return m_baseGenerator->GetIntegerVariate(n); }

	/** uniform variable in [min max] */
	inline double GetUniformVariate(double min=0.0, double max=1.0) { return m_baseGenerator->GetUniformVariate(min, max);	}
	
	/** gaussian variable, with specified mean and variance */
	inline double GetNormalVariate(double mean=0.0, double var=1.0) { return m_baseGenerator->GetNormalVariate(mean, var); }
	
	/** lognormale variable, with specified parameters */
	inline double GetLogNormalVariate(double mean=0.0, double std=1.0) { return exp(mean+std*m_baseGenerator->GetNormalVariate(0,1)); }
	
	/** triangular variable, in the interval [a b], the mode is at c */
	inline double GetTriangularVariate(double a=-1.0, double c=0.0, double b=1.0) {
		double tmp = m_baseGenerator->GetUniformVariate(0, 1);
		if (tmp<(c-a)/(b-a))	return ( a+sqrt(  tmp  *(b-a)*(c-a)) );
		else					return ( b-sqrt((1-tmp)*(b-a)*(c-a)) );
	}

protected:
	typedef itk::Statistics::MersenneTwisterRandomVariateGenerator GeneratorType;	
	GeneratorType::Pointer m_baseGenerator;
};

} // namespace psciob


#endif //RANDOMVARIABLES_H_