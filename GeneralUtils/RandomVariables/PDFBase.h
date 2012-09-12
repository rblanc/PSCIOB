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
 * \file PDFBase.h
 * \author Rémi Blanc 
 * \date 26. September 2011
*/

#ifndef PDFBASE_H_
#define PDFBASE_H_

#include "RandomVariables.h"

namespace psciob {


/** \class PDFBase
* \brief Base class for Probability Density Functions
* each instance has its own random variable generator, initialized by default on the 0 seed
* \sa RandomVariables
*/ 

template<class TSampleType>
class PDFBase : public itk::LightObject {
public: 
	typedef PDFBase								Self;
	typedef typename itk::LightObject			Superclass;
	typedef typename itk::SmartPointer<Self>	Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(PDFBase, itk::LightObject);

	/** Initialize the internal random variable generator against the clock time */
	virtual void Initialize();
	/** Initialize the internal random variable generator against a given seed */
	virtual void Initialize(int seed);

	/** Get the likelihood of the sample p(x), where p is the probability density function */
	virtual double GetLikelihood(const TSampleType &x) = 0;

	/** Get the maximum value the likelihood can take - may be useful for normalization purposes */
	virtual double GetMaximumLikelihoodValue() = 0;

	/** Get the loglikelihood of the sample log( p(x) ), where p is the probability density function */
	virtual double GetLogLikelihood(const TSampleType &x) = 0;

	/** Get the maximum value the loglikelihood can take - may be useful for normalization purposes */
	//OPTIMIZATION: implement the function in the child classes instead...?
	inline double GetMaximumLogLikelihoodValue() { return log(GetMaximumLikelihoodValue()); }

	/** Draw a random sample */
	inline TSampleType DrawSample() { m_lastSample = _DrawNewSample(); return m_lastSample; }

	/** Get the last generated sample */
	inline TSampleType GetLastSample() {return m_lastSample;}

	/** Dimensionality of the generated variables */
	virtual unsigned int GetNumberOfDimensions() = 0;

	/** Set generator seed */
	void SetGeneratorSeed(int seed) { m_baseGenerator->Initialize(seed); }

protected:
	PDFBase();
	virtual ~PDFBase() {};

	typedef itk::Statistics::MersenneTwisterRandomVariateGenerator GeneratorType;	
	GeneratorType::Pointer m_baseGenerator;
	TSampleType m_lastSample;

	virtual TSampleType _DrawNewSample() = 0;

private:
	PDFBase(const Self&);					//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};

} // namespace psciob

#include "PDFBase.txx"

#endif //PDFBASE_H_