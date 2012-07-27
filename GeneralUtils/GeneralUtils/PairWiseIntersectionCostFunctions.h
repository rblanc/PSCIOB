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
 * \file PairWiseIntersectionCostFunctions.h
 * \author Rémi Blanc 
 * \date 10. May 2012
*/

#ifndef PAIRWISEINTERSECTIONCOSTFUNCTIONS_H_
#define PAIRWISEINTERSECTIONCOSTFUNCTIONS_H_

#include "GeneralUtils.h"
#include "GenericParametricFunctions.h"
#include <math.h> 

namespace psciob {

//TODO create a .txx file to make this file more readable... + document the classes in here.

//TODO: try to enforce default parameters in the constructor ; if not, check paramsSet in the ->Evaluate(x) method

/** \class PairWiseIntersectionCostFunction
* \brief PairWiseIntersectionCostFunction base class for cost functions relative to pairwise intersection of objects
* these functions need 3 input: the volume of set1 and set2, and the volume of their intersection.
*/
class PairWiseIntersectionCostFunction : public itk::LightObject {
public: 
	typedef PairWiseIntersectionCostFunction Self;
	typedef itk::LightObject                 Superclass;
	typedef itk::SmartPointer<Self>          Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(PairWiseIntersectionCostFunction, itk::LightObject);

	virtual double Evaluate( double set1, double set2, double setInter) = 0;

protected:
	PairWiseIntersectionCostFunction() {}
	virtual ~PairWiseIntersectionCostFunction() {}
private:
	PairWiseIntersectionCostFunction(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};


/** \class PairWiseOverlapDiceCoefficient
* \brief PairWiseOverlapDiceCoefficient: computes the Dice coefficient = 2*|V1 inter V2| / (|V1|+|V2|)
*/
class PairWiseOverlapDiceCoefficient : public PairWiseIntersectionCostFunction {
public: 
	typedef PairWiseOverlapDiceCoefficient   Self;
	typedef PairWiseIntersectionCostFunction Superclass;
	typedef itk::SmartPointer<Self>          Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(PairWiseOverlapDiceCoefficient, PairWiseIntersectionCostFunction);
	itkNewMacro(PairWiseOverlapDiceCoefficient);

	inline double Evaluate( double set1, double set2, double setInter) { return (2.0*setInter)/(set1+set2); }

protected:
	PairWiseOverlapDiceCoefficient() {}
	virtual ~PairWiseOverlapDiceCoefficient() {};
private:
	PairWiseOverlapDiceCoefficient(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};

/** \class PairWiseOverlapJaccardIndex
* \brief PairWiseOverlapJaccardIndex: computes the Jaccard index = |V1 inter V2|/|V1 union V2|
*/
class PairWiseOverlapJaccardIndex : public PairWiseIntersectionCostFunction {
public: 
	typedef PairWiseOverlapJaccardIndex      Self;
	typedef PairWiseIntersectionCostFunction Superclass;
	typedef itk::SmartPointer<Self>          Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(PairWiseOverlapJaccardIndex, PairWiseIntersectionCostFunction);
	itkNewMacro(PairWiseOverlapJaccardIndex);

	inline double Evaluate( double set1, double set2, double setInter) { return (setInter)/(set1+set2-setInter); }

protected:
	PairWiseOverlapJaccardIndex() {}
	virtual ~PairWiseOverlapJaccardIndex() {};
private:
	PairWiseOverlapJaccardIndex(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};

/** \class PairWiseOverlapCoefficient
* \brief PairWiseOverlapCoefficient: computes the overlap coefficient = |V1 inter V2|/min(|V1|,|V2|)
*/
class PairWiseOverlapCoefficient : public PairWiseIntersectionCostFunction {
public: 
	typedef PairWiseOverlapCoefficient      Self;
	typedef PairWiseIntersectionCostFunction Superclass;
	typedef itk::SmartPointer<Self>          Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(PairWiseOverlapCoefficient, PairWiseIntersectionCostFunction);
	itkNewMacro(PairWiseOverlapCoefficient);

	inline double Evaluate( double set1, double set2, double setInter) { return (setInter)/std::min(set1,set2); }

protected:
	PairWiseOverlapCoefficient() {}
	virtual ~PairWiseOverlapCoefficient() {};
private:
	PairWiseOverlapCoefficient(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};

/** \class TransformedPairWiseOverlapMeasure
* \brief TransformedPairWiseOverlapMeasure: applies a parametric scalar function to a PairWiseIntersectionCostFunction
*/
template<class TParametricFunction, class TPairWiseIntersectionCostFunction>
class TransformedPairWiseOverlapMeasure : public PairWiseIntersectionCostFunction {
public: 
	typedef TransformedPairWiseOverlapMeasure<TParametricFunction, TPairWiseIntersectionCostFunction> Self;
	typedef PairWiseIntersectionCostFunction  Superclass;
	typedef itk::SmartPointer<Self>           Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(TransformedPairWiseOverlapMeasure, PairWiseIntersectionCostFunction);
	itkNewMacro(TransformedPairWiseOverlapMeasure);

	/** Set the parameters to the parametric function */
	bool SetParameters(vnl_vector<double> p) { return m_parametricFunction->SetParameters(p); }

	/** Computes the overlap measure, and transforms it using the given parametric function */
	inline double Evaluate( double set1, double set2, double setInter) { 
		//std::cout<<"set1: "<<set1<<", set2: "<<set2<<", inter: "<<setInter<<", overlap score = "<<m_overlapMeasure->Evaluate(set1, set2, setInter)<<", value = "<<m_parametricFunction->Evaluate(m_overlapMeasure->Evaluate(set1, set2, setInter))<<" ; parameters = "<<m_parametricFunction->GetParameters()<<std::endl;
		return m_parametricFunction->Evaluate(m_overlapMeasure->Evaluate(set1, set2, setInter)); 
	}

protected:
	TransformedPairWiseOverlapMeasure() {
		m_parametricFunction = TParametricFunction::New();
		m_overlapMeasure = TPairWiseIntersectionCostFunction::New();
	}
	virtual ~TransformedPairWiseOverlapMeasure() {};

	typename TParametricFunction::Pointer m_parametricFunction;
	typename TPairWiseIntersectionCostFunction::Pointer m_overlapMeasure;

private:
	TransformedPairWiseOverlapMeasure(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};

} // namespace psciob

#endif //PAIRWISEINTERSECTIONCOSTFUNCTIONS_H_