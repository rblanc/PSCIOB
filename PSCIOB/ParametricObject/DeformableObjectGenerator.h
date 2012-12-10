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
 * \file DeformableObjectGenerator.h
 * \author Rémi Blanc 
 * \date 26. September 2011
*/

#ifndef DEFORMABLEOBJECTGENERATOR_H_
#define DEFORMABLEOBJECTGENERATOR_H_

//#include "CommonTypes.h"
#include "MultivariatePDF.h"
#include "ParametricObject.h"
//#include "ShapeAndPoseDeformableObject.h"
//#include "Binary2DConvexModel.h"

#include <typeinfo>


//IDEA: augment the functionalities of this class, so that it can also serve as a random parameter generator for the object
//default mode is still a prior distribution on the object parameters
//but other modes can be the generation of propositions for random modifications of the object parameters, etc...
//-> this is transfered to the class ObjectTypesLibrary, which maintains a collection of pdfs for each object type.
//...??? or use the generator instead to maintain such collections... create new objects, generate random parameters, etc...

namespace psciob {


/** 
 * \class DeformableObjectGenerator
 * \brief DeformableObjectGenerator: a class able to generate new instances of a given object type
 *
 * Can associate a multivariate random distribution and a concrete implementation of ParametricObject
 * To sample, and generate random instances of the requested object
 * in pratice, it is rather used through the ObjectTypesLibrary class
 *
 */


template <class TDeformableObjectType>
class DeformableObjectGenerator : public itk::LightObject {
public:
	/** Standard class typedefs. */
	typedef DeformableObjectGenerator		Self;
	typedef itk::LightObject				Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(DeformableObjectGenerator,itk::LightObject);
	itkNewMacro(Self);

	typedef TDeformableObjectType			DeformableObjectType;
	static const unsigned int Dimension =	DeformableObjectType::Dimension;

	/** Set pointers to parametric shape and transform */
	void SetBaseObject(DeformableObjectType *object) { m_baseObject = object; }

	/** no check is done yet... they will be done in the GetRandomObject() method */
	void SetPrior(MultivariatePDF *prior) {	m_priorIsSet=true; m_pdf = prior; }

	/** Get Prior */
	MultivariatePDF* GetPrior() const { return const_cast<MultivariatePDF*>(m_pdf.GetPointer()); }

	/** Get number of parameters */
	unsigned int GetNumberOfObjectParameters() { return m_baseObject->GetNumberOfParameters(); }

	/** checks if typeid of the input is the same as that of the reference object */
	inline bool CheckTypes(DeformableObjectType *object) {
		return (typeid(*m_baseObject)==typeid(*object));
	}

	/** Generate a new object of the given type */
	DeformableObjectType* GetNewObject() {
		return m_baseObject;
		//DeformableObjectType::Pointer object = static_cast<DeformableObjectType*>(m_baseObject->CreateAnother().GetPointer());
		//if (!object->SetVTKResolution( m_baseObject->GetVTKResolution() )) {
		//	m_baseObject->PrintInfo();
		//}
		//object->SetImageSpacing( m_baseObject->GetImageSpacing() );
		//
		//m_lastGeneratedObject = object;
		//if (m_priorIsSet)	m_lastGeneratedObject->SetParameters(m_pdf->DrawSample());
		//else				m_lastGeneratedObject->SetDefaultParameters();
	
		//return m_lastGeneratedObject.GetPointer();
	}

protected:
	DeformableObjectGenerator() : m_priorIsSet(false) {};
	virtual ~DeformableObjectGenerator() {};

	typename DeformableObjectType::Pointer m_baseObject, m_lastGeneratedObject;
	MultivariatePDF::Pointer m_pdf;
	bool m_priorIsSet;

private:
	DeformableObjectGenerator(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};



} // namespace psciob

#endif /* DEFORMABLEOBJECTGENERATOR_H_ */


