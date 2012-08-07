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
 * \file TranslationScale2DTransform.h
 * \author Rémi Blanc 
 * \date 7. August 2012
*/

#ifndef TRANSLATIONSCALE2DTRANSFORM_H_
#define TRANSLATIONSCALE2DTRANSFORM_H_

#include "Abstract2DTransform.h"

namespace psciob {

/** \class TranslationScale2DTransform
*\brief TranslationScale2DTransform: 2 translation parameters + 1 scale parameter
*
*/


class TranslationScale2DTransform : public Abstract2DTransform {
public:
	/** Standard class typedefs. */
	typedef TranslationScale2DTransform			Self;
	typedef Abstract2DTransform				Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(TranslationScale2DTransform,Abstract2DTransform);
	itkNewMacro(Self);

	/** Give Information about Self */
	inline std::string GetClassName()		const	{return m_name;}
	void PrintInfo() const { std::cout<<"TranslationScale2DTransform with parameters: "<<m_parameters<<std::endl; }

	/** Get Number of parameters */
	inline unsigned int GetNumberOfParameters() const {return m_nbParams;}

	/** Get Parameters */
	vnl_vector<double> GetDefaultParameters() const ;
	inline vnl_matrix<double> GetMatrixFromParameters(const vnl_vector<double> &poseParameters) const ;
	inline vnl_vector<double> GetParametersFromMatrix(const vnl_matrix<double> &transformMatrix) const ;

	/** Check Validity of the parameters, returns false if parameters are not valid */
	inline bool CheckParameters(const vnl_vector<double> &p) const {
		if ( p.size() != m_nbParams ) return false;
		if (p(2)<TINY) return false; //scale must be >0
		return true;
	}

	bool Scale(vnl_vector<double> scale) {
		if ( (scale(0)==scale(1)) && (scale(0)==scale(2)) ) return PoseTransform::Scale(scale(0));
		else return false;
	}

	//no rotation for this transform
	void ApplyRotationToParameters(vnl_matrix<double> rot, vnl_vector<double> &params) {}
	
protected:
	TranslationScale2DTransform();
	virtual ~TranslationScale2DTransform() {};

	static const std::string m_name;
	static const unsigned int m_nbParams = 3; //2 translations

private:
	TranslationScale2DTransform(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};



} // namespace psciob

#endif /* TRANSLATIONSCALE2DTRANSFORM_H_ */
