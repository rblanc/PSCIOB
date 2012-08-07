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
 * \file Translation3DTransform.h
 * \author Rémi Blanc 
 * \date 28. July 2011
*/

#ifndef TRANSLATION3DTRANSFORM_H_
#define TRANSLATION3DTRANSFORM_H_

#include "Abstract3DTransform.h"

namespace psciob {

/** \class Translation3DTransform
 * \brief Translation3DTransform: 3 translation parameters
*/


class Translation3DTransform : public Abstract3DTransform {
public:
	/** Standard class typedefs. */
	typedef Translation3DTransform			Self;
	typedef Abstract3DTransform				Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(Translation3DTransform,Abstract3DTransform);
	itkNewMacro(Self);

	/** Give Information about Self */
	inline std::string GetClassName()		const	{return m_name;}
	void PrintInfo() const { std::cout<<"Translation3DTransform with parameters: "<<m_parameters<<std::endl; }

	/** Get Number of parameters */
	inline unsigned int GetNumberOfParameters() const {return m_nbParams;}

	/** Get Parameters */
	vnl_vector<double> GetDefaultParameters() const ;
	inline vnl_matrix<double> GetMatrixFromParameters(const vnl_vector<double> &poseParameters) const ;
	inline vnl_vector<double> GetParametersFromMatrix(const vnl_matrix<double> &transformMatrix) const ;

	/** Check Validity of the parameters, returns false if parameters are not valid */
	inline bool CheckParameters(const vnl_vector<double> &p) const {
		if ( p.size() != m_nbParams ) return false;
		return true;
	}


	bool RotateAroundX(double angle) {return false;}
	bool RotateAroundY(double angle) {return false;}
	bool RotateAroundZ(double angle) {return false;}
	bool RotateAroundAxis(double angle, const vnl_vector<double> &axis){return false;}

	//no scaling or rotation for this transform
	bool Scale(double scale) {return false;}
	bool Scale(vnl_vector<double> scale) {return false;}
	void ApplyScalingToParameters(double scaleFactor, vnl_vector<double> &params) {}
	void ApplyRotationToParameters(vnl_matrix<double> rot, vnl_vector<double> &params) {}


protected:
	Translation3DTransform();
	virtual ~Translation3DTransform() {};

	static const std::string m_name;
	static const unsigned int m_nbParams = 3; //3 translations

private:
	Translation3DTransform(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};



} // namespace psciob

#endif /* TRANSLATION3DTRANSFORM_H_ */
