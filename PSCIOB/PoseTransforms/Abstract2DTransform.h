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
 * \file Abstract2DTransform.h
 * \author Rémi Blanc 
 * \date 27. July 2011
*/

#ifndef ABSTRACT2DTRANSFORM_H_
#define ABSTRACT2DTRANSFORM_H_

#include "PoseTransform.h"
#include "2DTransformUtils.h"

namespace psciob {

/** \class Abstract2DTransform
 * \brief Abstract2DTransform: abstract class for a 2D transfom, inheriting from PoseTransform ; not very useful 
 */


class Abstract2DTransform : public PoseTransform<2> {
public:
	/** Standard class typedefs. */
	typedef Abstract2DTransform				Self;
	typedef PoseTransform					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(Abstract2DTransform,PoseTransform);

	//WARNING: verify that this is an authorized transformation (overload it in the actual implementation, raising a "unauthorized transformation" exception)
	bool Rotate(double angle) {
		vnl_matrix<double> mat(m_nbDimensions+1, m_nbDimensions+1);
		mat.set_identity();
		mat.put(0,0, cos(angle));	mat.put(0,1,-sin(angle));
		mat.put(1,0, sin(angle));	mat.put(1,1, cos(angle));
		return Compose(mat);
	}

protected:
	Abstract2DTransform() : PoseTransform() {}
	virtual ~Abstract2DTransform() {};

	static const unsigned int m_nbDimensions = 2;

private:
	Abstract2DTransform(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented

};

inline vnl_matrix<double> GetRotationMatrixFromAngle(double angle) {
	vnl_matrix<double> rotmat(2,2);
	rotmat.put(0,0, cos(angle));	rotmat.put(0,1,-sin(angle));
	rotmat.put(1,0, sin(angle));	rotmat.put(1,1, cos(angle));

	return rotmat;
}

inline double GetAngleFromRotationMatrix(vnl_matrix<double> mat) {
	return atan2(mat(1,0),mat(0,0));
}


} // namespace psciob

#endif /* ABSTRACT2DTRANSFORM_H_ */
