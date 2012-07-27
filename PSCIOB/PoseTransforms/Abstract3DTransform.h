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
 * \file Abstract3DTransform.h
 * \author Rémi Blanc 
 * \date 27. July 2011
*/

#ifndef ABSTRACT3DTRANSFORM_H_
#define ABSTRACT3DTRANSFORM_H_

#include "PoseTransform.h"
#include "3DTransformUtils.h"

namespace psciob {

/** \class Abstract3DTransform
* \brief Abstract3DTransform: abstract class for a 2D transfom, inheriting from PoseTransform ; not very useful 
*/


class Abstract3DTransform : public PoseTransform<3> {
public:
	/** Standard class typedefs. */
	typedef Abstract3DTransform				Self;
	typedef PoseTransform<3>				Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(Abstract3DTransform,PoseTransform);
	//itkNewMacro(Self);

	//WARNING: verify that this is an authorized transformation (overload it in the actual implementation, raising a "unauthorized transformation" exception)
	bool RotateAroundX(double angle) {
		vnl_matrix<double> mat(m_nbDimensions+1, m_nbDimensions+1), rotmat(m_nbDimensions,m_nbDimensions);
		mat.set_identity();
		//mat.put(0,0,   1   );   mat.put(0,1,   0   );		mat.put(0,2,   0   );
		                          mat.put(1,1, cos(angle));	mat.put(1,2,-sin(angle));	
		                          mat.put(2,1, sin(angle));	mat.put(2,2, cos(angle));
		return Compose(mat);
	}

	//WARNING: verify that this is an authorized transformation (overload it in the actual implementation, raising a "unauthorized transformation" exception)
	bool RotateAroundY(double angle) {
		vnl_matrix<double> mat(m_nbDimensions+1, m_nbDimensions+1), rotmat(m_nbDimensions,m_nbDimensions);
		mat.set_identity();
		mat.put(0,0, cos(angle));	                        mat.put(0,2, sin(angle));
		//mat.put(1,0,   0   );		mat.put(1,1,   1   );	mat.put(1,2,   0   );
		mat.put(2,0,-sin(angle));	                        mat.put(2,2, cos(angle));
		return Compose(mat);			
	}

	//WARNING: verify that this is an authorized transformation (overload it in the actual implementation, raising a "unauthorized transformation" exception)
	bool RotateAroundZ(double angle) {
		vnl_matrix<double> mat(m_nbDimensions+1, m_nbDimensions+1), rotmat(m_nbDimensions,m_nbDimensions);
		mat.set_identity();
		mat.put(0,0, cos(angle));	mat.put(0,1,-sin(angle));	//mat.put(0,2,   0   );
		mat.put(1,0, sin(angle));	mat.put(1,1, cos(angle));	//mat.put(1,2,   0   );
		//mat.put(2,0,   0   );		mat.put(2,1,   0   );		mat.put(2,2,   1   );
		return Compose(mat);
	}

	//WARNING: verify that this is an authorized transformation (overload it in the actual implementation, raising a "unauthorized transformation" exception)
	bool RotateAroundAxis(double angle, vnl_vector<double> &axis) {
		//copied from http://inside.mines.edu/~gmurray/ArbitraryAxisRotation/
		if (axis.size() != 3) { throw DeformableModelException("RotateAroundAxis: unauthorized for this transformation"); }
		axis.normalize();
		double u=axis(0), v=axis(1), w=axis(2), ct=cos(angle), st=sin(angle);
		vnl_matrix<double> mat(m_nbDimensions+1, m_nbDimensions+1), rotmat(m_nbDimensions,m_nbDimensions);
		mat.set_identity();
		mat(0,0) = u + (1-u*u)*ct;	mat(0,1) = u*v*(1-ct)-w*st;	mat(0,2) = u*w*(1-ct)+v*st;
		mat(1,0) = u*v*(1-ct)+w*st;	mat(1,1) = v*v+(1-v*v)*ct;	mat(1,2) = v*w*(1-ct)-u*st;
		mat(2,0) = u*w*(1-ct)-v*st;	mat(2,1) = v*w*(1-ct)+u*st;	mat(2,2) = w*w+(1-w*w)*ct;
		return Compose(mat);			
	}

protected:
	Abstract3DTransform() : PoseTransform() {}
	virtual ~Abstract3DTransform() {};

	static const unsigned int m_nbDimensions = 3;

private:
	Abstract3DTransform(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented

};


//
//inline vnl_matrix<double> Get3DRotationMatrixFromEulerAngles(float h, float a, float b) {
//	//convention for the Euler Angles: see http://www.euclideanspace.com/maths/standards/index.htm
//	vnl_matrix<double> rot(3,3);
//	//vnl_matrix<double> r1(3,3), r2(3,3), r3(3,3);
//	////rotation around y
//	//r1.put(0,0, cos(h));	r1.put(0,1,   0   );	r1.put(0,2, sin(h));
//	//r1.put(1,0,   0   );	r1.put(1,1,   1   );	r1.put(1,2,   0   );
//	//r1.put(2,0,-sin(h));	r1.put(2,1,   0   );	r1.put(2,2, cos(h));
//	////rotation around z
//	//r2.put(0,0, cos(a));	r2.put(0,1,-sin(a));	r2.put(0,2,   0   );
//	//r2.put(1,0, sin(a));	r2.put(1,1, cos(a));	r2.put(1,2,   0   );
//	//r2.put(2,0,   0   );	r2.put(2,1,   0   );	r2.put(2,2,   1   );
//	////rotation around x
//	//r3.put(0,0,   1   );	r3.put(0,1,   0   );	r3.put(0,2,   0   );
//	//r3.put(1,0,   0   );	r3.put(1,1, cos(b));	r3.put(1,2,-sin(b));
//	//r3.put(2,0,   0   );	r3.put(2,1, sin(b));	r3.put(2,2, cos(b));
//	//rot = r1*r2*r3;
//
//	double ch=cos(h), sh=sin(h), ca=cos(a), sa=sin(a), cb=cos(b), sb=sin(b);
//	double sasb = sa*sb, sacb = sa*cb;
//
//	rot.put(0,0, ch*ca);		rot.put(0,1,-ch*sacb+sh*sb);		rot.put(0,2, ch*sasb+sh*cb);
//	rot.put(1,0, sa);			rot.put(1,1, ca*cb);				rot.put(1,2,-ca*sb);
//	rot.put(2,0,-sh*ca);		rot.put(2,1, sh*sacb+ch*sb);		rot.put(2,2,-sh*sasb+ch*cb);
//
//	return rot;
//}
//
//inline vnl_matrix<double> Get3DRotationMatrixFromEulerAngles(vnl_vector<double> euler_angles) {
//	float b = euler_angles[0];	float a = euler_angles[1];	float h = euler_angles[2];
//	return Get3DRotationMatrixFromEulerAngles(h, a, b);
//}
//
//inline vnl_vector<double> GetEulerAnglesFrom3DRotationMatrix(vnl_matrix<double> mat) {
//	//code adapted from http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToEuler/index.htm
//	vnl_vector<double> euler_angles(3);
//
//	if (mat(1,0) > 0.998) { // singularity at north pole
//		euler_angles(2) = atan2( mat(0,2), mat(2,2));
//		euler_angles(1) = PI/2.0;
//		euler_angles(0) = 0;
//		return euler_angles;
//	}
//	if (mat(1,0) < -0.998) { // singularity at south pole
//		euler_angles(2) = atan2( mat(0,2), mat(2,2));
//		euler_angles(1) =-PI/2.0;
//		euler_angles(0) = 0;
//		return euler_angles;
//	}
//	euler_angles(2) = atan2(-mat(2,0), mat(0,0));
//	euler_angles(1) =  asin( mat(1,0));
//	euler_angles(0) = atan2(-mat(1,2), mat(1,1));
//	return euler_angles;
//}

} // namespace psciob

#endif /* ABSTRACT3DTRANSFORM_H_ */
