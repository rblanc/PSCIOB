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
 * \file 3DTransformUtils.cpp
 * \author Rémi Blanc 
 * \date 27. February 2012
*/

#include "3DTransformUtils.h"

using namespace psciob;

vnl_matrix<double> psciob::Get3DRotationMatrixFromEulerAngles(double h, double a, double b) {
	//convention for the Euler Angles: see http://www.euclideanspace.com/maths/standards/index.htm
	vnl_matrix<double> rot(3,3);

	double ch=cos(h), sh=sin(h), ca=cos(a), sa=sin(a), cb=cos(b), sb=sin(b);
	double sasb = sa*sb, sacb = sa*cb;

	rot(0,0) = ch*ca;  rot(0,1) = sh*sb-ch*sacb;   rot(0,2)= ch*sasb+sh*cb;
	rot(1,0) = sa;     rot(1,1) = ca*cb;           rot(1,2)=-ca*sb;
	rot(2,0) =-sh*ca;  rot(2,1) = sh*sacb+ch*sb;   rot(2,2)=-sh*sasb+ch*cb;

	return rot;
}


vnl_matrix<double> psciob::Get3DRotationMatrixFromEulerAngles(const vnl_vector<double> &euler_angles) {
	//float b = euler_angles[0];	float a = euler_angles[1];	float h = euler_angles[2];
	return Get3DRotationMatrixFromEulerAngles(euler_angles(2), euler_angles(1), euler_angles(0));
}

vnl_vector<double> psciob::GetEulerAnglesFrom3DRotationMatrix(const vnl_matrix<double> &mat) {
	//code adapted from http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToEuler/index.htm
	vnl_vector<double> euler_angles(3);

	if (mat(1,0) > 0.999) { // singularity at north pole
		euler_angles(2) = atan2( mat(0,2), mat(2,2));
		euler_angles(1) = PI/2.0;
		euler_angles(0) = 0;
		return euler_angles;
	}
	if (mat(1,0) < -0.999) { // singularity at south pole
		euler_angles(2) = atan2( mat(0,2), mat(2,2));
		euler_angles(1) =-PI/2.0;
		euler_angles(0) = 0;
		return euler_angles;
	}
	euler_angles(2) = atan2(-mat(2,0), mat(0,0));
	euler_angles(1) = asin(  mat(1,0));
	euler_angles(0) = atan2(-mat(1,2), mat(1,1));
	return euler_angles;
}



vnl_matrix<double> psciob::Rotate3DTransformAroundX(const vnl_matrix<double> &R, double angle) {
	if (R.rows()!=R.cols()) throw DeformableModelException("Rotate3DRotationMatrixAroundX: input matrix must be square");
	if (!( (R.rows()==3) || ((R.rows()==4)) )) throw DeformableModelException("Rotate3DRotationMatrixAroundX: input matrix must be either 3*3 or 4*4 (in the case of homogeneous transforms)");
	vnl_matrix<double> mat(R.rows(), R.rows());
	mat.set_identity();
	//mat.put(0,0,   1   );		mat.put(0,1,   0   );		mat.put(0,2,   0   );
	mat.put(1,1, cos(angle));	mat.put(1,2,-sin(angle));//	mat.put(1,0,   0   );	
	mat.put(2,1, sin(angle));	mat.put(2,2, cos(angle));//	mat.put(2,0,   0   );	
	return ComposeTransforms(R, mat);
}

void psciob::Rotate3DTransformAroundX_InPlace(vnl_matrix<double> &R, double angle) {
	if (R.rows()!=R.cols()) throw DeformableModelException("Rotate3DRotationMatrixAroundX: input matrix must be square");
	if (!( (R.rows()==3) || ((R.rows()==4)) )) throw DeformableModelException("Rotate3DRotationMatrixAroundX: input matrix must be either 3*3 or 4*4 (in the case of homogeneous transforms)");
	vnl_matrix<double> mat(R.rows(), R.rows());	mat.set_identity(); mat.put(1,1, cos(angle));	mat.put(1,2,-sin(angle)); mat.put(2,1, sin(angle));	mat.put(2,2, cos(angle));
	ComposeTransforms_InPlace(R, mat);
}




vnl_matrix<double> psciob::Rotate3DTransformAroundY(const vnl_matrix<double> &R, double angle) {
	if (R.rows()!=R.cols()) throw DeformableModelException("Rotate3DRotationMatrixAroundX: input matrix must be square");
	if (!( (R.rows()==3) || ((R.rows()==4)) )) throw DeformableModelException("Rotate3DRotationMatrixAroundX: input matrix must be either 3*3 or 4*4 (in the case of homogeneous transforms)");
	vnl_matrix<double> mat(R.rows(), R.rows());
	mat.set_identity();
	mat.put(0,0, cos(angle));	mat.put(0,1,   0   );	mat.put(0,2, sin(angle));
	//mat.put(1,0,   0   );		mat.put(1,1,   1   );	mat.put(1,2,   0   );
	mat.put(2,0,-sin(angle));	mat.put(2,1,   0   );	mat.put(2,2, cos(angle));
	return ComposeTransforms(R, mat);
}

void psciob::Rotate3DTransformAroundY_InPlace(vnl_matrix<double> &R, double angle) {
	if (R.rows()!=R.cols()) throw DeformableModelException("Rotate3DRotationMatrixAroundX: input matrix must be square");
	if (!( (R.rows()==3) || ((R.rows()==4)) )) throw DeformableModelException("Rotate3DRotationMatrixAroundX: input matrix must be either 3*3 or 4*4 (in the case of homogeneous transforms)");
	vnl_matrix<double> mat(R.rows(), R.rows());	mat.set_identity();	mat.put(0,0, cos(angle)); mat.put(0,2, sin(angle)); mat.put(2,0,-sin(angle)); mat.put(2,2, cos(angle));
	ComposeTransforms_InPlace(R, mat);
}



vnl_matrix<double> psciob::Rotate3DTransformAroundZ(const vnl_matrix<double> &R, double angle) {
	if (R.rows()!=R.cols()) throw DeformableModelException("Rotate3DRotationMatrixAroundX: input matrix must be square");
	if (!( (R.rows()==3) || ((R.rows()==4)) )) throw DeformableModelException("Rotate3DRotationMatrixAroundX: input matrix must be either 3*3 or 4*4 (in the case of homogeneous transforms)");
	vnl_matrix<double> mat(R.rows(), R.rows());
	mat.set_identity();
	mat.put(0,0, cos(angle));	mat.put(0,1,-sin(angle));	//mat.put(0,2,   0   );
	mat.put(1,0, sin(angle));	mat.put(1,1, cos(angle));	//mat.put(1,2,   0   );
	//mat.put(2,0,   0   );		mat.put(2,1,   0   );		mat.put(2,2,   1   );
	return ComposeTransforms(R, mat);
}


void psciob::Rotate3DTransformAroundZ_InPlace(vnl_matrix<double> &R, double angle) {
	if (R.rows()!=R.cols()) throw DeformableModelException("Rotate3DRotationMatrixAroundX: input matrix must be square");
	if (!( (R.rows()==3) || ((R.rows()==4)) )) throw DeformableModelException("Rotate3DRotationMatrixAroundX: input matrix must be either 3*3 or 4*4 (in the case of homogeneous transforms)");
	vnl_matrix<double> mat(R.rows(), R.rows());	mat.set_identity();	mat.put(0,0, cos(angle));	mat.put(0,1,-sin(angle));	mat.put(1,0, sin(angle));	mat.put(1,1, cos(angle));
	ComposeTransforms_InPlace(R, mat);
}



/** 3D rotation matrix to Quaternion 
* formula from http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
*/
vnl_vector<double> psciob::GetQuaternionFrom3DRotationMatrix(const vnl_matrix<double> &R) {
	vnl_vector<double> Q(4);
	double T = 1.0 + R(0,0) + R(1,1) + R(2,2);
	double S;
	if (T>TINY) {
		S = 0.5 / sqrt(T);          
		Q(0) = 0.25 / S;            
		Q(1) = (R(2,1)-R(1,2)) * S; 
		Q(2) = (R(0,2)-R(2,0)) * S; 
		Q(3) = (R(1,0)-R(0,1)) * S; 
	}
	else if ( R(0,0) > R(1,1) && R(0,0) > R(2,2) ) {
		S = 2.0*sqrt(1.0+R(0,0)-R(1,1)-R(2,2));
		Q(0) = (R(1,2)-R(2,1))/S;
		Q(1) = 0.25*S;
		Q(2) = (R(0,1)+R(1,0))/S;
		Q(3) = (R(0,2)+R(2,0))/S;
	}
	else if ( R(1,1) > R(2,2) ) {
		S = 2.0*sqrt(1.0-R(0,0)+R(1,1)-R(2,2));
		Q(0) = (R(0,2)-R(2,0))/S;
		Q(1) = (R(0,1)+R(1,0))/S;
		Q(2) = 0.25*S;
		Q(3) = (R(1,2)+R(2,1))/S;
	}
	else {
		S = 2.0*sqrt(1.0-R(0,0)-R(1,1)+R(2,2));
		Q(0) = (R(0,1)-R(1,0))/S;
		Q(1) = (R(0,2)+R(2,0))/S;
		Q(2) = (R(1,2)+R(2,1))/S;
		Q(3) = 0.25*S;
	}

	return Q.normalize();
}

/** Quaternion to 3D rotation matrix (3*3)*/
vnl_matrix<double> psciob::Get3DRotationMatrixFromQuaternion_33(const vnl_vector<double> &Q) {
	vnl_matrix<double> R(3,3);
	double xx = Q(1)*Q(1), xy = Q(1)*Q(2), xz = Q(1)*Q(3), xw = Q(1)*Q(0);
	double yy = Q(2)*Q(2), yz = Q(2)*Q(3), yw = Q(2)*Q(0);
	double zz = Q(3)*Q(3), zw = Q(3)*Q(0);

	R(0,0) = 1.0 - 2.0 * ( yy + zz );
	R(0,1) =       2.0 * ( xy - zw );
	R(0,2) =       2.0 * ( xz + yw );

	R(1,0) =       2.0 * ( xy + zw );
	R(1,1) = 1.0 - 2.0 * ( xx + zz );
	R(1,2) =       2.0 * ( yz - xw );

	R(2,0) =       2.0 * ( xz - yw );
	R(2,1) =       2.0 * ( yz + xw );
	R(2,2) = 1.0 - 2.0 * ( xx + yy );
	return R;
}

/** Quaternion to 3D rotation matrix (3*3)*/
vnl_matrix<double> psciob::Get3DRotationMatrixFromQuaternion_44(const vnl_vector<double> &Q) {
	vnl_matrix<double> R(4,4); R.set_identity();
	double xx = Q(1)*Q(1), xy = Q(1)*Q(2), xz = Q(1)*Q(3), xw = Q(1)*Q(0);
	double yy = Q(2)*Q(2), yz = Q(2)*Q(3), yw = Q(2)*Q(0);
	double zz = Q(3)*Q(3), zw = Q(3)*Q(0);

	R(0,0) = 1.0 - 2.0 * ( yy + zz );
	R(0,1) =       2.0 * ( xy - zw );
	R(0,2) =       2.0 * ( xz + yw );

	R(1,0) =       2.0 * ( xy + zw );
	R(1,1) = 1.0 - 2.0 * ( xx + zz );
	R(1,2) =       2.0 * ( yz - xw );

	R(2,0) =       2.0 * ( xz - yw );
	R(2,1) =       2.0 * ( yz + xw );
	R(2,2) = 1.0 - 2.0 * ( xx + yy );
	return R;
}


/** Get vector and angle from quaternion */
void psciob::GetVectorAndAngleFromQuaternion(vnl_vector<double> &v, double &a, const vnl_vector<double> &Q) {
	vnl_vector<double> nQ = Q;
	nQ.normalize();

	a = 2.0 * acos(nQ(0));
	if (a>TINY) {
		v = nQ.extract(3,1);
		v.normalize();
	}
	else { v.set_size(3); v.fill(1.0/sqrt(3.0)); }
}

/** Get quaternion from vector and angle */
void psciob::GetQuaternionFromVectorAndAngle(const vnl_vector<double> &v, double a, vnl_vector<double> &Q) {
	double sina = sin(a/2.0);
	Q.set_size(4);
	Q(0) = cos(a/2.0);
	Q(1) = v(0) * sina;
	Q(2) = v(1) * sina;
	Q(3) = v(2) * sina;
	Q.normalize();
}
