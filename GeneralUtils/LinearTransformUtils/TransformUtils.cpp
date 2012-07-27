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
 * \file TransformUtils.cpp
 * \author Rémi Blanc 
 * \date 27. February 2012
*/

#include "TransformUtils.h"

using namespace psciob;



vnl_matrix<double> psciob::ComposeTransforms(const vnl_matrix<double> &T1, const vnl_matrix<double> &T2) {
	if ( (T1.rows()!=T1.cols()) || (T2.rows()!=T2.cols()) || (T1.rows()!=T2.cols()) ) throw DeformableModelException("ComposeTransforms: inconsistent inputs");
	return T1*T2;
}

void psciob::ComposeTransforms_InPlace(vnl_matrix<double> &T1, const vnl_matrix<double> &T2) {
	if ( (T1.rows()!=T1.cols()) || (T2.rows()!=T2.cols()) || (T1.rows()!=T2.cols()) ) throw DeformableModelException("ComposeTransforms: inconsistent inputs");
	T1 = T1*T2;
}

///** Compose Homogeneous Transforms, of the generic form:
//* R t
//* 0 1
//* RESULT IS JUST THE SAME AS ComposeTransforms ; may be faster, or may be slower... ???
//*/
//vnl_matrix<double> ComposeHomogeneousTransform(const vnl_matrix<double> &T1, const vnl_matrix<double> &T2) {
//	if ( (T1.rows()!=T1.cols()) || (T2.rows()!=T2.cols()) || (T1.rows()!=T2.cols()) ) throw DeformableModelException("ComposeTransforms: inconsistent inputs");
//	unsigned D = T1.rows()-1;
//
//	vnl_matrix<double> outputMat = T1;
//	vnl_matrix<double> tmp_mat = T1.extract(D,D,0,0), tmp_vect;
//
//	tmp_vect= T1.extract(D,D,0,0)*T2.extract(n,1,0,n) + T1.extract(n,1,0,n);
//	tmp_mat = T1.extract(D,D,0,0)*T2.extract(n,n,0,0);
//
//	for (unsigned i=0 ; i<n ; i++) {
//		m_internalTransformMatrix(i,n) = tmp_vect(i,0);
//		for (unsigned j=0 ; j<n ; j++) { m_internalTransformMatrix(i,j) = tmp_mat(i,j); }	
//}






vnl_vector<double> psciob::TransformBoundingBox(const vnl_vector<double> &inputBB, const vnl_matrix<double> &transformMatrix) {
	unsigned int n = transformMatrix.rows();
	unsigned int len = inputBB.size();
	unsigned int D = n-1;

	if ( (n!=transformMatrix.rows()) || (len!=2*D) ) {
		std::cout<<"-- size inputBB= "<<inputBB.size()<<", size transformMatrix= "<<transformMatrix.rows()<<" ; matrix = "<<transformMatrix<<std::endl; 
		throw DeformableModelException("Error in TransformBoundingBox : inconsistent inputs");
	}
	vnl_vector<double> outputBB(len);

	vnl_matrix<double> rot(D,D); vnl_vector<double> trans(D);
	rot = transformMatrix.extract(D,D,0,0); trans = transformMatrix.extract(D,1,0,D).get_column(0);

	vnl_vector<double> p1(D), p2(D), p3(D), p4(D), p5(D), p6(D), p7(D), p8(D);
	vnl_vector<double> q1(D), q2(D), q3(D), q4(D), q5(D), q6(D), q7(D), q8(D);

	switch(D) {
		case 2:
			p1(0) = inputBB(0); p1(1) = inputBB(2); 	p2(0) = inputBB(1); p2(1) = inputBB(2); 
			p3(0) = inputBB(1); p3(1) = inputBB(3);		p4(0) = inputBB(0); p4(1) = inputBB(3); 

			q1 = rot*p1 + trans;		q2 = rot*p2 + trans;		q3 = rot*p3 + trans;		q4 = rot*p4 + trans;

			outputBB(0)= q1(0); outputBB(1)= q1(0); outputBB(2)= q1(1); outputBB(3)= q1(1);
			for (unsigned int i=0 ; i<D ; i++) {
				outputBB(2*i) = min(outputBB(2*i), q2(i));	outputBB(2*i+1) = max(outputBB(2*i+1), q2(i));
				outputBB(2*i) = min(outputBB(2*i), q3(i));	outputBB(2*i+1) = max(outputBB(2*i+1), q3(i));
				outputBB(2*i) = min(outputBB(2*i), q4(i));	outputBB(2*i+1) = max(outputBB(2*i+1), q4(i));
			}
			break;
		case 3:
			p1(0) = inputBB(0); p1(1) = inputBB(2); p1(2) = inputBB(4);		p2(0) = inputBB(1); p2(1) = inputBB(2); p2(2) = inputBB(4);
			p3(0) = inputBB(1); p3(1) = inputBB(3); p3(2) = inputBB(4);		p4(0) = inputBB(0); p4(1) = inputBB(3); p4(2) = inputBB(4);
			p5(0) = inputBB(0); p5(1) = inputBB(2); p5(2) = inputBB(5);		p6(0) = inputBB(1); p6(1) = inputBB(2); p6(2) = inputBB(5);
			p7(0) = inputBB(1); p7(1) = inputBB(3); p7(2) = inputBB(5);		p8(0) = inputBB(0); p8(1) = inputBB(3); p8(2) = inputBB(5);

			q1 = rot*p1 + trans;		q2 = rot*p2 + trans;		q3 = rot*p3 + trans;		q4 = rot*p4 + trans;
			q5 = rot*p5 + trans;		q6 = rot*p6 + trans;		q7 = rot*p7 + trans;		q8 = rot*p8 + trans;

			outputBB(0)= q1(0); outputBB(1)= q1(0); outputBB(2)= q1(1); outputBB(3)= q1(1); outputBB(4)= q1(2); outputBB(5)= q1(2);
			for (unsigned int i=0 ; i<D ; i++) {
				outputBB(2*i) = min(outputBB(2*i), q2(i));	outputBB(2*i+1) = max(outputBB(2*i+1), q2(i));
				outputBB(2*i) = min(outputBB(2*i), q3(i));	outputBB(2*i+1) = max(outputBB(2*i+1), q3(i));
				outputBB(2*i) = min(outputBB(2*i), q4(i));	outputBB(2*i+1) = max(outputBB(2*i+1), q4(i));
				outputBB(2*i) = min(outputBB(2*i), q5(i));	outputBB(2*i+1) = max(outputBB(2*i+1), q5(i));
				outputBB(2*i) = min(outputBB(2*i), q6(i));	outputBB(2*i+1) = max(outputBB(2*i+1), q6(i));
				outputBB(2*i) = min(outputBB(2*i), q7(i));	outputBB(2*i+1) = max(outputBB(2*i+1), q7(i));
				outputBB(2*i) = min(outputBB(2*i), q8(i));	outputBB(2*i+1) = max(outputBB(2*i+1), q8(i));
			}

			break;
		default : 
			throw DeformableModelException("Error in TransformBoundingBox : expecting 2 or 3 dimensional spaces only");
			break;
	}

	return outputBB;
}





/** Applies an affine transform specified by the homogeneous transformation matrix to a vtkPolyData */
void psciob::AffineTransformVTKPolyData( vtkPolyData* inputPolyData, vnl_matrix<double> transformMatrix, vtkPolyData* outputPolyData) {
	if ( transformMatrix.rows() != transformMatrix.cols() ) throw DeformableModelException("transform matrix must be square");
	unsigned int d = transformMatrix.rows()-1;
	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();

	switch(d) {
	case 2:
		matrix->SetElement(0,0, transformMatrix(0,0)); matrix->SetElement(0,1, transformMatrix(0,1)); matrix->SetElement(0,2, 0); matrix->SetElement(0,3, transformMatrix(0,2)); 
		matrix->SetElement(1,0, transformMatrix(1,0)); matrix->SetElement(1,1, transformMatrix(1,1)); matrix->SetElement(1,2, 0); matrix->SetElement(1,3, transformMatrix(1,2)); 
		matrix->SetElement(2,0,         0           ); matrix->SetElement(2,1,         0           ); matrix->SetElement(2,2, 1); matrix->SetElement(2,3,         0           ); 
		matrix->SetElement(3,0,         0           ); matrix->SetElement(3,1,         0           ); matrix->SetElement(3,2, 0); matrix->SetElement(3,3,         1           ); 
		break;
	case 3:
		for (unsigned i=0 ; i<4 ; i++) {
			for (unsigned j=0 ; j<4 ; j++) { 
				matrix->SetElement(i,j, transformMatrix(i,j)); 
			} 
		}
		break;
	default: throw DeformableModelException("only 2D or 3D transforms are allowed -> 3*3 or 4*4 matrices");
	}

	transform->SetMatrix( matrix );
	transform->Update();

	transformFilter->SetInput( inputPolyData );
	transformFilter->SetTransform( transform );
	transformFilter->Update();

	outputPolyData->ShallowCopy( transformFilter->GetOutput() );
}

