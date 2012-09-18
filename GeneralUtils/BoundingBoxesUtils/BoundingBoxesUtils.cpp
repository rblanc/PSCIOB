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
 * \file BoundingBoxesUtils.cpp
 * \author Rémi Blanc 
 * \date 27. February 2012
*/

#include "BoundingBoxesUtils.h"

using namespace psciob;

bool psciob::TestBoundingBoxesIntersection(const vnl_vector<double> &BB1, const vnl_vector<double> &BB2) {
	unsigned int D = BB1.size()/2;
	if (2*D != BB2.size()) throw DeformableModelException("TestBoundingBoxesIntersection : error with dimensionality of inputs");

	for (unsigned i2=0 ; i2<BB2.size() ; i2+=2) { //if the leftmost point of I1 is to the right of the rightmost point of I2, or vice versa, then intersection is empty
		if ( BB2(i2+1)-BB1( i2 ) <= TINY ) { return false; } //actually, it could be a fraction of the spacing...
		if ( BB1(i2+1)-BB2( i2 ) <= TINY ) { return false; }
	}
	return true;
}


bool psciob::IntersectionBoundingBoxes(const vnl_vector<double> &BB1, const vnl_vector<double> &BB2, vnl_vector<double> *interBB) {
	unsigned int D = BB1.size()/2;
	if (2*D != BB2.size()) throw DeformableModelException("IntersectionBoundingBoxes : error with dimensionality of inputs");

	(*interBB).set_size(BB2.size());
	for (unsigned j=0 ; j<BB2.size() ; j+=2) { //if the leftmost point of I1 is to the right of the rightmost point of I2, or vice versa, then intersection is empty
		if ( BB2(j+1)-BB1( j ) <= TINY ) { return false; } //actually, it could be a fraction of the spacing...
		if ( BB1(j+1)-BB2( j ) <= TINY ) { return false; }
		(*interBB)(j) = max(BB1(j),BB2(j));
		(*interBB)(j+1) = min(BB1(j+1),BB2(j+1));
	}
	return true;
}


bool psciob::TestBoundingBoxFullyInsideAnother(const vnl_vector<double> &BB1, const vnl_vector<double> &BB2) {
	unsigned int D = BB1.size()/2;
	if (2*D != BB2.size()) throw DeformableModelException("TestBoundingBoxFullyInsideAnother : error with dimensionality of inputs");

	for (unsigned i2=0 ; i2<BB2.size() ; i2+=2) {
		if ( BB1( i2 )<BB2( i2 ) ) return false;
		if ( BB1(i2+1)>BB2(i2+1) ) return false;
	}
	return true;
}



std::vector<unsigned> psciob::IdentifyIntersectionBoundingBoxes(const vnl_vector<double> &BB1, const vnl_vector<double> &BB2) {
	std::vector<unsigned> intersects;
	unsigned int D = BB1.size()/2;
	if (2*D != BB2.size()) throw DeformableModelException("IdentifyIntersectionBoundingBoxes : error with dimensionality of inputs");

	for (unsigned i2=0 ; i2<BB2.size() ; i2+=2) {
		if ( (BB1( i2 )<BB2( i2 )) && (BB1(i2+1)>BB2( i2 )) ) intersects.push_back( i2 );
		if ( (BB1( i2 )<BB2(i2+1)) && (BB1(i2+1)>BB2(i2+1)) ) intersects.push_back(i2+1);
	}
	return intersects;
}
