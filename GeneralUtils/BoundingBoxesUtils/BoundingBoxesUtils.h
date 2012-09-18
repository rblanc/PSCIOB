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
 * \file BoundingBoxesUtils.h
 * \author Rémi Blanc 
 * \26. September 2011
*/



#ifndef BOUNDINGBOXESUTILS_H_
#define BOUNDINGBOXESUTILS_H_

#include "GeneralUtils.h"


namespace psciob {


/** Test whether 2 axis-aligned bounding boxes intersect */
bool TestBoundingBoxesIntersection(const vnl_vector<double> &BB1, const vnl_vector<double> &BB2);

/** Test whether 2 bounding boxes intersect - without performing dimensionality checks */
inline bool TestBoundingBoxesIntersection_NoCheck(const vnl_vector<double> &BB1, const vnl_vector<double> &BB2) {
	for (unsigned i2=0 ; i2<BB1.size() ; i2+=2) { //if the leftmost point of I1 is to the right of the rightmost point of I2, or vice versa, then intersection is empty
		if ( BB2(i2+1)-BB1( i2 ) <= TINY ) { return false; } //actually, it could be a fraction of the spacing...
		if ( BB1(i2+1)-BB2( i2 ) <= TINY ) { return false; }
	}
	return true;
}


/** Computes the intersection of two bounding boxes 
* returns false if the bounding box do not overlap
*/
bool IntersectionBoundingBoxes(const vnl_vector<double> &BB1, const vnl_vector<double> &BB2, vnl_vector<double> *interBB);


/** Test whether bounding box 1 is fully inside bounding box 2 */
bool TestBoundingBoxFullyInsideAnother(const vnl_vector<double> &BB1, const vnl_vector<double> &BB2);


/** Identify which walls of the 2nd bounding box are crossed by BB1
* returns an std::vector containing indices of walls that are intersected (0=left, 1=right, 2= ...)
*/
std::vector<unsigned> IdentifyIntersectionBoundingBoxes(const vnl_vector<double> &BB1, const vnl_vector<double> &BB2);

/** Identify which walls of the 2nd bounding box are crossed by BB1
* returns an std::vector containing indices of walls that are intersected (0=left, 1=right, 2= ...)
* \warning does not check whether the dimensionality of both boxes matches
*/
inline std::vector<unsigned> IdentifyIntersectionBoundingBoxes_NoCheck(const vnl_vector<double> &BB1, const vnl_vector<double> &BB2) {
	std::vector<unsigned> intersects;
	for (unsigned i2=0 ; i2<BB2.size() ; i2+=2) {
		if ( (BB1( i2 )<BB2( i2 )) && (BB1(i2+1)>BB2( i2 )) ) intersects.push_back( i2 );
		if ( (BB1( i2 )<BB2(i2+1)) && (BB1(i2+1)>BB2(i2+1)) ) intersects.push_back(i2+1);
	}
	return intersects;
}

} // namespace psciob

#endif //BOUNDINGBOXESUTILS_H_

