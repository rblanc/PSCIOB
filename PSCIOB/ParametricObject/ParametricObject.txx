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

/*
* \file ParametricObject.txx
* \author Rémi Blanc 
* \date 8. March 2012
*/


#include "ParametricObject.h"

namespace psciob {



template <unsigned int VDimension, class TAppearance>
ParametricObject<VDimension, TAppearance>::ParametricObject() : m_uptodatePolyData(false), m_uptodateBinaryImage(false), m_uptodateLabelMap(false),
m_physicalBBoxUpToDate(false), m_physicalBoundingBox(2*VDimension),
m_imageBBoxUpToDate(false), m_imageBoundingBox(2*VDimension),
m_outputPolyData(NULL), m_outputBinaryImage(NULL), m_outputLabelMap(NULL), m_labelMapToBinaryImageFilter(NULL)
{
	m_imageSpacing.Fill(1);
}

//
// SetParameters
//
template <unsigned int VDimension, class TAppearance>
bool 
ParametricObject<VDimension, TAppearance>::SetParameters(const vnl_vector<double> &params) 
{
	if (!CheckParameters(params)) { return false;}

	//First, look at the non-translation parameters
	for (unsigned i=Dimension ; i<params.size() ; i++) { 
		if ( fabs(m_parameters(i)-params(i))>TINY ) { m_parameters = params; Modified(); return true; }
	}

	//Then, look at the translation parameters and look for a grid-integer translation
	vnl_vector<int> integerTranslation(Dimension); double tmpval; //bool zerotranslation = true; 
	for (unsigned i=0 ; i<Dimension ; i++) {
		tmpval = (params(i)-m_parameters(i))/m_imageSpacing[i];
		integerTranslation(i) = round(tmpval);
		if ( fabs(tmpval - integerTranslation(i)) > TINY ) { m_parameters = params; Modified(); return true; }
		//else if ( abs(integerTranslation(i)) > TINY ) zerotranslation = false;
	}
	//At this point, nothing but the translation may be different, if it is a non-zero translation, just applied it
	//if (!zerotranslation) 
	_IntegerGridTranslate_NoCheck(integerTranslation);
	return true;
}



//
// PositionAt
//
template <unsigned int VDimension, class TAppearance>
bool 
ParametricObject<VDimension, TAppearance>::PositionAt(const vnl_vector<double> &pos) 
{
	if (pos.size()!=Dimension) return false;

	vnl_vector<int> integerTranslation(Dimension); double tmpval; //bool zerotranslation = true; 
	for (unsigned i=0 ; i<Dimension ; i++) {
		tmpval = ( pos(i)-m_parameters(i) ) / m_imageSpacing[i]; //continuous index
		integerTranslation(i) = round(tmpval); 
		if ( fabs(tmpval - integerTranslation(i)) > TINY ) { 
			//if one of the parameters is not a grid-integer translation, do the translation and exit (idea is to avoid the next divisions, rounding, ...)
			for (unsigned j=0 ; j<Dimension ; j++) m_parameters(j) = pos(j);
			Modified(); return true; 
		}
		//else if ( abs(integerTranslation(i)) > TINY ) zerotranslation = false;
	}
	//
	//if (!zerotranslation) 
	_IntegerGridTranslate_NoCheck(integerTranslation);
	return true;
}


//
// Translate
//
template <unsigned int VDimension, class TAppearance>
bool 
ParametricObject<VDimension, TAppearance>::Translate(const vnl_vector<double> &translation) 
{
	if (translation.size()!=Dimension) return false;

	vnl_vector<int> integerTranslation(Dimension); double tmpval; //bool zerotranslation = true;
	for (unsigned i=0 ; i<Dimension ; i++) {
		tmpval = translation(i) / m_imageSpacing[i];
		integerTranslation(i) = round(tmpval);
		if ( fabs(tmpval - integerTranslation(i)) > TINY ) { 
			//if one of the parameters is not a grid-integer translation, do the translation and exit (idea is to avoid the next divisions, rounding, ...)
			for (unsigned j=0 ; j<Dimension ; j++) m_parameters(j) += translation(j);
			Modified(); return true; 
		}
		//else if ( abs(integerTranslation(i)) > TINY ) zerotranslation = false;
	}
	//
	//if (!zerotranslation) 
	_IntegerGridTranslate_NoCheck(integerTranslation);
	return true;
}


/** Get Center Of Gravity and Inertia matrix of the shape 
* the function fills the information in the provided structures
* this is a base implementation which computes it from the LabelObject, it can be re-implemented to be faster in child classes
*/
template <unsigned int VDimension, class TAppearance>
void 
ParametricObject<VDimension, TAppearance>::GetObjectCenterAndInertiaMatrix(vnl_vector<double> &center, vnl_matrix<double> &mat) {
	center.set_size(VDimension); center.fill(0);
	mat.set_size(VDimension,VDimension); mat.fill(0);

	//browse the lines of the object
	LabelMapType* lm = GetObjectAsLabelMap(); //make sure the labelMap is uptodate
	LabelObjectType* lo = lm->GetNthLabelObject(0);

	LabelObjectType::ConstLineIterator lit( lo );
	LabelObjectType::IndexType index;
	vnl_vector<double> coords(VDimension); unsigned nbPts=0;
	for (lit.GoToBegin() ; ! lit.IsAtEnd() ; ++lit) {
		//get the index, and convert it to physical coordinates
		index = lit.GetLine().GetIndex();
		for (unsigned d1=0 ; d1<VDimension ; d1++) coords(d1) = lm->GetOrigin()[d1] + index[d1]*m_imageSpacing[d1];

		for (unsigned i=0 ; i< lit.GetLine().GetLength() ; i++, coords(0)+=m_imageSpacing[0], nbPts++) {
			center += coords;
			// http://fr.wikipedia.org/wiki/Covariance
			mat += outer_product(coords,coords);
		}
	}

	center /= nbPts;
	mat = mat/(nbPts-1.0) - outer_product(center,center)*nbPts*nbPts/((nbPts-1.0)*(nbPts-1.0));

}

} // namespace psciob

