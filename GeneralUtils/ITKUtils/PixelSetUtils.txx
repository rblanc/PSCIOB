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
* \file PixelSetUtils.txx
* \author Rémi Blanc 
* \date 8. March 2012
*/

#include "PixelSetUtils.h"

namespace psciob {

/** Convert a label map containing a single object into a list of pixels (only the first object is processed)
* this fills a std::vector in such a way that the pixel adresses in the 'image' defined by the LabelMap are stored in order
* this is based on the coding: address = i + j*width (+ k*width*height + ... )
* return false if the pixelSet ends empty for some reason...
*
* \todo THIS FUNCTION NEED TO BE TESTED!
*/
template <class TLabelMap>
bool ConvertSingleObjectLabelMapToPixelList( const TLabelMap *singleObjectMap, std::vector<unsigned long> &pixelList, typename TLabelMap::LabelObjectType::LabelType label) {
	pixelList.clear();

	TLabelMap::LabelObjectType *inputObject;
	if (label==0) inputObject = const_cast<TLabelMap::LabelObjectType *>(singleObjectMap->GetNthLabelObject(0));
	else {
		if (!singleObjectMap->HasLabel(label)) { std::cout<<"  label "<<(long)label<<" not found"<<std::endl; return false; }
		inputObject = const_cast<TLabelMap::LabelObjectType *>(singleObjectMap->GetLabelObject(label));
	}
	TLabelMap::IndexType index;

	vnl_vector<unsigned long> multvect(TLabelMap::ImageDimension);
	multvect(0) = 1;
	for (unsigned i=0 ; i<TLabelMap::ImageDimension-1 ; i++) multvect(i+1) = multvect(i)*singleObjectMap->GetLargestPossibleRegion().GetSize(i);
	long add;

	for (unsigned j=0 ; j<inputObject->GetNumberOfLines() ; j++) {
		index = inputObject->GetLine(j).GetIndex(); add = index[0];
		for (unsigned k=1 ; k<TLabelMap::ImageDimension ; k++) { add += index[k]*multvect[k]; }
		for (unsigned l = 0 ; l<inputObject->GetLine(j).GetLength() ; l++) { 
			pixelList.push_back( static_cast<unsigned long>(add++) ); //add the current pixel, and increment the address
		}
	}//processed all lines
	return (!pixelList.empty());
}


/** Convert a single object from a label map into a pixel set into a PixelSet sharing the same grid
* by default only the first object of the map is processed
* return false if the pixelSet ends empty for some reason...
*/
template <class TLabelMap>
bool ConvertSingleObjectLabelMapToPixelSet( const TLabelMap *singleObjectMap, ObjectPixelSet *pixelSet,
										   typename TLabelMap::LabelObjectType::LabelType label)
{
	pixelSet->Reset();
	ObjectPixelSet::SizeType size(TLabelMap::ImageDimension);
	for (unsigned k=0 ; k<TLabelMap::ImageDimension ; k++) size(k) = singleObjectMap->GetLargestPossibleRegion().GetSize(k);
	pixelSet->SetImageInformation( singleObjectMap->GetOrigin().GetVnlVector(), size, singleObjectMap->GetSpacing().GetVnlVector() );

	TLabelMap::LabelObjectType *inputObject;
	if (label==0) inputObject = const_cast<TLabelMap::LabelObjectType *>(singleObjectMap->GetNthLabelObject(0));
	else {
		if (!singleObjectMap->HasLabel(label)) { std::cout<<"  label "<<(long)label<<" not found"<<std::endl; return false; }
		inputObject = const_cast<TLabelMap::LabelObjectType *>(singleObjectMap->GetLabelObject(label));
	}

	TLabelMap::IndexType index;
	TLabelMap::LengthType length;
	ObjectPixelSet::IndexType psindex(TLabelMap::ImageDimension);
	vnl_vector<unsigned long> multvect = pixelSet->GetMultVect();

	bool emptyOutput = true; long add;
	if (inputObject->GetNumberOfLines()>0) emptyOutput = false;
	for (unsigned j=0 ; j<inputObject->GetNumberOfLines() ; j++) {
		//input index
		index = inputObject->GetLine(j).GetIndex();
		add = index[0];
		for (unsigned k=1 ; k<TLabelMap::ImageDimension ; k++) { add += index[k]*multvect[k]; }
		for (unsigned l = 0 ; l<inputObject->GetLine(j).GetLength() ; l++) { 
			pixelSet->AddPixel( add++ ); //add the current pixel, and increment the address
		}
	}//processed all lines
	return (!emptyOutput);

}


/** Convert a label containing a single object into a pixel set (only the first object is processed), with respect to the specified grid 
* The spacing must be the same in the label map and in the specified input
* Portions of the input object which fall outside the specified grid are cropped.
* return false if the pixelSet ends empty for some reason...
*/
template <class TLabelMap>
bool ConvertSingleObjectLabelMapToPixelSet( const TLabelMap *singleObjectMap, ObjectPixelSet *pixelSet, 
										   typename TLabelMap::PointType origin, typename TLabelMap::SizeType size, typename TLabelMap::SpacingType spacing) 
{
	unsigned int D = TLabelMap::ImageDimension;

	const TLabelMap::SpacingType &inputSpacing = singleObjectMap->GetSpacing();
	const TLabelMap::PointType &inputOrigin = singleObjectMap->GetOrigin();
	TLabelMap::IndexType index;
	TLabelMap::LengthType length;

	ObjectPixelSet::SpacingType outputSpacing(D);
	ObjectPixelSet::PointType outputOrigin(D);
	ObjectPixelSet::SizeType outputSize(D);

	TLabelMap::OffsetType offset;
	for (unsigned i=0 ; i<D ; i++) {
		outputSpacing(i) = spacing[i];
		outputOrigin(i) = origin[i];
		outputSize(i) = size[i];
		if ( outputSpacing[i] != inputSpacing[i] ) throw DeformableModelException("InsertSingleObjectLabelMapIntoAnother : both maps must have the same spacing!");
		offset[i] = round((inputOrigin[i] - origin[i]) / inputSpacing[i]);
	}

	//set the image size
	pixelSet->Reset();
	pixelSet->SetImageInformation(outputOrigin, outputSize, outputSpacing);
	ObjectPixelSet::IndexType setIndex(D);

	bool emptyOutput = true, lineOutside;
	long add;

	TLabelMap::LabelObjectType *inputObject = const_cast<TLabelMap::LabelObjectType *>(singleObjectMap->GetNthLabelObject(0));

	for (unsigned j=0 ; j<inputObject->GetNumberOfLines() ; j++) {
		lineOutside = false;
		//input index
		index = inputObject->GetLine(j).GetIndex();
		//first look at the non-varying coordinates of the line
		for (unsigned k=D-1 ; k>0 ; k--) {
			index[k] += offset[k];
			if ( (index[k] < 0) || (index[k] >= outputSize[k]) ) { lineOutside = true; break; } //discard lines that are fully outside
		}
		//finally look at the varying coordinate
		if (!lineOutside) {
			index[0] += offset[0];
			if (index[0]>=static_cast<int>(outputSize[0])) { lineOutside = true; continue; } //discard lines that are fully outside
			length = inputObject->GetLine(j).GetLength();
			if (index[0]+length<0) { lineOutside = true; continue; } //discard lines that are fully outside

			if (index[0]<0) { length += index[0];  index[0] = 0; } //shorten the line and update its starting index if it start too much on the left
			if (index[0]+length>static_cast<int>(outputSize[0])) { length = outputSize[0] - index[0]; } //shorten the line if it goes too far on the right

			if (length>0) { //add the line to the representation
				emptyOutput = false;
				for (unsigned i=0 ; i<D ; i++) setIndex(i) = index[i];
				add = pixelSet->IndexToFlatIndex( setIndex );
				for (unsigned l = 0 ; l<length ; l++) { 
					pixelSet->AddPixel( add++ ); //add the current pixel, and increment the address
				}
			}
			else lineOutside = true;
		}
	}//processed all lines
	return (!emptyOutput);
}

} // namespace psciob
