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
* \file LabelMapUtils.txx
* \author Rémi Blanc 
* \date 8. March 2012
*/

#include "ITKUtils.h"
#include "LabelMapUtils.h"
#include "itkBinaryBallStructuringElement.h"
#include "itkLabelObjectLineComparator.h"

namespace psciob {

/** Convert a label map to a different frame (already indicated as the largest possible region of the outputMap)
* objects that fall outside the output frame are discarded
* objects that fall partially outside the output frame are cropped accordingly
* the input and output maps must have the same spacing
*/
//it would be quite complicated to allow different spacings ; or very spurious effects (holes, duplicated lines;... ) 
template <class TInputLabelMap, class TOutputLabelMap>
void CastLabelMapToOtherFrame(TInputLabelMap *inputMap, TOutputLabelMap *outputMap) {
	std::cout<<"CastLabelMapToOtherFrame :: TODO / OPTIMIZATION: use an iterator over the LabelObjects of the map!!!"<<std::endl;
	outputMap->ClearLabels();
	unsigned int D = TInputLabelMap::ImageDimension;
	if (TOutputLabelMap::ImageDimension != D) throw DeformableModelException("CastLabelMapToOtherFrame : both maps must the same dimensionality!");

	const TInputLabelMap::SpacingType & inputSpacing  = inputMap->GetSpacing();
	const TInputLabelMap::SpacingType & outputSpacing = outputMap->GetSpacing();
	const TInputLabelMap::PointType   & inputOrigin   = inputMap->GetOrigin();
	const TInputLabelMap::PointType   & outputOrigin  = outputMap->GetOrigin();
	const TInputLabelMap::SizeType    & inputSize     = inputMap->GetLargestPossibleRegion().GetSize();
	const TInputLabelMap::SizeType    & outputSize    = outputMap->GetLargestPossibleRegion().GetSize();
	TInputLabelMap::IndexType index;
	TInputLabelMap::LengthType length;

	TInputLabelMap::OffsetType offset;
	bool sameFrame = true;
	for (unsigned i=0 ; i<D ; i++) {
		if ( outputSpacing[i] != inputSpacing[i] ) throw DeformableModelException("CastLabelMapToOtherFrame : both maps must have the same spacing!");
		offset[i] = round((inputOrigin[i] - outputOrigin[i]) / inputSpacing[i]);
		if ( (inputOrigin[i]!=outputOrigin[i]) || (inputSize[i]!=outputSize[i]) ) sameFrame=false;
	}

	bool emptyOutput, lineOutside;

	TInputLabelMap::Iterator objectIt  =  TInputLabelMap::Iterator(inputMap);
	for (objectIt.GoToBegin() ; !objectIt.IsAtEnd() ; ++objectIt) {
		TInputLabelMap::LabelObjectType *inputObject = objectIt.GetLabelObject();
		emptyOutput = true;
		TOutputLabelMap::LabelObjectType::Pointer outputObject = TOutputLabelMap::LabelObjectType::New();
		outputObject->SetLabel( inputObject->GetLabel() );
		if (sameFrame) { 
			for (unsigned j=0 ; j<inputObject->GetNumberOfLines() ; j++) {
				outputObject->AddLine(inputObject->GetLine(j)); emptyOutput=false; 
			}
		}
		else {
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

					if (index[0]<0) { length += index[0];  index[0] = 0; } //shorten the line and update its starting index
					if (index[0]+length>static_cast<int>(outputSize[0])) { length = outputSize[0] - index[0]; } //shorten the line

					if (length>0) { //add the line
						outputObject->AddLine(index, length);
						emptyOutput = false;
					}
					else lineOutside = true;
				}
			}
		}//processed all lines
		if (!emptyOutput) outputMap->AddLabelObject(outputObject);
	}//processed all LabelObjects
}


//
//
//
template <class TInputLabelMap, class TOutputLabelMap>
bool InsertSingleObjectLabelMapIntoAnother(TInputLabelMap *singleObjectMap, TOutputLabelMap *mergingMap, typename TOutputLabelMap::LabelType proposedLabel) {
	unsigned int D = TInputLabelMap::ImageDimension;
	if (TOutputLabelMap::ImageDimension != D) throw DeformableModelException("InsertSingleObjectLabelMapIntoAnother : both maps must the same dimensionality!");
	if (singleObjectMap->GetNumberOfLabelObjects()==0) return false;

	const TInputLabelMap::SpacingType & inputSpacing  = singleObjectMap->GetSpacing();
	const TInputLabelMap::SpacingType & outputSpacing = mergingMap->GetSpacing();
	const TInputLabelMap::PointType   & inputOrigin   = singleObjectMap->GetOrigin();
	const TInputLabelMap::PointType   & outputOrigin  = mergingMap->GetOrigin();
	const TInputLabelMap::SizeType    & inputSize     = singleObjectMap->GetLargestPossibleRegion().GetSize();
	const TInputLabelMap::SizeType    & outputSize    = mergingMap->GetLargestPossibleRegion().GetSize();
	TInputLabelMap::IndexType  index;
	long length; //necessary to use a signed type, to allow subtractions...

	TInputLabelMap::OffsetType offset;
	bool sameFrame = true;
	for (unsigned i=0 ; i<D ; i++) {
		if ( outputSpacing[i] != inputSpacing[i] ) throw DeformableModelException("InsertSingleObjectLabelMapIntoAnother : both maps must have the same spacing!");
		offset[i] = round((inputOrigin[i] - outputOrigin[i]) / inputSpacing[i]);
		if ( (inputOrigin[i]!=outputOrigin[i]) || (inputSize[i]!=outputSize[i]) ) sameFrame=false;
	}
	bool emptyOutput = true, lineOutside;

	TInputLabelMap::LabelObjectType *inputObject = singleObjectMap->GetNthLabelObject(0);
	TOutputLabelMap::LabelObjectType::Pointer outputObject = TOutputLabelMap::LabelObjectType::New();

	if (sameFrame) { 
		for (unsigned j=0 ; j<inputObject->GetNumberOfLines() ; j++) {
			outputObject->AddLine( inputObject->GetLine(j) ); emptyOutput=false; 
		}
	}
	else {
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

				if (index[0]<0) { length += index[0];  index[0] = 0; } //shorten the line and update its starting index
				if (index[0]+length>static_cast<int>(outputSize[0])) { length = outputSize[0] - index[0]; } //shorten the line
				if (length>0) { //add the line
					outputObject->AddLine(index, length);
					emptyOutput = false;
				}
				else lineOutside = true;
			}
		}
	}//processed all lines
	if (!emptyOutput) {
		if (proposedLabel==0) {
			mergingMap->PushLabelObject(outputObject);
		}
		else {
			outputObject->SetLabel( proposedLabel );
			mergingMap->AddLabelObject(outputObject);
		}
		return true;
	}
	else return false; 
}



//
//
//
template <class TInputLabelMap, class TOutputLabelMap>
bool InsertSingleObjectLabelMapIntoAnother_FullyInside(TInputLabelMap *singleObjectMap, TOutputLabelMap *mergingMap, typename TOutputLabelMap::LabelType proposedLabel) {
	unsigned int D = TInputLabelMap::ImageDimension;
	if (TOutputLabelMap::ImageDimension != D) throw DeformableModelException("CastLabelMapToOtherFrame : both maps must the same dimensionality!");

	const TInputLabelMap::SpacingType & inputSpacing  = singleObjectMap->GetSpacing();
	const TInputLabelMap::SpacingType & outputSpacing = mergingMap->GetSpacing();
	const TInputLabelMap::PointType   & inputOrigin   = singleObjectMap->GetOrigin();
	const TInputLabelMap::PointType   & outputOrigin  = mergingMap->GetOrigin();
	const TInputLabelMap::SizeType    & inputSize     = singleObjectMap->GetLargestPossibleRegion().GetSize();
	const TInputLabelMap::SizeType    & outputSize    = mergingMap->GetLargestPossibleRegion().GetSize();
	TInputLabelMap::IndexType index;
	TInputLabelMap::LengthType length;

	TInputLabelMap::OffsetType offset;
	bool sameFrame=true;
	for (unsigned i=0 ; i<D ; i++) {
		if ( outputSpacing[i] != inputSpacing[i] ) throw DeformableModelException("InsertSingleObjectLabelMapIntoAnother : both maps must have the same spacing!");
		offset[i] = round((inputOrigin[i] - outputOrigin[i]) / inputSpacing[i]);
		if ( (inputOrigin[i]!=outputOrigin[i]) || (inputSize[i]!=outputSize[i]) ) sameFrame=false;
	}

	TInputLabelMap::LabelObjectType::Pointer inputObject = singleObjectMap->GetNthLabelObject(0);
	TOutputLabelMap::LabelObjectType::Pointer outputObject = TOutputLabelMap::LabelObjectType::New();

	if (sameFrame) { 
		for (unsigned j=0 ; j<inputObject->GetNumberOfLines() ; j++) {
			outputObject->AddLine(inputObject->GetLine(j)); }
	}
	else {
		for (unsigned j=0 ; j<inputObject->GetNumberOfLines() ; j++) {
			//input index
			index = inputObject->GetLine(j).GetIndex();
			for (unsigned k=0 ; k<D ; k++) { index[k] += offset[k]; }
			outputObject->AddLine(index, inputObject->GetLine(j).GetLength());
		}
	}

	if (proposedLabel==0) {
		mergingMap->PushLabelObject(outputObject);
	}
	else {
		outputObject->SetLabel( proposedLabel );
		mergingMap->AddLabelObject(outputObject);
	}
	return true;
}



/** Generate a LabelObject that represent a ball structuring element of specified radius */
template <class TLabelObject>
void GenerateBallStructuringLabelObject(const unsigned int radius, TLabelObject *seObject) {
	typedef itk::BinaryBallStructuringElement<unsigned char, TLabelObject::ImageDimension > StructuringElementType;
	StructuringElementType se; se.SetRadius( radius ); se.CreateStructuringElement();
	//look into the data, and construct the LabelObject...
	seObject->Clear();

	StructuringElementType::OffsetType offset;
	TLabelObject::IndexType index;
	StructuringElementType::Iterator it; unsigned i;
	//the iterator actually browses an image (?) and returns 0 or 1 whether the position is outside/inside the structuring element
	for (it = se.Begin(), i=0 ; it != se.End() ; ++it, ++i) {
		if (*it) {
			offset = se.GetOffset(i);
			for (unsigned j=0 ; j<TLabelObject::ImageDimension ; j++) index[j] = offset[j];
			seObject->AddIndex(index);
		}
	}
}


/** Dilate each object of the input label map independently one from the others
* using the specified structuring element (formatted as a labelobject)
*/
template <class TLabelMap>
bool DilateOffContextObjects(const TLabelMap *inputMap, const typename TLabelMap::LabelObjectType *SE, TLabelMap *outputMap) {
	outputMap->ClearLabels();
	outputMap->SetOrigin( inputMap->GetOrigin() );
	outputMap->SetSpacing( inputMap->GetSpacing() );
	outputMap->SetRegions( inputMap->GetLargestPossibleRegion() );

	const TLabelMap::SizeType &size = inputMap->GetLargestPossibleRegion().GetSize();
	TLabelMap::LabelObjectType::ConstPointer inputLabelObject;
	//1: browse all objects of the input map
	TLabelMap::LabelObjectType::IndexType newIndex;
	TLabelMap::LabelObjectType::LengthType length, newLength;
	bool lineOutside;
	int rightIndex;
	TLabelMap::ConstIterator labelObjectIterator =  TLabelMap::ConstIterator(inputMap);
	for (labelObjectIterator.GoToBegin() ; !labelObjectIterator.IsAtEnd() ; ++labelObjectIterator) {
		inputLabelObject = labelObjectIterator.GetLabelObject();
		//instanciate the output object.
		TLabelMap::LabelObjectType::Pointer outputLabelObject = TLabelMap::LabelObjectType::New();
		outputLabelObject->SetLabel( inputLabelObject->GetLabel() );

		for (unsigned lineIndex = 0 ; lineIndex < inputLabelObject->GetNumberOfLines() ; lineIndex++) {
			const TLabelMap::LabelObjectType::IndexType &index = inputLabelObject->GetLine(lineIndex).GetIndex();
			length = inputLabelObject->GetLine(lineIndex).GetLength();
			for (unsigned seLine = 0 ; seLine<SE->GetNumberOfLines() ; seLine++) {
				//generate a new line, using the right offset, and the right length.
				const TLabelMap::LabelObjectType::IndexType &offset = SE->GetLine(seLine).GetIndex();
				lineOutside = false;
				//check the 2nd and next indices to check the line will remain inside the frame.
				for (unsigned d=1 ; d<TLabelMap::ImageDimension ; d++) {
					newIndex[d] = index[d] + offset[d]; 
					if ( (newIndex[d]<0) || (newIndex[d]>=size[d]) ) { lineOutside = true; break; } 
				}
				if (lineOutside) { continue; }
				//check the 1st index now and compute the length
				newIndex[0] = index[0] + offset[0];
				newLength = length + SE->GetLine(seLine).GetLength() - 1;

				//shorten the line if necessary (left and right)
				rightIndex = newIndex[0] + newLength;
				if ( rightIndex > size[0]) { rightIndex = size[0]; } 
				if (newIndex[0]<0) { newIndex[0]=0; }
				newLength = rightIndex - newIndex[0];

				outputLabelObject->AddLine( newIndex, newLength );
			}
		}
		outputLabelObject->Optimize();
		outputMap->AddLabelObject(outputLabelObject);
	}

	return true;
}



/** Compute a LabelMap from the input LabelMap, which contains the same number of objects
* each output object corresponds to the (offcontext) surroundings of the input ones
* and which is obtained through the subtraction of the morphological dilation of the object with itself, using the specified structuring element (formatted as a labelobject)
*/
template <class TLabelMap>
bool GetOffContextObjectsSurroundings(const TLabelMap *inputMap, const typename TLabelMap::LabelObjectType *SE, TLabelMap *outputMap) {
	//1: dilate the input objects, this also formats the outputMap to the right dimensions.
	DilateOffContextObjects(inputMap, SE, outputMap);

	//2: perform a set-subtraction, object by object...
	TLabelMap::LabelObjectType::ConstPointer inputLabelObject;
	TLabelMap::LabelObjectType::ConstPointer outputLabelObject;

	TLabelMap::ConstIterator inputObjectIterator  =  TLabelMap::ConstIterator(inputMap);
	TLabelMap::ConstIterator outputObjectIterator =  TLabelMap::ConstIterator(outputMap);
	TLabelMap::LabelObjectType::IndexType newIndex;
	TLabelMap::LabelObjectType::LengthType newLength;
	int inRightIndex, outRightIndex;
	int inLineIndex, outLineIndex, lastOutLineIndex, nbLinesInSubtract;
	//process each object independently from the others
	for (inputObjectIterator.GoToBegin(), outputObjectIterator.GoToBegin() ; !inputObjectIterator.IsAtEnd() ; ++inputObjectIterator, ++outputObjectIterator) {
		inputLabelObject  = inputObjectIterator.GetLabelObject();
		outputLabelObject = outputObjectIterator.GetLabelObject();

		TLabelMap::LabelObjectType::Pointer subtractedLabelObject = TLabelMap::LabelObjectType::New();

		if (!GetLabelObjectDifference<TLabelMap::LabelObjectType>(outputLabelObject, inputLabelObject, subtractedLabelObject)) std::cout<<"WARNING -- GetOffContextObjectsSurroundings: the subtracted object is empty, this should never happen"<<std::endl;
		else outputMap->AddLabelObject(subtractedLabelObject);
	}

	return true;
}




/** Compute the intersection between 2 label objects (assumed to be related to the same image frame...)
* returns the number of intersecting pixels.
* The input objects are assumed to be in optimal representation (no pixels belong to multiple segments, segments are sorted, ...)
* The output is optimally represented.
*/
template <class TLabelObject>
unsigned int GetLabelObjectIntersection(const TLabelObject *obj1, const TLabelObject *obj2, TLabelObject *outputObj) {
	unsigned int ind1=0, ind2=0;

	outputObj->Clear();
	outputObj->SetLabel(obj1->GetLabel());
	unsigned int nbIntersectingPixels = 0;
	bool sameRow;
	unsigned end1, end2, minEnd;

	while ( (ind1<obj1->GetNumberOfLines()) && (ind2<obj2->GetNumberOfLines()) ) {
		//get both lines.
		const TLabelObject::LineType &l1 = obj1->GetLine(ind1);
		const TLabelObject::LineType &l2 = obj2->GetLine(ind2);
		const TLabelObject::IndexType & idx1 = l1.GetIndex();
		const TLabelObject::IndexType & idx2 = l2.GetIndex();

		//check whether they intersect, and save the segment if they do, and identify in which order they are
		//the order is based on the row (indices>0), and then if they are on the same row, on the end-point of the segment.
		sameRow=true;
		for (unsigned d=TLabelObject::ImageDimension-1 ; d>0 ; d--) {
			if (idx1[d]!=idx2[d]) { //if they do not intersect, advance the 'smallest' line
				sameRow=false; 
				if (idx1[d]<idx2[d]) ind1++;
				else                 ind2++;
				break; 
			}
		}
		if (sameRow) { //the 2 segments are on the same row ; now check for intersection
			end1 = idx1[0] + l1.GetLength(); //this index value is just OUTSIDE the line!
			end2 = idx2[0] + l2.GetLength(); //this index value is just OUTSIDE the line!
			//in all cases, advance the 'smallest' line already.
			if (end1<end2) { minEnd = end1; ind1++;}
			else           { minEnd = end2; ind2++;}

			if (idx1[0]<idx2[0]) {
				if (idx2[0]<end1) { //intersection
					outputObj->AddLine(idx2, minEnd-idx2[0]); nbIntersectingPixels+=minEnd-idx2[0];
				}
				//else { }//no intersection... nothing else to do.
			}
			else {
				if (idx1[0]<end2) { //intersection
					outputObj->AddLine(idx1, minEnd-idx1[0]); nbIntersectingPixels+=minEnd-idx1[0];
				}
				//else { }//no intersection... nothing else to do.
			}
		}
		//else {} //if the segments are not on the same row, there is nothing else to do.
	}

	return nbIntersectingPixels;
}





/** Compute the set-difference between 2 label objects (assumed to be related to the same image frame...)
* returns the number of pixels in the output object.
* The input objects are assumed to be in optimal representation (no pixels belong to multiple segments, segments are sorted, ...)
* The output is optimally represented.
*/
template <class TLabelObject>
unsigned int GetLabelObjectDifference(const TLabelObject *obj1, const TLabelObject *obj2, TLabelObject *outputObj) {
	outputObj->Clear();
	outputObj->SetLabel(obj1->GetLabel());
	unsigned int nbIntersectingPixels = 0;
	bool sameRow;
	unsigned end1, end2;
	TLabelObject::IndexType tmpStartIndex; unsigned tmpEndIndex;

	unsigned int ind1=0, ind2=0;
	tmpStartIndex = obj1->GetLine(ind1).GetIndex(); tmpEndIndex = tmpStartIndex[0] + obj1->GetLine(ind1).GetLength();

	while ( (ind1<obj1->GetNumberOfLines()) && (ind2<obj2->GetNumberOfLines()) ) {

		//get both lines.
		const TLabelObject::LineType &l1 = obj1->GetLine(ind1);
		const TLabelObject::LineType &l2 = obj2->GetLine(ind2);
		const TLabelObject::IndexType & idx1 = l1.GetIndex();
		const TLabelObject::IndexType & idx2 = l2.GetIndex();

		//check whether they intersect, and save the segment if they do, and identify in which order they are
		//the order is based on the row (indices>0), and then if they are on the same row, on the end-point of the segment.
		sameRow=true;
		for (unsigned d=TLabelObject::ImageDimension-1 ; d>0 ; d--) {
			if (idx1[d]!=idx2[d]) { //if they do not intersect, advance the 'smallest' line
				sameRow=false; 
				if (idx1[d]<idx2[d]) { 
					//write whatever remains of the current line (if anything at all...)
					if (tmpEndIndex>=tmpStartIndex[0]) { outputObj->AddLine(tmpStartIndex, tmpEndIndex-tmpStartIndex[0]); nbIntersectingPixels+=tmpEndIndex-tmpStartIndex[0]; }
					//and move to the next line (if any)
					ind1++; if (ind1<obj1->GetNumberOfLines()) { tmpStartIndex = obj1->GetLine(ind1).GetIndex(); tmpEndIndex = tmpStartIndex[0] + obj1->GetLine(ind1).GetLength(); }
				}
				else ind2++;
				break; 
			}
		}
		if (sameRow) { //the 2 segments are on the same row ; now check for intersection
			end1 = idx1[0] + l1.GetLength(); //this index value is just OUTSIDE the line!
			end2 = idx2[0] + l2.GetLength(); //this index value is just OUTSIDE the line!

			//minEnd = std::min(end1, end2);

			if (idx1[0]<idx2[0]) {
				if (idx2[0]<end1) { //intersection => shorten the line on the right side
					tmpEndIndex = idx2[0];
					//check whether there is a second segment appearing on the right side!
					if (end1>end2) {
						//if so, write the current segment (not necessary to check...)
						outputObj->AddLine(tmpStartIndex, tmpEndIndex-tmpStartIndex[0]); nbIntersectingPixels+=tmpEndIndex-tmpStartIndex[0];
						//and start a new one.
						tmpStartIndex[0] = end2; tmpEndIndex = end1;
					}

				}
				//else { }//no intersection... nothing else to do.
			}
			else {
				if (idx1[0]<end2) { //intersection => shorten the line on the left side
					tmpStartIndex[0] = end2;
				}
				//else { }//no intersection... nothing else to do.
			}

			//in all cases, advance the 'smallest' line already.
			if (end1<end2) { 
				if (tmpEndIndex>=tmpStartIndex[0]) { outputObj->AddLine(tmpStartIndex, tmpEndIndex-tmpStartIndex[0]); nbIntersectingPixels+=tmpEndIndex-tmpStartIndex[0]; }
				ind1++;
				if (ind1<obj1->GetNumberOfLines()) { tmpStartIndex = obj1->GetLine(ind1).GetIndex(); tmpEndIndex = tmpStartIndex[0] + obj1->GetLine(ind1).GetLength(); }
			}
			else ind2++;

		}
		//else {} //if the segments are not on the same row, there is nothing else to do.
	}

	//finish the remaining lines of obj1 if any.
	if (ind1<obj1->GetNumberOfLines()) {
		//finish the current line
		if (tmpEndIndex>=tmpStartIndex[0]) { outputObj->AddLine(tmpStartIndex, tmpEndIndex-tmpStartIndex[0]); nbIntersectingPixels+=tmpEndIndex-tmpStartIndex[0]; }
		ind1++;
		//and continue with the rest.
		while (ind1<obj1->GetNumberOfLines()) { 
			outputObj->AddLine( obj1->GetLine(ind1) ); nbIntersectingPixels+=obj1->GetLine(ind1).GetLength(); 
			ind1++;
		}
	}

	return nbIntersectingPixels;
}

} // namespace psciob
