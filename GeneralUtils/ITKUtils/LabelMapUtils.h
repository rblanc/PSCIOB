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
 * \file LabelMapUtils.h
 * \author Rémi Blanc 
 * \date 8. March 2012
*/

#ifndef LABELMAPUTILS_H_
#define LABELMAPUTILS_H_


#include "ITKUtils.h"
#include "VTKUtils.h"

namespace psciob {

/** Convert a label map to a different frame (already indicated as the largest possible region of the outputMap)
* the input and output maps must have the same spacing 
* objects that fall outside the output frame are discarded
* objects that fall partially outside the output frame are cropped accordingly
*/
template <class TInputLabelMap, class TOutputLabelMap>
void CastLabelMapToOtherFrame(TInputLabelMap *inputMap, TOutputLabelMap *outputMap);


/** Insert Single Object LabelMap into an existing labelMap with the proposed label, does not check whether this object already exists
* if several objects are present in singleObjectMap, only the first is inserting into mergingMap
* crops the single object if necessary
* both label maps must have the same spacing.
* if the proposed label already exists in the mergingMap, it is replaced by the new object.
*/
template <class TInputLabelMap, class TOutputLabelMap>
bool InsertSingleObjectLabelMapIntoAnother(TInputLabelMap *singleObjectMap, TOutputLabelMap *mergingMap, typename TOutputLabelMap::LabelType proposedLabel = 0);


/** Insert Single Object LabelMap into an existing labelMap with the proposed label, does not check whether this object already exists
* if several objects are present in singleObjectMap, only the first is inserting into mergingMap
* both label maps must have the same spacing.
* It is assumed that the object is fully inside the targetMap ; no checks are performed and no test were performed to check what happens if this is not the case...
*/
template <class TInputLabelMap, class TOutputLabelMap>
bool InsertSingleObjectLabelMapIntoAnother_FullyInside(TInputLabelMap *singleObjectMap, TOutputLabelMap *mergingMap, typename TOutputLabelMap::LabelType proposedLabel = 0);



/** Generate a LabelObject that represent a ball structuring element of specified radius */
template <class TLabelObject>
void GenerateBallStructuringLabelObject(const unsigned int radius, TLabelObject *seObject);


/** Dilate each object of the input label map independently one from the others
* using the specified structuring element (formatted as a labelobject)
*/
template <class TLabelMap>
bool DilateOffContextObjects(const TLabelMap *inputMap, const typename TLabelMap::LabelObjectType *SE, TLabelMap *outputMap);


/** Compute a LabelMap from the input LabelMap, which contains the same number of objects
* each output object corresponds to the (offcontext) surroundings of the input ones
* and which is obtained through the subtraction of the morphological dilation of the object with itself, using the specified structuring element (formatted as a labelobject)
*/
template <class TLabelMap>
bool GetOffContextObjectsSurroundings(const TLabelMap *inputMap, const typename TLabelMap::LabelObjectType *SE, TLabelMap *outputMap);


/** Tests whether 2 label objects intersect, returns true if so, false otherwise */
template <class TLabelObject>
bool TestLabelObjectIntersection(const TLabelObject *obj1, const TLabelObject *obj2);


/** Compute the intersection between 2 label objects (assumed to be related to the same image frame...)
* returns the number of intersecting pixels.
* The input objects are assumed to be in optimal representation (no pixels belong to multiple segments, segments are sorted, ...)
* The output is optimally represented.
*/
template <class TLabelObject>
unsigned int GetLabelObjectIntersection(const TLabelObject *obj1, const TLabelObject *obj2, TLabelObject *outputObj);


/** Compute the set-difference between 2 label objects (assumed to be related to the same image frame...)
* returns the number of pixels in the output object.
* The input objects are assumed to be in optimal representation (no pixels belong to multiple segments, segments are sorted, ...)
* The output is optimally represented.
*/
template <class TLabelObject>
unsigned int GetLabelObjectDifference(const TLabelObject *obj1, const TLabelObject *obj2, TLabelObject *outputObj);

} // namespace psciob

#include "LabelMapUtils.txx"

#endif //LABELMAPUTILS_H_