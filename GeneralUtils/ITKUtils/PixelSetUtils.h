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
 * \file PixelSetUtils.h
 * \author Rémi Blanc 
 * \date 3. April 2012
*/

#ifndef PIXELSETUTILS_H_
#define PIXELSETUTILS_H_

#include "ObjectPixelSet.h"

namespace psciob {

/** Convert a label map containing a single object into a list of pixels (only the first object is processed)
* this fills a std::vector in such a way that the pixel adresses in the 'image' defined by the LabelMap are stored in order
* this is based on the coding: address = i + j*width (+ k*width*height + ... )
* return false if the pixelSet ends empty for some reason...
*/
template <class TLabelMap>
bool ConvertSingleObjectLabelMapToPixelList( const TLabelMap *singleObjectMap, std::vector<unsigned long> &pixelList, typename TLabelMap::LabelObjectType::LabelType label = 0);



/** Convert a label containing a single object into a pixel set (only the first object is processed)
* into a PixelSet with the same grid as the labelMap
* return false if the pixelSet ends empty for some reason...
*/
template <class TLabelMap>
bool ConvertSingleObjectLabelMapToPixelSet( const TLabelMap *singleObjectMap, ObjectPixelSet *pixelSet, typename TLabelMap::LabelObjectType::LabelType label = 0);

/** Convert a label containing a single object into a pixel set (only the first object is processed), with respect to the speicified grid 
* The spacing must be the same in the label map and in the specified input
* Portions of the input object which fall outside the specified grid are cropped.
* return false if the pixelSet ends empty for some reason...
*/
template <class TLabelMap>
bool ConvertSingleObjectLabelMapToPixelSet( const TLabelMap *singleObjectMap, ObjectPixelSet *pixelSet, 
										   typename TLabelMap::PointType origin, typename TLabelMap::SizeType size, typename TLabelMap::SpacingType spacing);
} // namespace psciob

#include "PixelSetUtils.txx"

#endif //PIXELSETUTILS_H_