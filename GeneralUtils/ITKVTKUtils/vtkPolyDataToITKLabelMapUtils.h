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
 * \file vtkPolyDataToITKLabelMapUtils.h
 * \author Rémi Blanc 
 * \date 1. March 2012
*/

#ifndef VTKPOLYDATATOITKLABELMAPUTILS_H_
#define VTKPOLYDATATOITKLABELMAPUTILS_H_


#include "ITKUtils.h"
#include "VTKUtils.h"

#include "itkLabelObject.h"
#include "itkLabelMap.h"
#include "itkLabelMapToLabelImageFilter.h"
#include "itkLabelMapToBinaryImageFilter.h"

#include "vtkPolyDataToITK3DUCharLabelMap.h"
#include "vtkPolyDataToITK3DUShortLabelMap.h"
#include "vtkPolyDataToITK3DUIntLabelMap.h"
#include "vtkPolyDataToITK2DUCharLabelMap.h"
#include "vtkPolyDataToITK2DUShortLabelMap.h"
#include "vtkPolyDataToITK2DUIntLabelMap.h"

namespace psciob {


typedef itk::LabelObject<unsigned int, 2> LabelObject2DUIntType;
typedef itk::LabelObject<unsigned char, 2> LabelObject2DUCharType;
typedef itk::LabelObject<unsigned short, 2> LabelObject2DUShortType;

typedef itk::LabelObject<unsigned int, 3> LabelObject3DUIntType;
typedef itk::LabelObject<unsigned char, 3> LabelObject3DUCharType;
typedef itk::LabelObject<unsigned short, 3> LabelObject3DUShortType;

typedef itk::LabelMap<LabelObject2DUIntType> LabelMap2DUIntType;
typedef itk::LabelMap<LabelObject2DUCharType> LabelMap2DUCharType;
typedef itk::LabelMap<LabelObject2DUShortType> LabelMap2DUShortType;

typedef itk::LabelMap<LabelObject3DUIntType> LabelMap3DUIntType;
typedef itk::LabelMap<LabelObject3DUCharType> LabelMap3DUCharType;
typedef itk::LabelMap<LabelObject3DUShortType> LabelMap3DUShortType;


struct vertice2D {
	double y0,x0,y1; //x0, y0 : indices of the current vertex ;  y1, m => indicate the next vertex (y0<y1)
	double m;		 //m <=> direction de la normale
};

struct vertice2D_less { 
	vertice2D_less(const std::vector<vertice2D> v) : vert(v) {}
	bool operator()(const size_t a, const size_t b) const	{ return vert[a].y0 < vert[b].y0; }
	const std::vector<vertice2D> vert;
};


/** Convert a vtkPolyData to a binary itkImage 
* The vtkPolyData is assumed to be 2D (only the first 2 coordinates being used) and convex
* Furthermore, 2 points with consecutive indices are assumed to be connected (and first is connected to last...)
* The image is defined with respect to the minimal bounding box containing the object (with the convention that coordinate 0 is center of a pixel)
*/
void VTK2DConvexPolyDataToITKImage( vtkPolyData *polydata, itk::Image<unsigned char, 2> *image, itk::Image<unsigned char, 2>::SpacingType spacing );

/** Convert a vtkPolyData to a binary itkImage (unsigned char)
* The image is defined with respect to the minimal bounding box containing the object (with the convention that coordinate 0 is center of a pixel)
*/
template <unsigned VDimension>
void VTKPolyDataToITKImage( vtkPolyData *polydata, itk::Image<unsigned char, VDimension> *image, typename itk::Image<unsigned char, VDimension>::SpacingType spacing );


/** Convert a vtkPolyData to a itkLabelMap containing a single itkLabelObject with label value 1
* The vtkPolyData is assumed to be 2D in the XY plane (only the first 2 coordinates being used) and convex
* Furthermore, 2 points with consecutive indices are assumed to be connected (and first is connected to last...)
* The map is defined with respect to the minimal bounding box containing the object (with the convention that coordinate 0 is center of a pixel)
* \attention The spacing of the label map must be defined beforehand
* the input bounding box may be modifed to fit the image bounding box
* \warning setting a non-empty bbox has not been tested, strange things may happen if it is too small
*/
template <class TLabelMap>
void VTK2DConvexPolyDataToLabelMap( vtkPolyData *polydata, TLabelMap *outputMap, typename TLabelMap::SpacingType spacing );

/** Convert a vtkPolyData to a itkLabelMap containing a single itkLabelObject with label value 1
* This interface is generic, but the actual code is different for 2D and 3D cases ; and only defined for a few types (unsigned char, unsigned short or unsigned int)
* The map is defined with respect to the minimal bounding box containing the object (with the convention that coordinate 0 is center of a pixel)
* \attention The map spacing must be defined beforehand
* the input bounding box may be modifed to fit the image bounding box
* \warning setting a non-empty bbox has not been tested, strange things may happen if it is too small
*/
template <class TLabelMap>
void VTKPolyDataToLabelMap( vtkPolyData *polydata, TLabelMap *outputMap, typename TLabelMap::SpacingType spacing ) {
	throw DeformableModelException("VTKPolyDataToLabelMap: only valid for dimension 2 or 3, and types unsigned char, unsigned short or unsigned int");
}


template <>
void VTKPolyDataToLabelMap<LabelMap2DUIntType>( vtkPolyData *polydata, LabelMap2DUIntType *outputMap, typename LabelMap2DUIntType::SpacingType spacing );

template <>
void VTKPolyDataToLabelMap<LabelMap2DUCharType>( vtkPolyData *polydata, LabelMap2DUCharType *outputMap, typename LabelMap2DUCharType::SpacingType spacing );

template <>
void VTKPolyDataToLabelMap<LabelMap2DUShortType>( vtkPolyData *polydata, LabelMap2DUShortType *outputMap, typename LabelMap2DUShortType::SpacingType spacing );

template <>
void VTKPolyDataToLabelMap<LabelMap3DUIntType>( vtkPolyData *polydata, LabelMap3DUIntType *outputMap, typename LabelMap3DUIntType::SpacingType spacing );

template <>
void VTKPolyDataToLabelMap<LabelMap3DUCharType>( vtkPolyData *polydata, LabelMap3DUCharType *outputMap, typename LabelMap3DUCharType::SpacingType spacing );

template <>
void VTKPolyDataToLabelMap<LabelMap3DUShortType>( vtkPolyData *polydata, LabelMap3DUShortType *outputMap, typename LabelMap3DUShortType::SpacingType spacing );

} // namespace psciob

#include "vtkPolyDataToITKLabelMapUtils.txx"

#endif //VTKPOLYDATATOITKLABELMAPUTILS_H_