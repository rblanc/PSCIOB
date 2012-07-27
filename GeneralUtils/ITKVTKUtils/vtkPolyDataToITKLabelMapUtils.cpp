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
* \file vtkPolyDataToITKLabelMapUtils.cpp
* \author Rémi Blanc 
* \date 1. March 2012
*/

#include "ITKUtils.h"
#include "vtkPolyDataToITKLabelMapUtils.h"

using namespace psciob;

/** Convert a vtkPolyData to a binary itkImage 
* The vtkPolyData is assumed to be 2D (only the first 2 coordinates being used) and convex
* Furthermore, 2 points with consecutive indices are assumed to be connected (and first is connected to last...)
* The image is defined with respect to the minimal bounding box containing the object (with the convention that coordinate 0 is center of a pixel)
*/
void psciob::VTK2DConvexPolyDataToITKImage( vtkPolyData *polydata, itk::Image<unsigned char, 2> *image, itk::Image<unsigned char, 2>::SpacingType spacing ) {
	typedef itk::Image<unsigned char, 2> ImageType;
	typedef itk::LabelObject<unsigned char, 2> LabelObjectType;
	typedef itk::LabelMap<LabelObjectType> LabelMapType;
	LabelMapType::Pointer labelMap = LabelMapType::New();

	VTK2DConvexPolyDataToLabelMap<LabelMapType>(polydata, labelMap, spacing);
	
	typedef itk::LabelMapToLabelImageFilter< LabelMapType, ImageType> LabelMapToLabelImageFilterType;
	LabelMapToLabelImageFilterType::Pointer labelMapToLabelImageFilter = LabelMapToLabelImageFilterType::New();
	labelMapToLabelImageFilter->SetInput(labelMap);
	labelMapToLabelImageFilter->Update();

	image = labelMapToLabelImageFilter->GetOutput();
}

/** Template specialization wrapping the vtkPolyDataToImageStencil, so that it also generates (modifies) an itkLabelMap */
template <>
void psciob::VTKPolyDataToLabelMap<LabelMap3DUCharType>( vtkPolyData *polydata, LabelMap3DUCharType *outputMap, LabelMap3DUCharType::SpacingType spacing) {
	// Set spacing and physical extent
	outputMap->SetSpacing(spacing);
	vnl_vector<double> bbox(6);
	double bounds[6]; polydata->GetBounds(bounds);
	for (unsigned i=0 ; i<3 ; i++) {
		bbox(2*i)   = std::min<double>(bounds[2*i], bounds[2*i+1]);
		bbox(2*i+1) = std::max<double>(bounds[2*i], bounds[2*i+1]);
	}

	// Compute and set image meta-data
	LabelMap3DUCharType::PointType origin;  LabelMap3DUCharType::RegionType region;
	ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing(bbox, spacing.GetVnlVector(), &origin, &region); 
	bbox = BoundingBoxFromITKImageInformation<LabelMap3DUCharType>(origin, spacing, region);

	outputMap->SetOrigin(origin); outputMap->SetRegions(region);
	const LabelMap3DUCharType::SizeType &size = region.GetSize();

	// Create the LabelObject that will host the object representation
	LabelMap3DUCharType::LabelObjectType::Pointer labelObject = LabelMap3DUCharType::LabelObjectType::New();
	labelObject->SetLabel(1); outputMap->AddLabelObject(labelObject);
	const LabelMap3DUCharType::IndexType &index = region.GetIndex();

	// Convert vtkPolyData to vtkImageStencilData
	vtkSmartPointer<vtkPolyDataToITK3DUCharLabelMap> poly2labelMap = vtkSmartPointer<vtkPolyDataToITK3DUCharLabelMap>::New();
	poly2labelMap->SetLabelMap( outputMap ); poly2labelMap->SetInput( polydata );
	poly2labelMap->SetOutputOrigin(origin[0], origin[1], origin[2]);
	poly2labelMap->SetOutputSpacing(spacing[0], spacing[1], spacing[2]);
	poly2labelMap->SetOutputWholeExtent(index[0], index[0]+size[0]-1, index[1], index[1]+size[1]-1, index[2], index[2]+size[2]-1);

	poly2labelMap->Update();

	outputMap = poly2labelMap->GetLabelMap();
}


template <>
void psciob::VTKPolyDataToLabelMap<LabelMap3DUShortType>( vtkPolyData *polydata, LabelMap3DUShortType *outputMap, LabelMap3DUShortType::SpacingType spacing) {
	// Get spacing and physical extent
	outputMap->SetSpacing(spacing);
	vnl_vector<double> bbox(6);
	double bounds[6]; polydata->GetBounds(bounds);
	for (unsigned i=0 ; i<3 ; i++) {
		bbox(2*i)   = std::min<double>(bounds[2*i], bounds[2*i+1]);
		bbox(2*i+1) = std::max<double>(bounds[2*i], bounds[2*i+1]);
	}


	// Compute and set image meta-data
	LabelMap3DUShortType::PointType origin;  LabelMap3DUShortType::RegionType region;
	ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing(bbox, spacing.GetVnlVector(), &origin, &region); 
	bbox = BoundingBoxFromITKImageInformation<LabelMap3DUShortType>(origin, spacing, region);

	outputMap->SetOrigin(origin); outputMap->SetRegions(region);
	const LabelMap3DUShortType::SizeType &size = region.GetSize();

	// Create the LabelObject that will host the object representation
	LabelMap3DUShortType::LabelObjectType::Pointer labelObject = LabelMap3DUShortType::LabelObjectType::New();
	labelObject->SetLabel(1); outputMap->AddLabelObject(labelObject);
	const LabelMap3DUShortType::IndexType &index = region.GetIndex();

	// Convert vtkPolyData to vtkImageStencilData
	vtkSmartPointer<vtkPolyDataToITK3DUShortLabelMap> poly2labelMap = vtkSmartPointer<vtkPolyDataToITK3DUShortLabelMap>::New();
	poly2labelMap->SetLabelMap( outputMap ); poly2labelMap->SetInput( polydata );
	poly2labelMap->SetOutputOrigin(origin[0], origin[1], origin[2]);
	poly2labelMap->SetOutputSpacing(spacing[0], spacing[1], spacing[2]);
	poly2labelMap->SetOutputWholeExtent(index[0], index[0]+size[0]-1, index[1], index[1]+size[1]-1, index[2], index[2]+size[2]-1);

	poly2labelMap->Update();

	outputMap = poly2labelMap->GetLabelMap();
}



template <>
void psciob::VTKPolyDataToLabelMap<LabelMap3DUIntType>( vtkPolyData *polydata, LabelMap3DUIntType *outputMap, LabelMap3DUIntType::SpacingType spacing) {
	// Get spacing and physical extent
	outputMap->SetSpacing(spacing);
	vnl_vector<double> bbox(6);
	double bounds[6]; polydata->GetBounds(bounds);
	for (unsigned i=0 ; i<3 ; i++) {
		bbox(2*i)   = std::min<double>(bounds[2*i], bounds[2*i+1]);
		bbox(2*i+1) = std::max<double>(bounds[2*i], bounds[2*i+1]);
	}

	// Compute and set image meta-data
	LabelMap3DUIntType::PointType origin;  LabelMap3DUIntType::RegionType region;
	ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing(bbox, spacing.GetVnlVector(), &origin, &region); 
	bbox = BoundingBoxFromITKImageInformation<LabelMap3DUIntType>(origin, spacing, region);

	outputMap->SetOrigin(origin); outputMap->SetRegions(region);
	const LabelMap3DUIntType::SizeType &size = region.GetSize();

	// Create the LabelObject that will host the object representation
	LabelMap3DUIntType::LabelObjectType::Pointer labelObject = LabelMap3DUIntType::LabelObjectType::New();
	labelObject->SetLabel(1); outputMap->AddLabelObject(labelObject);
	const LabelMap3DUIntType::IndexType &index = region.GetIndex();

	// Convert vtkPolyData to vtkImageStencilData
	vtkSmartPointer<vtkPolyDataToITK3DUIntLabelMap> poly2labelMap = vtkSmartPointer<vtkPolyDataToITK3DUIntLabelMap>::New();
	poly2labelMap->SetLabelMap( outputMap ); poly2labelMap->SetInput( polydata );
	poly2labelMap->SetOutputOrigin(origin[0], origin[1], origin[2]);
	poly2labelMap->SetOutputSpacing(spacing[0], spacing[1], spacing[2]);
	poly2labelMap->SetOutputWholeExtent(index[0], index[0]+size[0]-1, index[1], index[1]+size[1]-1, index[2], index[2]+size[2]-1);

	poly2labelMap->Update();

	outputMap = poly2labelMap->GetLabelMap();
}




template <>
void psciob::VTKPolyDataToLabelMap<LabelMap2DUCharType>( vtkPolyData *polydata, LabelMap2DUCharType *outputMap, LabelMap2DUCharType::SpacingType spacing) {
	// Get spacing and physical extent
	outputMap->SetSpacing(spacing);
	vnl_vector<double> bbox(4);
	double bounds[6]; polydata->GetBounds(bounds);
	for (unsigned i=0 ; i<2 ; i++) {
		bbox(2*i)   = std::min<double>(bounds[2*i], bounds[2*i+1]);
		bbox(2*i+1) = std::max<double>(bounds[2*i], bounds[2*i+1]);
	}

	// Compute and set image meta-data
	LabelMap2DUCharType::PointType origin;  LabelMap2DUCharType::RegionType region;
	ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing<2>(bbox, spacing.GetVnlVector(), &origin, &region); 
	bbox = BoundingBoxFromITKImageInformation<LabelMap2DUCharType>(origin, spacing, region);

	outputMap->SetOrigin(origin); outputMap->SetRegions(region);
	const LabelMap2DUCharType::SizeType &size = region.GetSize();

	// Create the LabelObject that will host the object representation
	LabelMap2DUCharType::LabelObjectType::Pointer labelObject = LabelMap2DUCharType::LabelObjectType::New();
	labelObject->SetLabel(1); outputMap->AddLabelObject(labelObject);
	const LabelMap2DUCharType::IndexType &index = region.GetIndex();

	// Convert vtkPolyData to vtkImageStencilData
	vtkSmartPointer<vtkPolyDataToITK2DUCharLabelMap> poly2labelMap = vtkSmartPointer<vtkPolyDataToITK2DUCharLabelMap>::New();
	poly2labelMap->SetLabelMap( outputMap ); poly2labelMap->SetInput( polydata );
	poly2labelMap->SetOutputOrigin(origin[0], origin[1], 0);
	poly2labelMap->SetOutputSpacing(spacing[0], spacing[1], 1);
	poly2labelMap->SetOutputWholeExtent(index[0], index[0]+size[0]-1, index[1], index[1]+size[1]-1, 0, 0);
	poly2labelMap->Update();

	outputMap = poly2labelMap->GetLabelMap();
}




template <>
void psciob::VTKPolyDataToLabelMap<LabelMap2DUShortType>( vtkPolyData *polydata, LabelMap2DUShortType *outputMap, LabelMap2DUShortType::SpacingType spacing) {
	// Get spacing and physical extent
	outputMap->SetSpacing(spacing);
	vnl_vector<double> bbox(4);
	double bounds[6]; polydata->GetBounds(bounds);
	for (unsigned i=0 ; i<2 ; i++) {
		bbox(2*i)   = std::min<double>(bounds[2*i], bounds[2*i+1]);
		bbox(2*i+1) = std::max<double>(bounds[2*i], bounds[2*i+1]);
	}

	// Compute and set image meta-data
	LabelMap2DUShortType::PointType origin;  LabelMap2DUShortType::RegionType region;
	ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing(bbox, spacing.GetVnlVector(), &origin, &region); 
	bbox = BoundingBoxFromITKImageInformation<LabelMap2DUShortType>(origin, spacing, region);

	outputMap->SetOrigin(origin); outputMap->SetRegions(region);
	const LabelMap2DUShortType::SizeType &size = region.GetSize();

	// Create the LabelObject that will host the object representation
	LabelMap2DUShortType::LabelObjectType::Pointer labelObject = LabelMap2DUShortType::LabelObjectType::New();
	labelObject->SetLabel(1); outputMap->AddLabelObject(labelObject);
	const LabelMap2DUShortType::IndexType &index = region.GetIndex();

	// Convert vtkPolyData to vtkImageStencilData
	vtkSmartPointer<vtkPolyDataToITK2DUShortLabelMap> poly2labelMap = vtkSmartPointer<vtkPolyDataToITK2DUShortLabelMap>::New();
	poly2labelMap->SetLabelMap( outputMap ); poly2labelMap->SetInput( polydata );
	poly2labelMap->SetOutputOrigin(origin[0], origin[1], 0);
	poly2labelMap->SetOutputSpacing(spacing[0], spacing[1], 1);
	poly2labelMap->SetOutputWholeExtent(index[0], index[0]+size[0]-1, index[1], index[1]+size[1]-1, 0, 0);

	poly2labelMap->Update();

	outputMap = poly2labelMap->GetLabelMap();
}




template <>
void psciob::VTKPolyDataToLabelMap<LabelMap2DUIntType>( vtkPolyData *polydata, LabelMap2DUIntType *outputMap, LabelMap2DUIntType::SpacingType spacing) {
	// Get spacing and physical extent
	outputMap->SetSpacing(spacing);
	vnl_vector<double> bbox(4);
	double bounds[6]; polydata->GetBounds(bounds);
	for (unsigned i=0 ; i<2 ; i++) {
		bbox(2*i)   = std::min<double>(bounds[2*i], bounds[2*i+1]);
		bbox(2*i+1) = std::max<double>(bounds[2*i], bounds[2*i+1]);
	}

	// Compute and set image meta-data
	LabelMap2DUIntType::PointType origin;  LabelMap2DUIntType::RegionType region;
	ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing(bbox, spacing.GetVnlVector(), &origin, &region); 
	bbox = BoundingBoxFromITKImageInformation<LabelMap2DUIntType>(origin, spacing, region);

	outputMap->SetOrigin(origin); outputMap->SetRegions(region);
	const LabelMap2DUIntType::SizeType &size = region.GetSize();

	// Create the LabelObject that will host the object representation
	LabelMap2DUIntType::LabelObjectType::Pointer labelObject = LabelMap2DUIntType::LabelObjectType::New();
	labelObject->SetLabel(1); outputMap->AddLabelObject(labelObject);
	const LabelMap2DUIntType::IndexType &index = region.GetIndex();

	// Convert vtkPolyData to vtkImageStencilData
	vtkSmartPointer<vtkPolyDataToITK2DUIntLabelMap> poly2labelMap = vtkSmartPointer<vtkPolyDataToITK2DUIntLabelMap>::New();
	poly2labelMap->SetLabelMap( outputMap ); poly2labelMap->SetInput( polydata );
	poly2labelMap->SetOutputOrigin(origin[0], origin[1], 0);
	poly2labelMap->SetOutputSpacing(spacing[0], spacing[1], 1);
	poly2labelMap->SetOutputWholeExtent(index[0], index[0]+size[0]-1, index[1], index[1]+size[1]-1, 0, 0);

	poly2labelMap->Update();

	outputMap = poly2labelMap->GetLabelMap();
}
