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
 * \file TransformUtils.h
 * \author Rémi Blanc 
 * \26. September 2011
*/

#ifndef TRANSFORMUTILS_H_
#define TRANSFORMUTILS_H_

#include "GeneralUtils.h"

#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkMatrix4x4.h>

#include <itkAffineTransform.h>
#include <itkResampleImageFilter.h>
#include <itkNearestNeighborInterpolateImageFunction.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkBSplineInterpolateImageFunction.h>

#include <vnl/algo/vnl_matrix_inverse.h>
#include <vnl/algo/vnl_determinant.h>


namespace psciob {


/** Types of image interpolation */
typedef enum {
	NEARESTNEIGHBOR, 
	LINEAR,
	SPLINE
} InterpolationType;


/** Compose transforms -> multiplies the two matrices 
* works both for rotation matrices, general transforms in homogeneous systems...
*/
vnl_matrix<double> ComposeTransforms(const vnl_matrix<double> &T1, const vnl_matrix<double> &T2);
/** same as ComposeTransforms, but InPlace, modifying the first input matrix */
void ComposeTransforms_InPlace(vnl_matrix<double> &T1, const vnl_matrix<double> &T2);

/** Computes the axis-oriented bounding box entirely containing the transformed input bounding box
* The input transformMatrix is expected to be in homogeneous coordinates
* Implemented for 2D or 3D spaces only.
*/
vnl_vector<double> TransformBoundingBox(const vnl_vector<double> &inputBB, const vnl_matrix<double> &transformMatrix);


/** Applies an affine transform specified by the homogeneous transformation matrix to a vtkPolyData 
*/
void AffineTransformVTKPolyData( vtkPolyData* inputPolyData, vnl_matrix<double> transformMatrix, vtkPolyData* outputPolyData);

/** Applies the affine transform specified by the homogeneous transformation matrix to an itkImage
* and the interpolation method corresponding to the specified code (see InterpolationType)
*/
template < class TImage >
void AffineTransformITKImage( TImage* inputImage, vnl_matrix<double> transformMatrix, InterpolationType code, TImage* outputImage ) {
	typedef TImage ImageType;
	static const unsigned int Dimension = ImageType::ImageDimension;

	typedef itk::AffineTransform<double, Dimension>          TransformType;
	typedef itk::ResampleImageFilter<ImageType, ImageType>   ResampleFilterType;
	typedef itk::InterpolateImageFunction<ImageType, double> InterpolatorType;

	TransformType::Pointer      itkTransform       = TransformType::New();
	ResampleFilterType::Pointer itkTransformFilter = ResampleFilterType::New();
	InterpolatorType::Pointer   itkInterpolator;

	switch (code) {
		case NEARESTNEIGHBOR: itkInterpolator = itk::NearestNeighborInterpolateImageFunction<ImageType, double>::New();
			break;
		case LINEAR: itkInterpolator = itk::LinearInterpolateImageFunction<ImageType, double>::New();
			break;
		case SPLINE: itkInterpolator = itk::BSplineInterpolateImageFunction<ImageType, double>::New();
			break;
	}
	itkTransformFilter->SetInterpolator(itkInterpolator); 
	itkTransformFilter->SetDefaultPixelValue( 0 );

	//Parameters of the transform
	vnl_matrix<double> inverseTransformMatrix = vnl_matrix_inverse<double>(transformMatrix);	
	itk::Matrix< double , Dimension , Dimension > mat ;
	itk::Vector< double , Dimension > trans;
	for (unsigned i=0 ; i<Dimension ; i++) {
		trans[i] = inverseTransformMatrix(i,Dimension);
		for (unsigned j=0 ; j<Dimension ; j++) { 
			mat[i][j] = inverseTransformMatrix(i,j); 
		}
	}
	itkTransform->SetMatrix( mat ) ;
	itkTransform->SetTranslation( trans ) ;

	itkTransformFilter->SetTransform( itkTransform );
	//output image: same spacing, and smallest bbox entirely containing the transformed image
	ImageType::SpacingType spacing = inputImage->GetSpacing(); 
	ImageType::PointType origin; ImageType::RegionType region;
	vnl_vector<double> bbox = BoundingBoxFromITKImage<ImageType>(inputImage);

	ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing<Dimension>( TransformBoundingBox( bbox, transformMatrix ), spacing.GetVnlVector(),  &origin, &region);

	//Parameters of the filter
	itkTransformFilter->SetOutputStartIndex( region.GetIndex() );
	itkTransformFilter->SetOutputSpacing( spacing );
	itkTransformFilter->SetOutputOrigin( origin );
	itkTransformFilter->SetSize( region.GetSize() );

	itkTransformFilter->SetInput( inputImage );
	itkTransformFilter->Modified();
	itkTransformFilter->UpdateLargestPossibleRegion();
	//actual computation
	//return itkTransformFilter->GetOutput();
	outputImage->Graft( itkTransformFilter->GetOutput() );
}


} // namespace psciob


#endif //TRANSFORMUTILS_H_