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
 * \file ITKUtils.h
 * \author Rémi Blanc 
 * \date 29. August 2011
*/

#ifndef ITKUTILS_H_
#define ITKUTILS_H_


#include <itkImageFileWriter.h>
#include <itkImageFileReader.h>
#include "itkImage.h"
#include "itkImportImageFilter.h"
#include "itkImageRegionIterator.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkImageRegionConstIterator.h"
#include "itkConstNeighborhoodIterator.h"
#include "itkMatrix.h"
#include "itkVector.h"
#include "itkDataObject.h"

#include "itkRescaleIntensityImageFilter.h"

#include "TransformUtils.h"
#include "3DTransformUtils.h"

#include "BoundingBoxesUtils.h"

namespace psciob {

/**
 * Reads the image at 'filename', and fills the corresponding pointer
*/
template < class  TImageType>
typename TImageType::Pointer ReadITKImageFromFile(std::string filename, double sp = 1.0 );

/**
 * Generates an itk image out of a raw pointer
 * Performs rescaling to the requested type, with rescaling as necessary
 * Rescaling is switched off if outMin>=outMax
*/
template < class  TInputPixel, class  TOutputPixel >
typename itk::Image<TOutputPixel, 2>::Pointer Generate2DITKImageRescaleFromBuffer(TInputPixel *buffer, TOutputPixel outMin, TOutputPixel outMax, 
																				  int w, int h, double sp_x = 1.0, double sp_y = 1.0, 
																				  double orig_x = 0.0, double orig_y = 0.0);
/**
 * Writes an image into a file with format specified in the filename
*/
template < class  TImageType>
void WriteITKImageToFile(std::string filename, TImageType *img);

/**
 * Writes a gray level image into a file with format specified in the filename
 * the intensity are rescaled to [0-255] for a nice display
*/
template < class  TImageType>
void Write2DGreyLevelRescaledImageToFile(std::string filename, TImageType *img);


/**
 * Computes the information desribing the grid of voxel which completely contains the physical bounding box
 * due to spacing, the grid of voxel may cover a slightly larger physical extent, but the input BB is included in the ImageBB
 * the physical point (0,0,0) is at the center of an element (which may be outside the actual grid)
*/
template <unsigned int VDimension> 
void ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing(vnl_vector<double> pBB, vnl_vector<double> sp, itk::Point<double, VDimension> *origin, itk::ImageRegion<VDimension> *region);

/**
 * Allocates a grid of voxel which completely contains the physical bounding box
 * Due to spacing, the grid of voxel may cover a slightly larger physical extent, but the input BB is included in the ImageBB
 * The grid is positioned in such a way the the point with coordinate (0,0,...,0) is at the center of a grid element
*/
template <class TImageType> 
void AllocateITKImageFromPhysicalBoundingBoxAndSpacing(vnl_vector<double> pBB, vnl_vector<double> sp, TImageType *img);

/**
 * Allocates a grid of voxel which completely contains the physical bounding box
 * the grid is positioned in such a way 
 * due to spacing, the grid of voxel may cover a slightly larger physical extent, but the input BB is included in the ImageBB
*/
template <class TImageType> 
void AllocateITKImageFromPhysicalBoundingBoxAndSpacing(vnl_vector<double> pBB, typename TImageType::SpacingType sp, TImageType *img);

/**
 * Fully clones an image, allocating the memory, and copying the LargestPossibleRegion
*/
template < class  TImageType>
void CloneITKImage(const TImageType *inputImage, TImageType *outputImage);

/**
 * Gets the physical bounding box of the voxel grid related to the itkImage
*/
template <class TImageType> 
inline 
vnl_vector<double> BoundingBoxFromITKImage(TImageType *inputImage);

/**
 * Gets the physical bounding box of the voxel grid related to a region in an itkImage
*/
template <class TImageType> 
inline 
vnl_vector<double> BoundingBoxFromITKImageRegion(TImageType *inputImage, typename TImageType::RegionType region);

/** 
 * Gets the physical bounding box of the voxel grid related to an origin, spacing and region 
*/
template <class TImageType> 
inline 
vnl_vector<double> BoundingBoxFromITKImageInformation(typename TImageType::PointType origin, typename TImageType::SpacingType spacing, typename TImageType::RegionType region);

/**
*/
void GetImageOriginAndSizeFromPhysicalBoundingBoxAndSpacing(vnl_vector<double> pBB, vnl_vector<double> sp, vnl_vector<double> *origin, vnl_vector<unsigned int> *size);

/**
 * Get the smallest region of an ITK image which includes a given physical bounding box
 * the bounding box of the region will generally be a bit larger due to pixel size
*/
template <class TImageType> 
inline 
bool ITKImageRegionFromBoundingBox(TImageType *Img, vnl_vector<double> bbox, typename TImageType::RegionType *region);

/**
 * Get the smallest region of an ITK image which includes a given physical bounding box
 * the bounding box of the region will generally be a bit larger due to pixel size
 * This version doesn't need the image to be created, just the origin, size and spacing
*/
template <unsigned int VDimension> 
inline 
bool ITKImageRegionFromBoundingBox(typename itk::Image<unsigned char, VDimension>::SpacingType spacing, 
								   typename itk::Image<unsigned char, VDimension>::PointType origin, 
								   typename itk::Image<unsigned char, VDimension>::SizeType size,
								   vnl_vector<double> bbox, 
								   typename itk::Image<unsigned char, VDimension>::RegionType *region);

/**
 * Get the region of intersection of two images I1 and I2
 * R1 and R2 are the corresponding regions respectively in I1 and I2 (physically, they correspond to the same intersection, but regions are specified with respect to the images resolution and origin...)
*/
template <class TImageType1, class TImageType2> 
inline 
bool ITKImagesIntersectionRegions(TImageType1 *I1, TImageType2 *I2, typename TImageType1::RegionType *R1, typename TImageType2::RegionType *R2);


// *** TODO: FINISH THE IMPLEMENTATION
// Get the set of regions of the intersection of two images I1 and I2 considering periodic boundary conditions
// R1 and R2 are the corresponding regions respectively in I1 and I2 (physically, they correspond to the same intersection, but regions are specified with respect to the images resolution and origin...)
// ***
template <class TImageType1, class TImageType2> 
inline 
bool ITKPeriodicBoundariesImagesIntersectionRegions(TImageType1 *I1, TImageType2 *I2, 
													typename std::vector<typename TImageType1::RegionType> *R1, 
													typename std::vector<typename TImageType2::RegionType> *R2);


//
//ConvertFlat3DITKImageTo2D
template <class T> 
void ConvertFlat3DITKImageTo2D(itk::Image<T,3> *I1, itk::Image<T,2> *I2);



//
//Convert2DITKImageToFlat3D
template <class T> 
void Convert2DITKImageToFlat3D(itk::Image<T,2> *I1, itk::Image<T,3> *I2, ObservationDirectionType dir = HORIZONTAL, double ref_coord = 0.0);

} // namespace psciob

#include "ITKUtils.txx"


#endif //ITKUTILS_H_