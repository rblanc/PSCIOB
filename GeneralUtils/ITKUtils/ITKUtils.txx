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
* \file ITKUtils.cpp
* \author Rémi Blanc 
* \date 29. August 2011
*/

#include "ITKUtils.h"

namespace psciob {

/**
* Reads the image at 'filename', and fills the corresponding pointer
*/
template < class  TImageType>
typename TImageType::Pointer ReadITKImageFromFile(std::string filename, double sp) {
	typedef itk::ImageFileReader< TImageType > ReaderType;
	ReaderType::Pointer imagereader = ReaderType::New();
	imagereader->SetFileName( filename.c_str() );

	try { imagereader->Update(); }
	catch( itk::ExceptionObject &e) {
		std::cerr << "Exception writing "<<filename<<": "<< std::endl;
		std::cerr << e << std::endl;
		return NULL;
	}

	TImageType::SpacingType spacing; 
	for (unsigned i=0 ; i<TImageType::ImageDimension ; i++) spacing[i] = sp;
	imagereader->GetOutput()->SetSpacing(spacing);

	return imagereader->GetOutput();
}

/**
* Generates an itk image out of a raw pointer
* Performs rescaling to the requested type, with rescaling as necessary
* Rescaling is switched off if outMin>=outMax
*/
template < class  TInputPixel, class  TOutputPixel >
typename itk::Image<TOutputPixel, 2>::Pointer Generate2DITKImageRescaleFromBuffer(TInputPixel *buffer, TOutputPixel outMin, TOutputPixel outMax, int w, int h,
																				  double sp_x, double sp_y, double orig_x, double orig_y) {
	typedef typename itk::Image<TOutputPixel, 2>	ImageType;

	ImageType::Pointer image = ImageType::New();

	ImageType::SizeType size; size[0] = w; size[1] = h;
	ImageType::IndexType start; start[0] = 0; start[1] = 0;
	ImageType::RegionType region ; region.SetSize( size ); region.SetIndex( start );
	image->SetRegions( region );

	ImageType::SpacingType spacing; spacing[0] = sp_x; spacing[1] = sp_y;
	image ->SetSpacing( spacing  );

	ImageType::PointType origin; origin[0] = orig_x; origin[1] = orig_y;
	image->SetOrigin( origin );

	try { image->Allocate(); }
	catch( itk::ExceptionObject &e) {
		std::cerr << "Exception allocating memory for image in Generate2DITKImageFromBuffer - buffer size: "<<w*h<<std::endl;
		std::cerr << e << std::endl;
		return NULL;
	}

	typedef itk::ImageRegionIterator< ImageType > IteratorType;
	IteratorType it( image, region );
	unsigned add;
	TInputPixel Vmin = buffer[0], Vmax = buffer[0]; 
	for ( add = 1 ; add<w*h ; add++ ) {
		Vmin = std::min<TInputPixel>(Vmin, buffer[add]);
		Vmax = std::max<TInputPixel>(Vmax, buffer[add]);
	}
	double diff = Vmax-Vmin, outDiff = outMax-outMin;
	if (outDiff<=TINY) {
		outDiff = diff;
		outMin = Vmin;
	}

	for ( add=0, it.GoToBegin(); !it.IsAtEnd(); ++it, ++add) { it.Set( static_cast<TOutputPixel>( outMin + outDiff*(buffer[add] - Vmin)/diff ) ); }

	return image;
}

/**
* Writes an image into a file with format specified in the filename
*/
template < class  TImageType>
void WriteITKImageToFile(std::string filename, TImageType *img) {
	typedef itk::ImageFileWriter< TImageType > WriterType;
	WriterType::Pointer imagewriter = WriterType::New();
	imagewriter->SetInput( img );
	imagewriter->SetFileName( filename.c_str() );
	try { imagewriter->Update(); }
	catch( itk::ExceptionObject &e) {
		std::cerr << "Exception writing "<<filename<<": "<< std::endl;
		std::cerr << e << std::endl;
		return;
	}
}

/**
* Writes a gray level image into a file with format specified in the filename
* the intensity are rescaled to [0-255] for a nice display
*/
template < class  TImageType>
void Write2DGreyLevelRescaledImageToFile(std::string filename, TImageType *img) {
	typedef itk::Image<unsigned char, 2> ImageType2;
	typedef itk::RescaleIntensityImageFilter<TImageType, ImageType2 > RescaleFilterType;
	RescaleFilterType::Pointer rescaler = RescaleFilterType::New();
	rescaler->SetOutputMinimum( 1 ); rescaler->SetOutputMaximum( 254 );
	rescaler->SetInput(img);
	typedef itk::ImageFileWriter< ImageType2 > WriterType;
	WriterType::Pointer imagewriter = WriterType::New();
	imagewriter->SetInput( rescaler->GetOutput() );
	imagewriter->SetFileName( filename.c_str() );
	try { imagewriter->Update(); }
	catch( itk::ExceptionObject &e) {
		std::cerr << "Exception writing "<<filename<<": "<< std::endl;
		std::cerr << e << std::endl;
		return;
	}
}



/**
* Computes the information desribing the grid of voxel which completely contains the physical bounding box
* due to spacing, the grid of voxel may cover a slightly larger physical extent, but the input BB is included in the ImageBB
* the physical point (0,0,0) is at the center of an element (which may be outside the actual grid)
*/
template <unsigned int VDimension> 
void ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing(vnl_vector<double> pBB, vnl_vector<double> sp, itk::Point<double, VDimension> *origin, itk::ImageRegion<VDimension> *region) {
	if ( (sp.size()!=VDimension) || (2*VDimension!=pBB.size()) ) {throw DeformableModelException("AllocateITKImageRegionsFromPhysicalBoundingBoxAndSpacing: inconsistent parameters ");}

	itk::Size<VDimension> size;	itk::Index<VDimension> startIndex;
		//I1 = std::floor( (interBBox( 2*i ) - origin[i]) / spacing[i] + 0.5 - TINY ); //test, used to be +TINY
		//I2 = std::floor( (interBBox(2*i+1) - origin[i]) / spacing[i] + 0.5 + TINY ); //test, used to be -TINY

	int N1, N2;
	for (unsigned int i=0 ; i<VDimension ; i++) {
		N1 = std::floor( pBB( 2*i )/sp[i] + 0.5 - TINY); //used to be +TINY
		N2 = std::floor( pBB(2*i+1)/sp[i] + 0.5 + TINY);  //used to be -TINY
		(*origin)[i] = N1*sp[i];
		startIndex[i] = 0;
		size[i] = max((unsigned int)1,(unsigned int)( 1 + N2 - N1 ) );
	}

	region->SetSize(size);	region->SetIndex(startIndex);
}


/**
 * Fully clones an image, allocating the memory, and copying the LargestPossibleRegion
*/
template < class  TImageType>
void CloneITKImage(const TImageType *inputImage, TImageType *outputImage) {
	outputImage->SetSpacing( inputImage->GetSpacing() );
	outputImage->SetOrigin( inputImage->GetOrigin() );
	outputImage->SetRegions( inputImage->GetLargestPossibleRegion() );

	try	{ outputImage->Allocate(); }
	catch (itk::ExceptionObject & e) { 
		std::cerr << "AllocateITKImageRegionsFromPhysicalBoundingBoxAndSpacing: failed to allocate image" << std::endl;
		std::cerr << e.GetDescription() << std::endl;
		std::cerr << e.GetLocation() << std::endl;
		return;
	}

	TImageType::PixelType *inBuf = const_cast<TImageType::PixelType *>(inputImage->GetBufferPointer());
	TImageType::PixelType *outBuf= outputImage->GetBufferPointer();

	for (unsigned add = 0 ; add<inputImage->GetLargestPossibleRegion().GetNumberOfPixels() ; add++) {
		outBuf[add] = inBuf[add];
	}
}


/**
* Allocates a grid of voxel which completely contains the physical bounding box
* Due to spacing, the grid of voxel may cover a slightly larger physical extent, but the input BB is included in the ImageBB
* The grid is positioned in such a way the the point with coordinate (0,0,...,0) is at the center of a grid element
*/
template <class TImageType> 
void AllocateITKImageFromPhysicalBoundingBoxAndSpacing(vnl_vector<double> pBB, vnl_vector<double> sp, TImageType *img) {
	TImageType::RegionType region;
	TImageType::PointType origin;
	ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing<TImageType::ImageDimension>(pBB, sp, &origin, &region);

	img->SetOrigin(origin);
	img->SetRegions(region);

	TImageType::SpacingType spacing; spacing.SetVnlVector(sp);
	img->SetSpacing(spacing);

	try	{ img->Allocate(); }
	catch (itk::ExceptionObject & e) { 
		std::cerr << "AllocateITKImageFromPhysicalBoundingBoxAndSpacing: failed to allocate image" << std::endl;
		std::cerr << e.GetDescription() << std::endl;
		std::cerr << e.GetLocation() << std::endl;
		return;
	}

}


/**
* Allocates a grid of voxel which completely contains the physical bounding box
* the grid is positioned in such a way 
* due to spacing, the grid of voxel may cover a slightly larger physical extent, but the input BB is included in the ImageBB
*/
template <class TImageType> 
void AllocateITKImageFromPhysicalBoundingBoxAndSpacing(vnl_vector<double> pBB, typename TImageType::SpacingType sp, TImageType *img) {
	AllocateITKImageFromPhysicalBoundingBoxAndSpacing(pBB, sp.GetVnlVector(), img);
}

/**
* Gets the physical bounding box of the voxel grid related to the itkImage
*/
template <class TImageType> 
vnl_vector<double> BoundingBoxFromITKImage(TImageType *inputImage) {
	vnl_vector<double> BB(2*TImageType::ImageDimension);

	const TImageType::PointType   & origin = inputImage->GetOrigin();
	const TImageType::SizeType    & size   = inputImage->GetLargestPossibleRegion().GetSize();
	const TImageType::SpacingType & sp     = inputImage->GetSpacing();

	for (unsigned i=0 ; i<TImageType::ImageDimension ; i++) {
		BB(2*i) = origin[i] - sp[i]/2.0;
		BB(2*i+1) = BB(2*i) + size[i]*sp[i];
	}
	return BB;
}
/**
* Gets the physical bounding box of the voxel grid related to a region in an itkImage
*/
template <class TImageType> 
vnl_vector<double> BoundingBoxFromITKImageRegion(TImageType *inputImage, typename TImageType::RegionType region) {
	vnl_vector<double> BB(2*TImageType::ImageDimension);

	const TImageType::PointType   &origin      = inputImage->GetOrigin();
	const TImageType::SpacingType &sp          = inputImage->GetSpacing();
	const TImageType::SizeType    &regionSize  = region.GetSize();
	const TImageType::IndexType   &regionIndex = region.GetIndex();

	for (unsigned i=0 ; i<TImageType::ImageDimension ; i++) {
		BB(2*i) = origin[i] - sp[i]/2.0 + regionIndex[i]*sp[i];
		BB(2*i+1) = BB(2*i) + regionSize[i]*sp[i];
	}
	return BB;
}


/** Gets the physical bounding box of the voxel grid related to an origin, spacing and region */
template <class TImageType> 
vnl_vector<double> BoundingBoxFromITKImageInformation(typename TImageType::PointType origin, typename TImageType::SpacingType spacing, typename TImageType::RegionType region) {
	vnl_vector<double> BB(2*TImageType::ImageDimension);

	const TImageType::SizeType  &regionSize  = region.GetSize();
	const TImageType::IndexType &regionIndex = region.GetIndex();

	for (unsigned i=0 ; i<TImageType::ImageDimension ; i++) {
		BB(2*i) = origin[i] - spacing[i]/2.0 + regionIndex[i]*spacing[i];
		BB(2*i+1) = BB(2*i) + regionSize[i]*spacing[i];
	}
	return BB;
}


/**
* Get the smallest region of an ITK image which includes a given physical bounding box
* the bounding box of the region will generally be a bit larger due to pixel size
*/
template <class TImageType> 
inline 
bool ITKImageRegionFromBoundingBox(TImageType *Img, vnl_vector<double> bbox, typename TImageType::RegionType *region) {
	if ( 2*TImageType::ImageDimension != bbox.size() ) throw DeformableModelException("ITKImageRegionFromBoundingBox: incompatible dimensions");

	vnl_vector<double> imageBBox = BoundingBoxFromITKImage<TImageType>(Img);
	vnl_vector<double> interBBox;
	if (!IntersectionBoundingBoxes(bbox, imageBBox, &interBBox)) return false;

	const TImageType::SizeType	  &imageSize   = Img->GetLargestPossibleRegion().GetSize();
	const TImageType::PointType	  &imageOrigin = Img->GetOrigin();	
	const TImageType::SpacingType &spacing     = Img->GetSpacing();

	TImageType::IndexType	regionStart;	
	TImageType::SizeType	regionSize;

	int I1, I2;
	for (unsigned i=0 ; i<TImageType::ImageDimension ; i++) {
		//convert physical point to an index, relative to the origin ; rounded so that the region always contains the borders of the box
		I1 = std::floor( (interBBox( 2*i ) - imageOrigin[i]) / spacing[i] + 0.5 - TINY ); //test, used to be +TINY
		I2 = std::floor( (interBBox(2*i+1) - imageOrigin[i]) / spacing[i] + 0.5 + TINY ); //test, used to be -TINY
		//now, get the start index & size
		regionStart[i] = max(0, I1 );
		regionSize[i] = min(1 + I2 - I1 , imageSize[i] - regionStart[i]); //test, used to be: regionSize[i] = max(1, 1 + I2 - I1 );
	}
	region->SetIndex( regionStart );
	region->SetSize( regionSize );
	return true;
}

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
								   typename itk::Image<unsigned char, VDimension>::RegionType *region)
{
	if ( 2*VDimension != bbox.size() ) throw DeformableModelException("ITKImageRegionFromBoundingBox: incompatible dimensions");

	vnl_vector<double> imageBBox(2*VDimension);
	for (unsigned i=0 ; i<VDimension ; i++) { 
		imageBBox(2*i) = origin[i] - spacing[i]/2.0;
		imageBBox(2*i+1) = imageBBox(2*i) + size[i]*spacing[i];
	}

	vnl_vector<double> interBBox;
	if (!IntersectionBoundingBoxes(bbox, imageBBox, &interBBox)) return false;

	itk::Image<unsigned char, VDimension>::IndexType	regionStart;	
	itk::Image<unsigned char, VDimension>::SizeType		regionSize;

	int I1, I2;

	for (unsigned i=0 ; i<VDimension ; i++) {
		//convert physical point to an index, relative to the origin ; rounded so that the region always contains the borders of the box
		I1 = std::floor( (interBBox( 2*i ) - origin[i]) / spacing[i] + 0.5 - TINY ); //test, used to be +TINY
		I2 = std::floor( (interBBox(2*i+1) - origin[i]) / spacing[i] + 0.5 + TINY ); //test, used to be -TINY

		//now, get the start index & size
		regionStart[i] = max(0, I1 );
		regionSize[i] = min(1 + I2 - I1 , size[i] - regionStart[i]); //test, used to be: regionSize[i] = max(1, 1 + I2 - I1 );
	}
	region->SetIndex( regionStart );
	region->SetSize( regionSize );

	return true;
}

/**
* Get the region of intersection of two images I1 and I2
* R1 and R2 are the corresponding regions respectively in I1 and I2 (physically, they correspond to the same intersection, but regions are specified with respect to the images resolution and origin...)
*/
template <class TImageType1, class TImageType2> 
inline 
bool ITKImagesIntersectionRegions(TImageType1 *I1, TImageType2 *I2, typename TImageType1::RegionType *R1, typename TImageType2::RegionType *R2) {
	unsigned int D = TImageType1::ImageDimension;
	if (TImageType2::ImageDimension!=D) throw DeformableModelException("Error in ITKImagesIntersectionRegions : can't intersect images of different dimensionality");
	unsigned int i;
	bool cropPossible = true;
	vnl_vector<double> BB1(2*D), BB2(2*D), interBB(2*D);
	BB1 = BoundingBoxFromITKImage<TImageType1>(I1);	BB2 = BoundingBoxFromITKImage<TImageType2>(I2);

	if (!IntersectionBoundingBoxes(BB1, BB2, &interBB)) return false;

	ITKImageRegionFromBoundingBox<TImageType1>(I1, interBB, R1);
	ITKImageRegionFromBoundingBox<TImageType2>(I2, interBB, R2);
	return true;
}


// *** TODO: FINISH THE IMPLEMENTATION
// Get the set of regions of the intersection of two images I1 and I2 considering periodic boundary conditions
// R1 and R2 are the corresponding regions respectively in I1 and I2 (physically, they correspond to the same intersection, but regions are specified with respect to the images resolution and origin...)
// ***
template <class TImageType1, class TImageType2> 
inline 
bool ITKPeriodicBoundariesImagesIntersectionRegions(TImageType1 *I1, TImageType2 *I2, typename std::vector<typename TImageType1::RegionType> *R1, typename std::vector<typename TImageType2::RegionType> *R2) {
	unsigned int D = TImageType1::ImageDimension;
	if (TImageType2::ImageDimension!=D) throw DeformableModelException("Error in ITKImagesIntersectionRegions : can't intersect images of different dimensionality");
	unsigned int i;
	vnl_vector<double> BB1(2*D), BB2(2*D), interBB(2*D);
	BB1 = BoundingBoxFromITKImage<TImageType1>(I1);	BB2 = BoundingBoxFromITKImage<TImageType2>(I2);

	std::vector<bool> wrap_min, wrap_max; unsigned nb_regions=1;
	for (i=0;i<D;i++) { //if the leftmost point of I1 is to the right of the rightmost point of I2, or vice versa, then intersection is empty
		if ( BB1(2*i)>BB2(2*i+1) ) {return false;}
		if ( BB1(2*i+1)<BB2(2*i) ) {return false;}
		interBB(2*i) = max(BB1(2*i),BB2(2*i));		
		interBB(2*i+1) = min(BB1(2*i+1),BB2(2*i+1));

		if (interBB(2*i)-min(BB1(2*i),BB2(2*i))<TINY)	wrap_min.push_back(true);
		else											wrap_min.push_back(false);

		if (interBB(2*i+1)-min(BB1(2*i+1),BB2(2*i+1))<TINY)	wrap_max.push_back(true);
		else												wrap_max.push_back(false);
		if (wrap_min(i) && wrap_max(i)) throw DeformableModelException("Error in ITKImagesIntersectionRegions : can't intersect images of different dimensionality");
		if (wrap_min(i) || wrap_max(i)) { nb_regions*=2; }
	}

	TImageType1::IndexType ind1, ind2;
	TImageType1::SizeType size1, size2;

	//TODO: FINISH THE IMPLEMENTATION -> extract the different intersections ; take care that the exact number of pixels is preserved among the different regions...

	//for (i=0;i<D;i++) { 
	//	//get the index of the left most point of the interBB in I1 and I2
	//	ind1[i] = floor( (interBB(2*i)-BB1(2*i))/I1->GetSpacing()[i] + TINY);
	//	ind2[i] = floor( (interBB(2*i)-BB2(2*i))/I2->GetSpacing()[2] + TINY);
	//	//get the size of interBB
	//	size1[i] = max( (unsigned int)1, min( (unsigned int)(I2->GetLargestPossibleRegion().GetSize()[i]),(unsigned int)(1+ceil(interBB(2*i+1)/I1->GetSpacing()[i]-TINY-0.5) - floor(interBB(2*i)/I1->GetSpacing()[i]+TINY+0.5))));
	//	size2[i] = size1[i];
	//}

	//(*R1).SetIndex(ind1);		(*R1).SetSize(size1);
	//(*R2).SetIndex(ind2);		(*R2).SetSize(size2);

	return true;
}


//
//ConvertFlat3DITKImageTo2D
template <class T> 
void ConvertFlat3DITKImageTo2D(itk::Image<T,3> *I1, itk::Image<T,2> *I2) {
	typedef itk::Image<T,3> InputImageType;
	typedef itk::Image<T,2> OutputImageType;

	InputImageType::SpacingType inputSpacing = I1->GetSpacing();
	OutputImageType::SpacingType outputSpacing; 
	InputImageType::PointType inputOrigin = I1->GetOrigin();
	OutputImageType::PointType outputOrigin; 
	OutputImageType::RegionType region;
	InputImageType::SizeType inputSize = I1->GetLargestPossibleRegion().GetSize();
	OutputImageType::SizeType outputSize; 
	OutputImageType::IndexType start; start[0] = 0; start[1] = 0;
	unsigned int c0, c1, c2; //c0 is the first axis of the output image ; the value 0 corresponds to x axis of the input image, 1<->y , 2<->z
	if ( inputSize[0] == 1 ) { c0=1;c1=2;c2=0;} //image is flat on the x direction -> image lives on the (y,z) plane
	else {
		if ( inputSize[1] == 1 ) {c0=0;c1=2;c2=1;} //image is flat on the y direction -> image lives on the (x,z) plane
		else {
			if ( inputSize[2] == 1 ) {c0=0;c1=1;c2=2;} //image is on the (x,y) plane - input image is flat in the z direction
			else { throw DeformableModelException("Error in ConvertFlat3DITKImageTo2D: input image is not flat!"); }
		}
	}
	outputSpacing[0]= inputSpacing[c0];	outputSpacing[1]= inputSpacing[c1];
	outputOrigin[0] = inputOrigin[c0];	outputOrigin[1] = inputOrigin[c1];
	outputSize[0]   = inputSize[c0];	outputSize[1]   = inputSize[c1];

	region.SetSize(outputSize); region.SetIndex(start);
	I2->SetSpacing( outputSpacing );
	I2->SetOrigin( outputOrigin );
	I2->SetRegions(region);
	I2->Allocate();

	//fill the image...
	typedef itk::ImageRegionIteratorWithIndex< OutputImageType > OutputIteratorType;
	OutputIteratorType outputIt( I2, region );

	//WARNING: make sure the order of traversal is the same...
	typedef itk::ImageRegionIteratorWithIndex< InputImageType > InputIteratorType;
	InputIteratorType inputIt( I1, I1->GetLargestPossibleRegion() );

	for ( outputIt.GoToBegin(), inputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt, ++inputIt) {
		outputIt.Set(inputIt.Get());
	}

}




//
//Convert2DITKImageToFlat3D
template <class T> 
void Convert2DITKImageToFlat3D(itk::Image<T,2> *I1, itk::Image<T,3> *I2, ObservationDirectionType dir, double ref_coord) {
	typedef itk::Image<T,2> InputImageType;
	typedef itk::Image<T,3> OutputImageType;

	OutputImageType::SpacingType outputSpacing;
	OutputImageType::PointType outputOrigin;

	OutputImageType::RegionType region;
	InputImageType::SizeType inputSize = I1->GetLargestPossibleRegion().GetSize();
	InputImageType::SpacingType inputSpacing = I1->GetSpacing();
	InputImageType::PointType inputOrigin = I1->GetOrigin();
	OutputImageType::SizeType outputSize; 
	OutputImageType::IndexType start; start[0] = 0; start[1] = 0; start[2] = 0;
	unsigned int c0, c1, c2; //c0 is the first axis of the output image ; the value 0 corresponds to x axis of the input image, 1<->y , 2<->z
	switch(dir) {
		case HORIZONTAL: c0=0;c1=1;c2=2; break; // (x,y) plane -  normal = z
		case SAGITTAL:	 c0=1;c1=2;c2=0; break;	// (y,z) plane -- normal = x
		case CORONAL:	 c0=0;c1=2;c2=1; break;	// (x,z) plane -- normal = y
	}

	outputSpacing[c0] = inputSpacing[0];	outputSpacing[c1] = inputSpacing[1];	outputSpacing[c2] = 1;
	outputOrigin[c0] = inputOrigin[0];		outputOrigin[c1] = inputOrigin[1];		outputOrigin[c2] = ref_coord; //not 0, but slightly positive so that the border of the pixel is at 0
	outputSize[c0] = inputSize[0];			outputSize[c1] = inputSize[1];			outputSize[c2] = 1;

	region.SetSize(outputSize); region.SetIndex(start);
	I2->SetSpacing( outputSpacing );
	I2->SetOrigin( outputOrigin );
	I2->SetRegions(region);
	I2->Allocate();

	//fill the image...
	typedef itk::ImageRegionIteratorWithIndex< OutputImageType > OutputIteratorType;
	OutputIteratorType outputIt( I2, region );

	//WARNING: make sure the order of traversal is the same...
	typedef itk::ImageRegionIteratorWithIndex< InputImageType > InputIteratorType;
	InputIteratorType inputIt( I1, I1->GetLargestPossibleRegion() );

	for ( outputIt.GoToBegin(), inputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt, ++inputIt) {
		outputIt.Set(inputIt.Get());
	}

}

} // namespace psciob
