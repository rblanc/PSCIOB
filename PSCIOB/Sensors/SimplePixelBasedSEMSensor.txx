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
* \file SimplePixelBasedSEMSensor.txx
* \author Rémi Blanc 
* \date 28. September 2011
*/

#ifndef SIMPLEPIXELBASEDSEMSENSOR_TXX_
#define SIMPLEPIXELBASEDSEMSENSOR_TXX_

#include "SimplePixelBasedSEMSensor.h"

namespace psciob {

//IDEA: should this class maintain a full labelImage at all times?
//it would make a slight overhead, recomputing the LabelMap out of it
//but allow to update only small regions of the sensor instead of the full image all the time...
//elements for this second option are written & commented after the Update_data method

//CONSTRUCTOR
template<class TScene, class TOutputImage>
SimplePixelBasedSEMSensor<TScene, TOutputImage>::SimplePixelBasedSEMSensor() : ImageSensor_Base() {
	m_direction = HORIZONTAL; m_dirAxis0=0;m_dirAxis1=1;m_dirAxis2=2;
	m_poseParams.set_size(1); m_poseParams(0)=m_direction;

	GPF_ScExpDecrFunction<double, double>::Pointer exp_fct = GPF_ScExpDecrFunction<double, double>::New();	
	vnl_vector<double> params(2); params(0) = 250; params(1) = 1.0/10.0;
	exp_fct->SetParameters(params);
	m_function = exp_fct;	
	m_appearanceFunctionFlag = true;
	m_outputLabelImage = OutputLabelImageType::New();
}

/** Set the direction: Generic interface */ 
template<class TScene, class TOutputImage>
bool 
SimplePixelBasedSEMSensor<TScene, TOutputImage>::SetPoseParameters(const vnl_vector<double> &p) {
	if (p.size()!=GetNumberOfPoseParameters()) return false;
	if ( fabs( p(0) ) <TINY) { SetObservationDirectionType(HORIZONTAL);return true;}
	if ( fabs(p(0)-1) <TINY) { SetObservationDirectionType(SAGITTAL);  return true;}
	if ( fabs(p(0)-2) <TINY) { SetObservationDirectionType(CORONAL);   return true;}
	return false; //
}

/** Set the direction, specific to this class : HORIZONTAL / SAGITTAL / CORONAL */ 
template<class TScene, class TOutputImage>
void 
SimplePixelBasedSEMSensor<TScene, TOutputImage>::SetObservationDirectionType(ObservationDirectionType dir)	{
	m_poseParams(0) = dir ; m_orientationFlag=true;
	if (m_direction!=dir) { m_direction = dir; Modified(); } //if nothing really changed, don't set raise the ->Modified() flag
	switch(m_direction) { // 0 = first axis in 3D = X ; 1: second axis => Y ; Z = 3rd axis => Z
		case SAGITTAL:	 m_dirAxis0=1;m_dirAxis1=2;m_dirAxis2=0; break; //if the observation direction is SAGITTAL, then the axes of the output image correspond to the (y,z) plane in 3D
		case CORONAL:	 m_dirAxis0=0;m_dirAxis1=2;m_dirAxis2=1; break; //CORONAL -> (x,z) plane
		case HORIZONTAL: m_dirAxis0=0;m_dirAxis1=1;m_dirAxis2=2; break; //HORIZONTAL -> (x,y)
		default: throw DeformableModelException("SimplePixelBasedSEMSensor: direction parameter must be 0, 1 or 2 "); //should never happen
	}
}

/** bounding box of the sensed scene */
template<class TScene, class TOutputImage>
vnl_vector<double> 
SimplePixelBasedSEMSensor<TScene, TOutputImage>::GetSensedBoundingBox(const vnl_vector<double> &bbox) {
	vnl_vector<double> sensedBBox(2*TOutputImage::ImageDimension);
	sensedBBox(0) = bbox(2*m_dirAxis0);	sensedBBox(1) = bbox(2*m_dirAxis0+1);
	sensedBBox(2) = bbox(2*m_dirAxis1);	sensedBBox(3) = bbox(2*m_dirAxis1+1);
	return sensedBBox;
}


/** Off Context image of an object: as if it was alone in the scene 
* this labelMap is expressed with respect to the full sensor grid
* returns false if the object is not in the scene
*/
template<class TScene, class TOutputImage>
bool 
SimplePixelBasedSEMSensor<TScene, TOutputImage>::GetOffContextObjectLabelMap(ObjectInScene *objectPtr, OutputLabelMapType *offContextObjectLabelMap) {
	if (objectPtr->id == 0) return false;
	if (!m_imageAllocated) AllocateOutputImage();

	vnl_vector<double> sceneBBox = m_scene->GetSceneBoundingBox();

	offContextObjectLabelMap->ClearLabels();
	offContextObjectLabelMap->SetSpacing( m_outputSpacing );
	offContextObjectLabelMap->SetOrigin(  m_outputOrigin  );
	offContextObjectLabelMap->SetRegions( m_outputRegion  );

	//
	InputLabelObjectType *inputObject = m_scene->GetObjectLabelObject(objectPtr->id);
	InputLabelObjectType::IndexType index3D; InputLabelObjectType::LengthType length3D;

	//Create an empty label object for the output.
	OutputLabelObjectType::Pointer outputLabelObject = OutputLabelObjectType::New();
	OutputImageType::IndexType index2D;
	unsigned char *visited; long add; //only interested the the labelmap here, so distance is not important
	switch(m_direction) {
			case SAGITTAL: //SAGITTAL -> (y,z) plane				
				for (unsigned j=0 ; j<inputObject->GetNumberOfLines() ; j++) {
					index3D = inputObject->GetLine(j).GetIndex();
					//sagittal => 3D line in the x direction => the index is enough to proceed
					index2D[0] = index3D[1]; index2D[1] = index3D[2];
					outputLabelObject->AddIndex(index2D);
				}
				break;
			case CORONAL: //CORONAL -> (x,z) plane
				visited = (unsigned char *)calloc(m_outputRegion.GetNumberOfPixels(),sizeof(unsigned char));
				//browse the lines of the object and update the zbuffer accordingly.
				for (unsigned j=0 ; j<inputObject->GetNumberOfLines() ; j++) {
					index3D = inputObject->GetLine(j).GetIndex();
					index2D[0] = index3D[0]; index2D[1] = index3D[2];
					add = index2D[0] + index2D[1]*m_outputSize[0]; //address in the local buffer.
					for (unsigned k=0 ; k<inputObject->GetLine(j).GetLength() ; k++) {
						if (!visited[add]) { visited[add] = 1; outputLabelObject->AddIndex(index2D); }
						add++; index2D[0]++;
					}
				}
				free(visited);
				break;
			case HORIZONTAL: //HORIZONTAL -> (x,y)
				visited = (unsigned char *)calloc(m_outputRegion.GetNumberOfPixels(),sizeof(unsigned char));
				//browse the lines of the object and update the zbuffer accordingly.
				for (unsigned j=0 ; j<inputObject->GetNumberOfLines() ; j++) {
					index3D = inputObject->GetLine(j).GetIndex();
					index2D[0] = index3D[0]; index2D[1] = index3D[1];
					add = index2D[0] + index2D[1]*m_outputSize[0]; //address in the local buffer.
					for (unsigned k=0 ; k<inputObject->GetLine(j).GetLength() ; k++) {
						if (!visited[add]) { visited[add] = 1; outputLabelObject->AddIndex(index2D); }
						add++; index2D[0]++;
					}
				}
				free(visited);
				break;
	}

	outputLabelObject->Optimize(); //this may actually not be necessary
	outputLabelObject->SetLabel(objectPtr->id);
	offContextObjectLabelMap->AddLabelObject(outputLabelObject);

	return true;
}


/** Off Context image of an object: as if it was alone in the scene 
* The image and label maps are expressed relative to the smallest possible region of the sensor frame (to save memory and computation time...)
* 	OPTIMIZATION: different possibilities for optimization
*  - caching the images / labelMap for each object, exploiting particular DataContainer for the scene?
* returns false if the object is not in the scene
*/
template<class TScene, class TOutputImage>
bool 
SimplePixelBasedSEMSensor<TScene, TOutputImage>::GetOffContextObjectImage(ObjectInScene *objectPtr, OutputImageType* image, OutputLabelMapType *labelMap) {
	if (!m_imageAllocated) AllocateOutputImage();
	if (!objectPtr) return false;

	//compute the bounding box of interest and the corresponding image grid
	vnl_vector<double> tmpBBox, roiBBox;
	const vnl_vector<double>& sceneBBox = m_scene->GetSceneBoundingBox();
	if (!IntersectionBoundingBoxes( sceneBBox, objectPtr->obj->GetPhysicalBoundingBox(), &tmpBBox )) return false;
	roiBBox = GetSensedBoundingBox(tmpBBox);
	//
	image->Initialize();labelMap->Initialize();
	const OutputImageType::SpacingType &outputSpacing = m_outputSpacing;
	OutputImageType::PointType  outputOrigin;
	OutputImageType::RegionType outputRegion;
	ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing<2>(roiBBox, outputSpacing.GetVnlVector(), &outputOrigin, &outputRegion);
	const OutputImageType::SizeType &outputSize = outputRegion.GetSize();
	labelMap->ClearLabels();
	labelMap->SetSpacing( outputSpacing ); image->SetSpacing( outputSpacing );
	labelMap->SetOrigin(  outputOrigin  ); image->SetOrigin(  outputOrigin  );
	labelMap->SetRegions( outputRegion  ); image->SetRegions( outputRegion  );
	image->Allocate();

	OutputImageType::PixelType *imageBuffer = image->GetBufferPointer();

	//Get the object label map from the scene
	InputLabelObjectType *inputObject = m_scene->GetObjectLabelObject(objectPtr->id);
	InputLabelObjectType::IndexType index3D; InputLabelObjectType::LengthType length3D;
	InputLabelMapType::PointType point3D;

	//offset related to the fact that only a small part of the sensor image grid is manipulated here
	OutputImageType::OffsetType offset;
	offset[0] = round((outputOrigin[0] - m_outputOrigin[0])/m_outputSpacing[0]);
	offset[1] = round((outputOrigin[1] - m_outputOrigin[1])/m_outputSpacing[1]);

	long add; int maxz;

	//Create an empty label object for the output.
	OutputLabelObjectType::Pointer outputLabelObject = OutputLabelObjectType::New();
	OutputImageType::IndexType index2D;
	int *zbuff; double zspacing, dist, zorigin;
	//Convert it to a pseudo-zbuffer
	switch(m_direction) {
		case SAGITTAL: //SAGITTAL -> (y,z) plane	
			zspacing = m_scene->GetSceneSpacing()[0]; zorigin = sceneBBox(0);
			//image->FillBuffer(0);	
			for (add=0 ; add<outputRegion.GetSize(0)*outputRegion.GetSize(1) ; add++) imageBuffer[add] = m_backgroundValue;
			for (unsigned j=0 ; j<inputObject->GetNumberOfLines() ; j++) {
				index3D = inputObject->GetLine(j).GetIndex();
				//sagittal => 3D line in the x direction => the index is enough to proceed. just compute the offset...
				//z value: index3D[0] ; [x,y] coordinates on the sensed image are given by index3D[1] and index3D[2]
				index2D[0] = index3D[1] - offset[0];
				index2D[1] = index3D[2] - offset[1];
				add = index2D[1]*outputRegion.GetSize(0)+index2D[0];
				dist = index3D[0]*zspacing;//dist = zorigin + index3D[0]*zspacing - sceneBBox(0);
				//the distance between the sensor and the object is directly related to the index
				imageBuffer[add] = m_function->Evaluate( dist );
				//image->SetPixel(index2D, m_function->Evaluate( dist ));
				outputLabelObject->AddIndex(index2D);
			}
			break;
		case CORONAL: //CORONAL -> (x,z) plane
			zspacing = m_scene->GetSceneSpacing()[1]; zorigin = sceneBBox(2);
			//initialize a zbuffer to the maximum distance
			maxz = std::numeric_limits<int>::max();
			zbuff = (int *)malloc(outputRegion.GetNumberOfPixels()*sizeof(int));
			for (unsigned j=0 ; j<outputRegion.GetNumberOfPixels() ; j++) zbuff[j] = maxz;
			//browse the lines of the object and update the zbuffer accordingly.
			for (unsigned j=0 ; j<inputObject->GetNumberOfLines() ; j++) {
				index3D = inputObject->GetLine(j).GetIndex();
				index2D[0] = index3D[0] - offset[0]; index2D[1] = index3D[2] - offset[1];
				add = index2D[0] + index2D[1]*outputSize[0]; //address in the local buffer.
				for (unsigned k=0 ; k<inputObject->GetLine(j).GetLength() ; k++) {
					zbuff[add] = std::min<int>(zbuff[add], index3D[1]);
					add++;
				}
			}
			//now, browse the zbuffer and fill the output images.
			for (index2D[1]=0 ; index2D[1]<outputRegion.GetSize(1) ; index2D[1]++) {
				add = index2D[1]*outputRegion.GetSize(0);
				for (index2D[0]=0 ; index2D[0]<outputRegion.GetSize(0) ; index2D[0]++) {
					if (zbuff[add]<maxz) {
						dist = zbuff[add]*zspacing;//dist = std::max<double>(0, zorigin+zbuff[add]*zspacing - sceneBBox(2));
						imageBuffer[add] = m_function->Evaluate( dist );
						outputLabelObject->AddIndex(index2D);
					}
					else { imageBuffer[add] = m_backgroundValue; } 
					add++;
				}
			}
			free(zbuff);
			break;
		case HORIZONTAL: //HORIZONTAL -> (x,y)
			zspacing = m_scene->GetSceneSpacing()[2]; zorigin = sceneBBox(4);
			//initialize a zbuffer to the maximum distance
			maxz = std::numeric_limits<int>::max();
			zbuff = (int *)malloc(outputRegion.GetNumberOfPixels()*sizeof(int));
			for (unsigned j=0 ; j<outputRegion.GetNumberOfPixels() ; j++) zbuff[j] = maxz;
			//browse the lines of the object and update the zbuffer accordingly.
			for (unsigned j=0 ; j<inputObject->GetNumberOfLines() ; j++) {
				index3D = inputObject->GetLine(j).GetIndex();
				index2D[0] = index3D[0] - offset[0]; index2D[1] = index3D[1] - offset[1];
				add = index2D[0] + index2D[1]*outputSize[0]; //address in the local buffer.
				for (unsigned k=0 ; k<inputObject->GetLine(j).GetLength() ; k++) {
					zbuff[add] = std::min<int>(zbuff[add], index3D[2]);
					add++;
				}
			}
			//now, browse the zbuffer and fill the output images.
			for (index2D[1]=0 ; index2D[1]<outputRegion.GetSize(1) ; index2D[1]++) {
				add = index2D[1]*outputRegion.GetSize(0);
				for (index2D[0]=0 ; index2D[0]<outputRegion.GetSize(0) ; index2D[0]++) {
					if (zbuff[add]<maxz) {
						dist = zbuff[add]*zspacing;//dist = std::max<double>(0, zorigin+zbuff[add]*zspacing - sceneBBox(4));
						imageBuffer[add] = m_function->Evaluate( dist );
						outputLabelObject->AddIndex(index2D);
					}
					else { imageBuffer[add] = m_backgroundValue; } 
					add++;
				}
			}
			free(zbuff);
			break;
	}
	outputLabelObject->Optimize(); //this may actually not be necessary
	outputLabelObject->SetLabel(objectPtr->id);
	labelMap->AddLabelObject(outputLabelObject);

	return true;
}


/** Same as before, but only compute the output image, not the label map. 
* is it faster?
* if necessary, I could avoid looping over the whole image again for the CORONAL & HORIZONTAL VIEWS, and directly compute the value...
*/
template<class TScene, class TOutputImage>
bool 
SimplePixelBasedSEMSensor<TScene, TOutputImage>::GetOffContextObjectImage(ObjectInScene *objectPtr, OutputImageType* image) {
	if (!m_imageAllocated) AllocateOutputImage();
	if (!objectPtr) return false;

	//compute the bounding box of interest and the corresponding image grid
	vnl_vector<double> tmpBBox, roiBBox;
	const vnl_vector<double>& sceneBBox = m_scene->GetSceneBoundingBox();
	if (!IntersectionBoundingBoxes( sceneBBox, objectPtr->obj->GetPhysicalBoundingBox(), &tmpBBox )) return false;
	roiBBox = GetSensedBoundingBox(tmpBBox);
	//
	const OutputImageType::SpacingType& outputSpacing = m_outputSpacing;
	OutputImageType::PointType  outputOrigin;
	OutputImageType::RegionType outputRegion;
	OutputImageType::SizeType   outputSize;
	ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing<2>(roiBBox, outputSpacing.GetVnlVector(), &outputOrigin, &outputRegion);
	outputSize = outputRegion.GetSize();
	image->SetSpacing( outputSpacing );
	image->SetOrigin(  outputOrigin  );
	image->SetRegions( outputRegion  );
	image->Allocate();

	OutputImageType::PixelType *imageBuffer = image->GetBufferPointer();

	//Get the object label map from the scene
	InputLabelObjectType *inputObject = m_scene->GetObjectLabelObject(objectPtr->id);
	InputLabelObjectType::IndexType index3D; InputLabelObjectType::LengthType length3D;
	InputLabelMapType::PointType point3D;

	//offset related to the fact that only a small part of the sensor image grid is manipulated here
	OutputImageType::OffsetType offset;
	offset[0] = round((outputOrigin[0] - m_outputOrigin[0])/m_outputSpacing[0]);
	offset[1] = round((outputOrigin[1] - m_outputOrigin[1])/m_outputSpacing[1]);

	long add;
	int maxz;

	//Create an empty label object for the output.
	OutputLabelObjectType::Pointer outputLabelObject = OutputLabelObjectType::New();
	OutputImageType::IndexType index2D;
	int *zbuff; double zspacing, dist, zorigin;
	//Convert it to a pseudo-zbuffer
	switch(m_direction) {
			case SAGITTAL: //SAGITTAL -> (y,z) plane	
				zspacing = m_scene->GetSceneSpacing()[0]; zorigin = sceneBBox(0);
				image->FillBuffer(0);
				for (unsigned j=0 ; j<inputObject->GetNumberOfLines() ; j++) {
					index3D = inputObject->GetLine(j).GetIndex();
					//sagittal => 3D line in the x direction => the index is enough to proceed. just compute the offset...
					//z value: index3D[0] ; [x,y] coordinates on the sensed image are given by index3D[1] and index3D[2]
					index2D[0] = index3D[1] - offset[0];
					index2D[1] = index3D[2] - offset[1];
					dist = index3D[0]*zspacing;//dist = zorigin + index3D[0]*zspacing - sceneBBox(0);
					//the distance between the sensor and the object is directly related to the index
					image->SetPixel(index2D, m_function->Evaluate( dist ));
				}
				break;
			case CORONAL: //CORONAL -> (x,z) plane
				zspacing = m_scene->GetSceneSpacing()[1]; zorigin = sceneBBox(2);
				//initialize a zbuffer to the maximum distance
				maxz = std::numeric_limits<int>::max();
				zbuff = (int *)malloc(outputRegion.GetNumberOfPixels()*sizeof(int));
				for (unsigned j=0 ; j<outputRegion.GetNumberOfPixels() ; j++) zbuff[j] = maxz;
				//browse the lines of the object and update the zbuffer accordingly.
				for (unsigned j=0 ; j<inputObject->GetNumberOfLines() ; j++) {
					index3D = inputObject->GetLine(j).GetIndex();
					index2D[0] = index3D[0] - offset[0]; index2D[1] = index3D[2] - offset[1];
					add = index2D[0] + index2D[1]*outputSize[0]; //address in the local buffer.
					for (unsigned k=0 ; k<inputObject->GetLine(j).GetLength() ; k++) {
						zbuff[add] = std::min<int>(zbuff[add], index3D[1]);
						add++;
					}
				}
				//now, browse the zbuffer and fill the output images.
				for (index2D[1]=0 ; index2D[1]<outputRegion.GetSize(1) ; index2D[1]++) {
					add = index2D[1]*outputRegion.GetSize(0);
					for (index2D[0]=0 ; index2D[0]<outputRegion.GetSize(0) ; index2D[0]++) {
						if (zbuff[add]<maxz) {
							dist = zbuff[add]*zspacing;//dist = std::max<double>(0, zorigin+zbuff[add]*zspacing - sceneBBox(2));
							imageBuffer[add] = m_function->Evaluate( dist );
						}
						else { imageBuffer[add] = m_backgroundValue; } 
						add++;
					}
				}
				free(zbuff);
				break;
			case HORIZONTAL: //HORIZONTAL -> (x,y)
				zspacing = m_scene->GetSceneSpacing()[2]; zorigin = sceneBBox(4);
				//initialize a zbuffer to the maximum distance
				maxz = std::numeric_limits<int>::max();
				zbuff = (int *)malloc(outputRegion.GetNumberOfPixels()*sizeof(int));
				for (unsigned j=0 ; j<outputRegion.GetNumberOfPixels() ; j++) zbuff[j] = maxz;
				//browse the lines of the object and update the zbuffer accordingly.
				for (unsigned j=0 ; j<inputObject->GetNumberOfLines() ; j++) {
					index3D = inputObject->GetLine(j).GetIndex();
					index2D[0] = index3D[0] - offset[0]; index2D[1] = index3D[1] - offset[1];
					add = index2D[0] + index2D[1]*outputSize[0]; //address in the local buffer.
					for (unsigned k=0 ; k<inputObject->GetLine(j).GetLength() ; k++) {
						zbuff[add] = std::min<int>(zbuff[add], index3D[2]);
						add++;
					}
				}
				//now, browse the zbuffer and fill the output images.
				for (index2D[1]=0 ; index2D[1]<outputRegion.GetSize(1) ; index2D[1]++) {
					add = index2D[1]*outputRegion.GetSize(0);
					for (index2D[0]=0 ; index2D[0]<outputRegion.GetSize(0) ; index2D[0]++) {
						if (zbuff[add]<maxz) {
							dist = zbuff[add]*zspacing;//dist = std::max<double>(0, zorigin+zbuff[add]*zspacing - sceneBBox(4));
							imageBuffer[add] = m_function->Evaluate( dist );
						}
						else { imageBuffer[add] = m_backgroundValue; } 
						add++;
					}
				}
				free(zbuff);
				break;
	}

	return true;
}




/** forces the output image to have the same resolution, dimension, etc... as the scene */
template<class TScene, class TOutputImage>
void
SimplePixelBasedSEMSensor<TScene, TOutputImage>::AllocateOutputImage() {	
	if (!m_scene) throw DeformableModelException("SimplePixelBasedSEMSensor::AllocateOutputImage the scene must be set first!");

	//Get the useful characteristics from the scene
	const InputLabelImageType::SpacingType& inputSpacing= m_scene->GetSceneSpacing();
	const InputLabelImageType::PointType&   inputOrigin = m_scene->GetSceneOrigin();
	const InputLabelImageType::RegionType&  sceneRegion = m_scene->GetSceneImageRegion();
	const InputLabelImageType::SizeType&    sceneSize	= sceneRegion.GetSize();

	//
	m_outputSpacing[0]= inputSpacing[m_dirAxis0]; m_outputSpacing[1]= inputSpacing[m_dirAxis1];
	m_outputOrigin[0] = inputOrigin[m_dirAxis0];  m_outputOrigin[1] = inputOrigin[m_dirAxis1];
	m_outputSize[0]   = sceneSize[m_dirAxis0];    m_outputSize[1]  = sceneSize[m_dirAxis1];    m_outputRegion.SetSize(m_outputSize);
	OutputImageType::IndexType outputStart;       outputStart[0] = 0; outputStart[1] = 0;      m_outputRegion.SetIndex(outputStart);

	//
	m_outputImage->SetSpacing( m_outputSpacing );
	m_outputImage->SetOrigin( m_outputOrigin );
	m_outputImage->SetRegions( m_outputRegion );
	try { m_outputImage->Allocate(); }
	catch( itk::ExceptionObject &e) { std::cerr << "Exception raised allocating the sensor image: \n"<< e << std::endl; }
	m_outputImage->FillBuffer(0);

	//
	m_outputLabelMap->SetSpacing( m_outputSpacing );
	m_outputLabelMap->SetOrigin( m_outputOrigin );
	m_outputLabelMap->SetRegions( m_outputRegion );
	m_outputLabelMap->ClearLabels();

	//
	m_outputLabelImage->SetSpacing( m_outputSpacing );
	m_outputLabelImage->SetOrigin( m_outputOrigin );
	m_outputLabelImage->SetRegions( m_outputRegion );
	try { m_outputLabelImage->Allocate(); }
	catch( itk::ExceptionObject &e) { std::cerr << "Exception raised allocating the sensor image: \n"<< e << std::endl; }
	m_outputLabelImage->FillBuffer(0);

	m_imageAllocated = true;
}


/** Update data in the requested region (CLEAN: use itk -> RequestedRegion instead?) */
template<class TScene, class TOutputImage>
void
SimplePixelBasedSEMSensor<TScene, TOutputImage>::Update_Data(typename InputLabelImageType::RegionType inputRegion) {

	//update the labelImage from the scene
	InputLabelImageType *inputLabelImage = m_scene->GetSceneAsLabelImage();
	InputLabelImageType::PixelType *inputLabelBuffer = inputLabelImage->GetBufferPointer();

	//get pointers to the buffers of the output 
	OutputLabelImageType::PixelType *outputLabelBuffer = m_outputLabelImage->GetBufferPointer();
	OutputImageType::PixelType *outputBuffer = m_outputImage->GetBufferPointer();

	//so... browse inputLabelImage (3D) starting on the face near the sensor.
	//then follow the normal direction until an object is encountered
	// -> fill the m_outputLabelImage and the m_outputImage accordingly
	//finally, convert the m_outputLabelImage to a m_outputLabelMap.
	const InputLabelImageType::RegionType& sceneRegion = m_scene->GetSceneImageRegion();
	const InputLabelImageType::SizeType&   sceneSize   = sceneRegion.GetSize();

	InputLabelImageType::IndexType index3D;
	OutputImageType::IndexType index2D;
	long add2D, add3D;
	LabelType label;
	bool found;
	unsigned int increment3D;
	switch(m_direction) {//how to increment the address when browsing the 3D scene, in the direction normal to the sensor plane
		case SAGITTAL:	 increment3D = 1; break; // normal is x
		case CORONAL:	 increment3D = sceneSize[0]; break; //CORONAL -> (x,z) plane, normal is y
		case HORIZONTAL: increment3D = sceneSize[0]*sceneSize[1]; break; //HORIZONTAL -> (x,y), normal is z
		default: throw DeformableModelException("SimplePixelBasedSEMSensor: direction parameter must be 0, 1 or 2 "); //should never happen
	}

	//Browse the region that needs to be updated on the sensor image
	for (index2D[1]=inputRegion.GetIndex(m_dirAxis1) ; index2D[1]<inputRegion.GetIndex(m_dirAxis1)+inputRegion.GetSize(m_dirAxis1) ; index2D[1]++) {			
		index3D[m_dirAxis1] = index2D[1];
		for (index2D[0]=inputRegion.GetIndex(m_dirAxis0), add2D = index2D[1]*m_outputSize[0]+index2D[0] ;
			index2D[0]<inputRegion.GetIndex(m_dirAxis0)+inputRegion.GetSize(m_dirAxis0) ; 
			index2D[0]++, add2D++)
		{
			index3D[m_dirAxis0] = index2D[0];
			//first: check if there is something in front of this region => if the sensor intensity is higher...
			//this could be optimized using a zbuffer (or perhaps exploiting the fact that intensity is strictly decreasing with distance)
			found = false;				
			for ( index3D[m_dirAxis2] = 0, add3D = index3D[0] + index3D[1]*sceneSize[0] + index3D[2]*sceneSize[0]*sceneSize[1];
				index3D[m_dirAxis2]<inputRegion.GetIndex(m_dirAxis2) ;
				index3D[m_dirAxis2]++, add3D+=increment3D )
			{
				if (inputLabelBuffer[add3D]>0) { 
					if (outputLabelBuffer[add2D]!=inputLabelBuffer[add3D]) {
						std::cout<<"--updated the sensor image based on a pixel in front of the region to update, from a different object than expected... should never happen... old id = "<<outputLabelBuffer[add2D]<<", new one: "<<inputLabelBuffer[add3D]<<", if the new id is higher than the old, this is most certainly a bug, check comments in the code"<<std::endl;
						//actually, there is one reason this may happen: when multiple objects overlap on this pixel
						//if the object with highest id is modified and e.g. moves back, the other may take its place
						//and cause this previously unexpected behavior
						outputLabelBuffer[add2D]=inputLabelBuffer[add3D];
						outputBuffer[add2D] = m_function->Evaluate( index3D[m_dirAxis2]*m_outputSpacing[m_dirAxis2] );
					}
					found = true;
					break; 
				}
			}
			if (found) continue;
			//now, traverse the 3D image in a direction normal to the sensor plane, but only if there is nothing in front...
			//found = false;
			for ( index3D[m_dirAxis2] = inputRegion.GetIndex(m_dirAxis2), add3D = index3D[0] + index3D[1]*sceneSize[0] + index3D[2]*sceneSize[0]*sceneSize[1];
				index3D[m_dirAxis2]<inputRegion.GetIndex(m_dirAxis2)+inputRegion.GetSize(m_dirAxis2) ;
				index3D[m_dirAxis2]++, add3D+=increment3D )
			{
				label = inputLabelBuffer[add3D];
				if (label>0) {
					outputLabelBuffer[add2D] = label;
					outputBuffer[add2D] = m_function->Evaluate( index3D[m_dirAxis2]*m_outputSpacing[m_dirAxis2] );
					found = true;
					break;
				}					
			}
			if (found) continue;
			//finish browsing the cube, updating things depending on the background (if an object was removed...)
			//found = false;
			for ( index3D[m_dirAxis2] = inputRegion.GetIndex(m_dirAxis2)+inputRegion.GetSize(m_dirAxis2), add3D = index3D[0] + index3D[1]*sceneSize[0] + index3D[2]*sceneSize[0]*sceneSize[1];
				index3D[m_dirAxis2] < sceneSize[m_dirAxis2] ;
				index3D[m_dirAxis2]++, add3D+=increment3D )
			{
				label = inputLabelBuffer[add3D];
				if (label>0) {
					outputLabelBuffer[add2D] = label;
					outputBuffer[add2D] = m_function->Evaluate( index3D[m_dirAxis2]*m_outputSpacing[m_dirAxis2] );
					found = true;
					break;
				}					
			}
			if (!found) { //nothing here. reset the sensor pixel to empty state
				outputLabelBuffer[add2D] = 0;
				outputBuffer[add2D] = 0;
			}

		}
	}

	//ok, now convert the output label image to a label map...
	//OPTIMIZATION: update only the objects that need to be updated?
	LabelImageToLabelMapFilterType::Pointer labelImageToLabelMapFilter = LabelImageToLabelMapFilterType::New();
	labelImageToLabelMapFilter->SetInput( m_outputLabelImage );
	labelImageToLabelMapFilter->Update();
	m_outputLabelMap = labelImageToLabelMapFilter->GetOutput();

}


} // namespace psciob

#endif /* SIMPLEPIXELBASEDSEMSENSOR_TXX_ */
