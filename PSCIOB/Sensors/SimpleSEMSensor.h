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
* \file SimpleSEMSensor.h
* \author Rémi Blanc 
* \date 28. September 2011
*/

#ifndef IMAGE3D2DSENSOR_H_
#define IMAGE3D2DSENSOR_H_


#include "ImageSensor_Base.h"
#include "vtkCamera.h"
#include "vtkLight.h"

//IDEA: should this class maintain a full labelImage at all times?
//it would make a slight overhead, recomputing the LabelMap out of it
//but allow to update only small regions of the sensor instead of the full image all the time...
//elements for this second option are written & commented after the Update_data method

namespace psciob {

/**
* \class SimpleSEMSensor
* \brief SimpleSEMSensor from a 3D scene, generates a 2D image => parallel / projective view, etc...
* uses a VTK camera
*	
* the direction is either HORIZONTAL (default), SAGITTAL or CORONAL
* the default appearance function is decreasing exponentially (ExpDecrFunction) with initial parameter value 250
*
* TODO: this seem to be broken... check the whole class
*
*/


//CONCRETE CLASS
template<class TScene, class TOutputImage>
class SimpleSEMSensor : public ImageSensor_Base<TScene, TOutputImage> { 
protected:
	static const unsigned int m_nbOrientationParams = 1;	//code for the direction (horizontal/sagittal/coronal)
	static const unsigned int m_nbDistortionParams  = 0;	//no distortion
	static const unsigned int m_nbNoiseParams		= 0;	//no noise
	static const unsigned int m_nbResolutionParams  = 0;	//no resolution params ~> best would be to set the requested resolution, and an interpolator rather than a set of parameters...
public:
	/** Standard class typedefs. */
	typedef SimpleSEMSensor								Self;
	typedef ImageSensor_Base							Superclass;
	typedef itk::SmartPointer<Self>						Pointer;
	typedef itk::SmartPointer<const Self>				ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SimpleSEMSensor, ImageSensor_Base);
	itkNewMacro(Self);


	/** One orientation parameters */
	unsigned int GetNumberOfOrientationParameters() { return m_nbOrientationParams; } 
	/** No distortion parameters */
	unsigned int GetNumberOfDistortionParameters()	{ return m_nbDistortionParams; }
	/** No noise parameters */
	unsigned int GetNumberOfNoiseParameters()		{ return m_nbNoiseParams; }
	/** No resolution parameters */
	unsigned int GetNumberOfResolutionParameters()	{ return m_nbResolutionParams; }

	/** Set the direction: Generic interface */ 
	bool SetOrientationParameters(vnl_vector<double> p) {
		if (p.size()!=GetNumberOfOrientationParameters()) return false;
		if ( fabs( p(0) ) <TINY) { SetObservationDirectionType(HORIZONTAL);return true;}
		if ( fabs(p(0)-1) <TINY) { SetObservationDirectionType(SAGITTAL);  return true;}
		if ( fabs(p(0)-2) <TINY) { SetObservationDirectionType(CORONAL);   return true;}
		return false; //
	}

	/** Set the direction, specific to this class : HORIZONTAL / SAGITTAL / CORONAL */ 
	void SetObservationDirectionType(ObservationDirectionType dir)	{
		m_orientationParams(0) = dir ; m_orientationFlag=true;
		if (m_direction!=dir) { m_direction = dir; Modified(); } //if nothing really changed, don't set raise the ->Modified() flag
		switch(m_direction) { // 0 = first axis in 3D = X ; 1: second axis => Y ; Z = 3rd axis => Z
		case SAGITTAL:	 m_dirAxis0=1;m_dirAxis1=2;m_dirAxis2=0; break; //if the observation direction is SAGITTAL, then the axes of the output image correspond to the (y,z) plane in 3D
		case CORONAL:	 m_dirAxis0=0;m_dirAxis1=2;m_dirAxis2=1; break; //CORONAL -> (x,z) plane
		case HORIZONTAL: m_dirAxis0=0;m_dirAxis1=1;m_dirAxis2=2; break; //HORIZONTAL -> (x,y)
		default: throw DeformableModelException("SimpleSEMSensor: direction parameter must be 0, 1 or 2 "); //should never happen
		}
	}
	ObservationDirectionType GetObservationDirectionType()			{return m_direction;}	


	/** bounding box of the sensed scene */
	vnl_vector<double> GetSensedBoundingBox(const vnl_vector<double> &bbox) {
		vnl_vector<double> sensedBBox(2*TOutputImage::ImageDimension);
		sensedBBox(0) = bbox(2*m_dirAxis0);	sensedBBox(1) = bbox(2*m_dirAxis0+1);
		sensedBBox(2) = bbox(2*m_dirAxis1);	sensedBBox(3) = bbox(2*m_dirAxis1+1);
		return sensedBBox;
	}


	/** Off Context image of an object: as if it was alone in the scene 
	* this labelMap is expressed with respect to the full sensor grid
	* returns false if the object is not in the scene
	*/
	bool GetOffContextObjectLabelMap(ObjectInScene *objectPtr, OutputLabelMapType *offContextObjectLabelMap) {
		//if (objectPtr->id == 0) return false;
		if (!m_imageAllocated) AllocateOutputImage();

		vnl_vector<double> sceneBBox = m_scene->GetSceneBoundingBox();

		offContextObjectLabelMap->ClearLabels();
		offContextObjectLabelMap->SetSpacing( m_outputSpacing );
		offContextObjectLabelMap->SetOrigin(  m_outputOrigin  );
		offContextObjectLabelMap->SetRegions( m_outputRegion  );

		//Setup the view, camera, light, ...
		vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();  renderer->SetBackground(0,0,0);
		vtkSmartPointer<vtkPolyDataMapper> map = vtkSmartPointer<vtkPolyDataMapper>::New(); map->SetInput( objectPtr->obj->GetObjectAsVTKPolyData() );
		vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New(); actor->SetMapper(map); 
		LabelType tmpid = objectPtr->id%(255*255); unsigned char R = objectPtr->id/(255*255); unsigned char G = tmpid/255; unsigned char B = tmpid%255;
		actor->GetProperty( )->SetColor(R/255.0, G/255.0, B/255.0); 
		actor->GetProperty( )->SetAmbient(1); actor->GetProperty( )->SetDiffuse(0); actor->GetProperty( )->SetSpecular(0);
		actor->SetMapper(map); 

		renderer->AddActor(actor);

		vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New(); 
		renderWindow->SetOffScreenRendering(1);
		renderWindow->AddRenderer(renderer); 
		renderWindow->SetSize(m_outputSize[0], m_outputSize[1]);
		renderer->SetActiveCamera(m_camera);
		renderer->RemoveAllLights(); renderer->AddLight(m_light);

		//Generate the image
		renderWindow->Render();

		//Extract the corresponding LabelMap
		unsigned char *pixbuff = renderWindow->GetPixelData(0, 0, m_outputSize[0]-1, m_outputSize[1]-1, 0);

		double dist;

		OutputLabelMapType::IndexType index;
		OutputLabelMapType::LabelType label, currentLabel;
		OutputLabelMapType::LengthType length;
		offContextObjectLabelMap->ClearLabels();

		currentLabel = 0;
		switch(m_direction) {
			case SAGITTAL:
				for (unsigned j=0 ; j<m_outputSize[1] ; j++) {
					if (currentLabel!=0) { //finalize a run
						index[0] = index[0]-length+1; 
						if ( !offContextObjectLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
							OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
							labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); offContextObjectLabelMap->AddLabelObject(labelObject);
						}
						else { offContextObjectLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
					}
					currentLabel = 0;
					for (unsigned i=0 ; i<m_outputSize[0] ; i++) {
						//not sure why this /2 is necessary, but it does in the ToyApplication to recover the correct label / id
						label = (255*255*(unsigned short)(pixbuff[3*(i+j*m_outputSize[0])])+255*(unsigned short)(pixbuff[3*(i+j*m_outputSize[0])+1])+(unsigned short)(pixbuff[3*(i+j*m_outputSize[0])+2]))/2;
						if (label!=currentLabel) { //start a new run
							if (currentLabel!=0) { //need to finalize the current run if any
								index[0] = index[0]-length+1; //flip the indices							
								if ( !offContextObjectLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
									OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
									labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); offContextObjectLabelMap->AddLabelObject(labelObject);
								}
								else { offContextObjectLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
							}
							//we start a new run
							currentLabel = label; index[0] = (m_outputSize[0]-i-1); index[1] = j; length = 1;
						}
						else { length++; } //continuing the current run
					}
				}
				if (currentLabel!=0) { //finalize the current run
					index[0] = index[0]-length+1; 
					if ( !offContextObjectLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
						OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
						labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length);	offContextObjectLabelMap->AddLabelObject(labelObject);
					}
					else { offContextObjectLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
				}
				break;
			case CORONAL:
				for (unsigned j=0 ; j<m_outputSize[1] ; j++) {
					if (currentLabel!=0) { //finalize a run
						if ( !offContextObjectLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
							OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
							labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); offContextObjectLabelMap->AddLabelObject(labelObject);
						}
						else { offContextObjectLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
					}
					currentLabel = 0;
					for (unsigned i=0 ; i<m_outputSize[0] ; i++) {
						//not sure why this /2 is necessary, but it does in the ToyApplication to recover the correct label / id
						label = (255*255*(unsigned short)(pixbuff[3*(i+j*m_outputSize[0])])+255*(unsigned short)(pixbuff[3*(i+j*m_outputSize[0])+1])+(unsigned short)(pixbuff[3*(i+j*m_outputSize[0])+2]))/2;
						if (label!=currentLabel) { //starting a new run							
							if (currentLabel!=0) { //need to finalize the current run if any
								if ( !offContextObjectLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
									OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
									labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); offContextObjectLabelMap->AddLabelObject(labelObject);
								}
								else { offContextObjectLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
							}
							//we start a new run
							currentLabel = label; index[0] = i; index[1] = j; length = 1;
						}
						else { length++; } //continuing the current run
					}
				}
				if (currentLabel!=0) { //finalize a run
					if ( !offContextObjectLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
						OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
						labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); offContextObjectLabelMap->AddLabelObject(labelObject);
					}
					else { offContextObjectLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
				}
				break;
			case HORIZONTAL:
				for (unsigned j=0 ; j<m_outputSize[1] ; j++) {
					if (currentLabel!=0) { //finalize a run
						index[0] = index[0]-length+1; 
						if ( !offContextObjectLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
							OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
							labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); offContextObjectLabelMap->AddLabelObject(labelObject);
						}
						else { offContextObjectLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
					}
					currentLabel = 0;
					for (unsigned i=0 ; i<m_outputSize[0] ; i++) {
						//not sure why this /2 is necessary, but it does in the ToyApplication to recover the correct label / id
						label = (255*255*(unsigned short)(pixbuff[3*(i+j*m_outputSize[0])])+255*(unsigned short)(pixbuff[3*(i+j*m_outputSize[0])+1])+(unsigned short)(pixbuff[3*(i+j*m_outputSize[0])+2]))/2;
						if (label!=currentLabel) { //start a new run
							if (currentLabel!=0) { //need to finalize the current run if any 
								index[0] = index[0]-length+1; 
								if ( !offContextObjectLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
									OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
									labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); offContextObjectLabelMap->AddLabelObject(labelObject);
								}
								else { offContextObjectLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
							}
							//we start a new run
							currentLabel = label; index[0] = (m_outputSize[0]-i-1); index[1] = j; length = 1;
						}
						else { length++; }//continuing the current run
					}
				}
				if (currentLabel!=0) { //finalize a run
					index[0] = index[0]-length+1; 
					if ( !offContextObjectLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
						OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
						labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); offContextObjectLabelMap->AddLabelObject(labelObject);
					}
					else { offContextObjectLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
				}
				break;
		}

		delete [] pixbuff;
		return true;
	}


	/** Off Context image of an object: as if it was alone in the scene 
	* The image and label maps are expressed relative to the smallest possible region of the sensor frame (to same memory and computation time...)
	* 	OPTIMIZATION: different possibilities for optimization
	*  - caching the images / labelMap for each object, exploiting particular DataContainer for the scene?
	* returns false if the object is not in the scene
	*/
	bool GetOffContextObjectImage(ObjectInScene *objectPtr, OutputImageType* image, OutputLabelMapType *labelMap) {
		if (!m_imageAllocated) AllocateOutputImage();
		if (!objectPtr) return false;
		//compute the bounding box of interest and the corresponding image grid
		vnl_vector<double> tmpBBox, roiBBox, sceneBBox = m_scene->GetSceneBoundingBox();
		if (!IntersectionBoundingBoxes( sceneBBox, objectPtr->obj->GetPhysicalBoundingBox(), &tmpBBox )) return false;
		roiBBox = GetSensedBoundingBox(tmpBBox);
		//
		OutputImageType::SpacingType outputSpacing = m_outputSpacing;
		OutputImageType::PointType  outputOrigin;
		OutputImageType::RegionType outputRegion;
		OutputImageType::SizeType   outputSize;
		ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing<2>(roiBBox, outputSpacing.GetVnlVector(), &outputOrigin, &outputRegion);
		outputSize = outputRegion.GetSize();
		labelMap->ClearLabels();
		labelMap->SetSpacing( outputSpacing ); image->SetSpacing( outputSpacing );
		labelMap->SetOrigin(  outputOrigin  ); image->SetOrigin(  outputOrigin  );
		labelMap->SetRegions( outputRegion  ); image->SetRegions( outputRegion  );
		image->Allocate();

		//Setup the view, camera, light, ...
		vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();  renderer->SetBackground(0,0,0);
		vtkSmartPointer<vtkPolyDataMapper> map = vtkSmartPointer<vtkPolyDataMapper>::New(); map->SetInput( objectPtr->obj->GetObjectAsVTKPolyData() );
		vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New(); actor->SetMapper(map); 
		LabelType tmpid = objectPtr->id%(255*255); unsigned char R = objectPtr->id/(255*255); unsigned char G = tmpid/255; unsigned char B = tmpid%255;
		actor->GetProperty( )->SetColor(R/255.0, G/255.0, B/255.0); 
		actor->GetProperty( )->SetAmbient(1); actor->GetProperty( )->SetDiffuse(0); actor->GetProperty( )->SetSpecular(0);
		
		renderer->AddActor(actor);

		vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New(); 
		renderWindow->SetOffScreenRendering(1);

		vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
		camera->ParallelProjectionOn();

		renderWindow->SetSize(outputRegion.GetSize(0), outputRegion.GetSize(1));
		renderWindow->AddRenderer( renderer ); 

		//always look toward the center of the region of interest
		//keep the distance between the camera and scene fixed, only translate the view box
		camera->SetFocalPoint( (tmpBBox(0)+tmpBBox(1))/2.0, (tmpBBox(2)+tmpBBox(3))/2.0, (tmpBBox(4)+tmpBBox(5))/2.0 );
		switch (m_direction) {
		case SAGITTAL: 
			camera->SetPosition( sceneBBox(0)-m_cameraOffset, (tmpBBox(2)+tmpBBox(3))/2.0, (tmpBBox(4)+tmpBBox(5))/2.0 ); 
			camera->SetViewUp(0,0,1);
			camera->SetParallelScale( (tmpBBox(5)-tmpBBox(4)) / 2.0 );
			camera->SetClippingRange( 0, m_cameraOffset + sceneBBox(1)-sceneBBox(0) );
			break; //if the observation direction is SAGITTAL, then the axes of the output image correspond to the (y,z) plane in 3D
		case CORONAL:
			camera->SetPosition( (tmpBBox(0)+tmpBBox(1))/2.0, sceneBBox(2)-m_cameraOffset, (tmpBBox(4)+tmpBBox(5))/2.0 );
			camera->SetViewUp(0,0,1);
			camera->SetParallelScale( (tmpBBox(5)-tmpBBox(4)) / 2.0 );
			camera->SetClippingRange( 0, m_cameraOffset + sceneBBox(3)-sceneBBox(2) );
			break; //CORONAL -> (x,z) plane
		case HORIZONTAL:
			camera->SetPosition( (tmpBBox(0)+tmpBBox(1))/2.0, (tmpBBox(2)+tmpBBox(3))/2.0, sceneBBox(4)-m_cameraOffset ); 
			camera->SetViewUp(0,1,0);
			camera->SetParallelScale( (tmpBBox(3)-tmpBBox(2)) / 2.0 );
			camera->SetClippingRange( 0, m_cameraOffset + sceneBBox(5)-sceneBBox(4) ); //doing so, the zubbfer
			break; //HORIZONTAL -> (x,y)
		}

		renderer->SetActiveCamera(camera);
		renderer->RemoveAllLights(); renderer->AddLight(m_light);

		vtkSmartPointer<vtkMatrix4x4> cameraTransformMat = camera->GetCompositeProjectionTransformMatrix( renderer->GetTiledAspectRatio(), 0, 1 );

		//Generate the image
		renderWindow->Render();

		//Extract the corresponding LabelMap
		float *zbuff = renderWindow->GetZbufferData(0, 0, outputSize[0]-1, outputSize[1]-1);
		unsigned char *pixbuff = renderWindow->GetPixelData(0, 0, outputSize[0]-1, outputSize[1]-1, 0);

		double dist;

		OutputImageType::PixelType *outBuff = image->GetBufferPointer();

		OutputLabelMapType::IndexType index;
		OutputLabelMapType::LabelType label, currentLabel;
		OutputLabelMapType::LengthType length;
		labelMap->ClearLabels();

		currentLabel = 0;
		switch(m_direction) {
			case SAGITTAL:
				for (unsigned j=0 ; j<outputSize[1] ; j++) {
					if (currentLabel!=0) { //finalize a run
						index[0] = index[0]-length+1; 
						if ( !labelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
							OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
							labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); labelMap->AddLabelObject(labelObject);
						}
						else { labelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
					}
					currentLabel = 0;
					for (unsigned i=0 ; i<outputSize[0] ; i++) {
						dist = (zbuff[i+j*outputSize[0]]-cameraTransformMat->GetElement(2,3))/cameraTransformMat->GetElement(2,0) - sceneBBox(0);
						outBuff[(outputSize[0]-i-1)+j*outputSize[0]] = m_function->Evaluate( dist );

						//not sure why this /2 is necessary, but it does in the ToyApplication to recover the correct label / id
						label = (255*255*(unsigned short)(pixbuff[3*(i+j*outputSize[0])])+255*(unsigned short)(pixbuff[3*(i+j*outputSize[0])+1])+(unsigned short)(pixbuff[3*(i+j*outputSize[0])+2]))/2;
						if (label!=currentLabel) { //start a new run
							if (currentLabel!=0) { //need to finalize the current run if any
								index[0] = index[0]-length+1; //flip the indices							
								if ( !labelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
									OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
									labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); labelMap->AddLabelObject(labelObject);
								}
								else { labelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
							}
							//we start a new run
							currentLabel = label; index[0] = (outputSize[0]-i-1); index[1] = j; length = 1;
						}
						else { length++; } //continuing the current run
					}
				}
				if (currentLabel!=0) { //finalize the current run
					index[0] = index[0]-length+1; 
					if ( !labelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
						OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
						labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length);	labelMap->AddLabelObject(labelObject);
					}
					else { labelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
				}
				break;
			case CORONAL:
				for (unsigned j=0 ; j<outputSize[1] ; j++) {
					if (currentLabel!=0) { //finalize a run
						if ( !labelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
							OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
							labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); labelMap->AddLabelObject(labelObject);
						}
						else { labelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
					}
					currentLabel = 0;
					for (unsigned i=0 ; i<outputSize[0] ; i++) {
						dist = (zbuff[i+j*outputSize[0]]-cameraTransformMat->GetElement(2,3))/cameraTransformMat->GetElement(2,1) - sceneBBox(2);
						outBuff[i+j*outputSize[0]] = m_function->Evaluate( dist );

						//not sure why this /2 is necessary, but it does in the ToyApplication to recover the correct label / id
						label = (255*255*(unsigned short)(pixbuff[3*(i+j*outputSize[0])])+255*(unsigned short)(pixbuff[3*(i+j*outputSize[0])+1])+(unsigned short)(pixbuff[3*(i+j*outputSize[0])+2]))/2;
						if (label!=currentLabel) { //starting a new run							
							if (currentLabel!=0) { //need to finalize the current run if any
								if ( !labelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
									OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
									labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); labelMap->AddLabelObject(labelObject);
								}
								else { labelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
							}
							//we start a new run
							currentLabel = label; index[0] = i; index[1] = j; length = 1;
						}
						else { length++; } //continuing the current run
					}
				}
				if (currentLabel!=0) { //finalize a run
					if ( !labelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
						OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
						labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); labelMap->AddLabelObject(labelObject);
					}
					else { labelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
				}
				break;
			case HORIZONTAL:
				for (unsigned j=0 ; j<outputSize[1] ; j++) {
					if (currentLabel!=0) { //finalize a run
						index[0] = index[0]-length+1; 
						if ( !labelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
							OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
							labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); labelMap->AddLabelObject(labelObject);
						}
						else { labelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
					}
					currentLabel = 0;
					for (unsigned i=0 ; i<outputSize[0] ; i++) {
						dist = (zbuff[i+j*outputSize[0]]-cameraTransformMat->GetElement(2,3))/cameraTransformMat->GetElement(2,2) - sceneBBox(4);
						outBuff[(outputSize[0]-i-1)+j*outputSize[0]] = m_function->Evaluate( dist );

						//not sure why this /2 is necessary, but it does in the ToyApplication to recover the correct label / id
						label = (255*255*(unsigned short)(pixbuff[3*(i+j*outputSize[0])])+255*(unsigned short)(pixbuff[3*(i+j*outputSize[0])+1])+(unsigned short)(pixbuff[3*(i+j*outputSize[0])+2]))/2;
						if (label!=currentLabel) { //start a new run
							if (currentLabel!=0) { //need to finalize the current run if any 
								index[0] = index[0]-length+1; 
								if ( !labelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
									OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
									labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); labelMap->AddLabelObject(labelObject);
								}
								else { labelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
							}
							//we start a new run
							currentLabel = label; index[0] = (outputSize[0]-i-1); index[1] = j; length = 1;
						}
						else { length++; }//continuing the current run
					}
				}
				if (currentLabel!=0) { //finalize a run
					index[0] = index[0]-length+1; 
					if ( !labelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
						OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
						labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); labelMap->AddLabelObject(labelObject);
					}
					else { labelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
				}
				break;
		}

		delete [] zbuff;
		delete [] pixbuff;

		return true;
	}


	/** Same as before, but only compute the output image, not the label map. 
	* it is however, not faster...
	*/
	bool GetOffContextObjectImage(ObjectInScene *objectPtr, OutputImageType* image) {
		//if (!m_imageAllocated) throw DeformableModelException("SimpleUniformSensor::GetOffContextObjectImage the output data must be allocated first");
		//if (!objectPtr) return false;
		////compute the bounding box of interest and the corresponding image grid
		//vnl_vector<double> tmpBBox, roiBBox, sceneBBox = m_scene->GetSceneBoundingBox();
		//if (!IntersectionBoundingBoxes( sceneBBox, objectPtr->obj->GetPhysicalBoundingBox(), &tmpBBox )) return false;
		//roiBBox = GetSensedBoundingBox(tmpBBox);
		////
		//OutputImageType::PointType  outputOrigin;
		//OutputImageType::RegionType outputRegion;
		////OutputImageType::SizeType   outputSize;
		//ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing<2>(roiBBox, m_outputSpacing.GetVnlVector(), &outputOrigin, &outputRegion);
		////outputSize = outputRegion.GetSize();
		//image->SetSpacing( m_outputSpacing );
		//image->SetOrigin(  outputOrigin  );
		//image->SetRegions( outputRegion  );
		//image->Allocate();

		////Setup the view, camera, light, ...
		//m_offContextMapper->SetInput( objectPtr->obj->GetObjectAsVTKPolyData() );
		//

		//renderer->AddActor(actor);

		//vtkRenderWindow *renderWindow = vtkRenderWindow::New(); 
		//renderWindow->SetOffScreenRendering(1);

		//vtkCamera *camera = vtkCamera::New();
		//camera->ParallelProjectionOn();

		//renderWindow->SetSize(outputRegion.GetSize(0), outputRegion.GetSize(1));
		//renderWindow->AddRenderer( renderer ); 

		////always look toward the center of the region of interest
		////keep the distance between the camera and scene fixed, only translate the view box
		//camera->SetFocalPoint( (tmpBBox(0)+tmpBBox(1))/2.0, (tmpBBox(2)+tmpBBox(3))/2.0, (tmpBBox(4)+tmpBBox(5))/2.0 );
		//switch (m_direction) {
		//case SAGITTAL: 
		//	camera->SetPosition( sceneBBox(0)-m_cameraOffset, (tmpBBox(2)+tmpBBox(3))/2.0, (tmpBBox(4)+tmpBBox(5))/2.0 ); 
		//	camera->SetViewUp(0,0,1);
		//	camera->SetParallelScale( (tmpBBox(5)-tmpBBox(4)) / 2.0 );
		//	camera->SetClippingRange( 0, m_cameraOffset + sceneBBox(1)-sceneBBox(0) );
		//	break; //if the observation direction is SAGITTAL, then the axes of the output image correspond to the (y,z) plane in 3D
		//case CORONAL:
		//	camera->SetPosition( (tmpBBox(0)+tmpBBox(1))/2.0, sceneBBox(2)-m_cameraOffset, (tmpBBox(4)+tmpBBox(5))/2.0 );
		//	camera->SetViewUp(0,0,1);
		//	camera->SetParallelScale( (tmpBBox(5)-tmpBBox(4)) / 2.0 );
		//	camera->SetClippingRange( 0, m_cameraOffset + sceneBBox(3)-sceneBBox(2) );
		//	break; //CORONAL -> (x,z) plane
		//case HORIZONTAL:
		//	camera->SetPosition( (tmpBBox(0)+tmpBBox(1))/2.0, (tmpBBox(2)+tmpBBox(3))/2.0, sceneBBox(4)-m_cameraOffset ); 
		//	camera->SetViewUp(0,1,0);
		//	camera->SetParallelScale( (tmpBBox(3)-tmpBBox(2)) / 2.0 );
		//	camera->SetClippingRange( 0, m_cameraOffset + sceneBBox(5)-sceneBBox(4) ); //doing so, the zubbfer
		//	break; //HORIZONTAL -> (x,y)
		//}

		//renderer->SetActiveCamera(camera);
		//renderer->RemoveAllLights(); renderer->AddLight(m_light);

		//vtkMatrix4x4 *cameraTransformMat = camera->GetCompositeProjectionTransformMatrix( renderer->GetTiledAspectRatio(), 0, 1 );

		////Generate the image
		//renderWindow->Render();

		////Extract the corresponding LabelMap
		//float *zbuff = renderWindow->GetZbufferData(0, 0, outputRegion.GetSize(0)-1, outputRegion.GetSize(1)-1);

		//double dist;

		//OutputImageType::PixelType *outBuff = image->GetBufferPointer();

		//int jw;
		//switch(m_direction) {
		//	case SAGITTAL:
		//		for (unsigned j=0 ; j<outputRegion.GetSize(1) ; j++) {
		//			jw = j*outputRegion.GetSize(0);
		//			for (unsigned i=0 ; i<outputRegion.GetSize(0) ; i++) {
		//				dist = (zbuff[i+jw]-cameraTransformMat->Element[2][3])/cameraTransformMat->Element[2][0] - sceneBBox(0);
		//				outBuff[(outputRegion.GetSize(0)-i-1)+jw] = m_function->Evaluate( dist );
		//			}
		//		}
		//		break;
		//	case CORONAL:
		//		for (unsigned j=0 ; j<outputRegion.GetSize(1) ; j++) {
		//			jw = j*outputRegion.GetSize(0);
		//			for (unsigned i=0 ; i<outputRegion.GetSize(0) ; i++) {
		//				dist = (zbuff[i+jw]-cameraTransformMat->Element[2][3])/cameraTransformMat->Element[2][1] - sceneBBox(2);
		//				outBuff[i+jw] = m_function->Evaluate( dist );
		//			}
		//		}
		//		break;
		//	case HORIZONTAL:
		//		for (unsigned j=0 ; j<outputRegion.GetSize(1) ; j++) {
		//			for (unsigned i=0 ; i<outputRegion.GetSize(0) ; i++) {
		//				dist = (zbuff[i+jw]-cameraTransformMat->Element[2][3])/cameraTransformMat->Element[2][2] - sceneBBox(4);
		//				outBuff[(outputRegion.GetSize(0)-i-1)+jw] = m_function->Evaluate( dist );
		//			}
		//		}
		//		break;
		//}

		//delete [] zbuff;

		//renderer->Delete();
		//map->Delete();
		//actor->Delete();
		//renderWindow->Delete();
		//camera->Delete();

		//return true;
		if (!m_imageAllocated) AllocateOutputImage();
		if (!objectPtr) return false;
		//compute the bounding box of interest and the corresponding image grid
		vnl_vector<double> tmpBBox, roiBBox, sceneBBox = m_scene->GetSceneBoundingBox();
		if (!IntersectionBoundingBoxes( sceneBBox, objectPtr->obj->GetPhysicalBoundingBox(), &tmpBBox )) return false;
		roiBBox = GetSensedBoundingBox(tmpBBox);
		//
		OutputImageType::PointType  outputOrigin;
		OutputImageType::RegionType outputRegion;
		//OutputImageType::SizeType   outputSize;
		ITKGridImageInformationFromPhysicalBoundingBoxAndSpacing<2>(roiBBox, m_outputSpacing.GetVnlVector(), &outputOrigin, &outputRegion);
		//outputSize = outputRegion.GetSize();
		image->SetSpacing( m_outputSpacing );
		image->SetOrigin(  outputOrigin  );
		image->SetRegions( outputRegion  );
		image->Allocate();

		//Setup the view, camera, light, ...
		vtkRenderer *renderer = vtkRenderer::New();  renderer->SetBackground(0,0,0);
		vtkPolyDataMapper *map = vtkPolyDataMapper::New(); map->SetInput( objectPtr->obj->GetObjectAsVTKPolyData() );
		vtkActor *actor = vtkActor::New(); actor->SetMapper(map); 

		renderer->AddActor(actor);

		vtkRenderWindow *renderWindow = vtkRenderWindow::New(); 
		renderWindow->SetOffScreenRendering(1);

		vtkCamera *camera = vtkCamera::New();
		camera->ParallelProjectionOn();

		renderWindow->SetSize(outputRegion.GetSize(0), outputRegion.GetSize(1));
		renderWindow->AddRenderer( renderer ); 

		//always look toward the center of the region of interest
		//keep the distance between the camera and scene fixed, only translate the view box
		camera->SetFocalPoint( (tmpBBox(0)+tmpBBox(1))/2.0, (tmpBBox(2)+tmpBBox(3))/2.0, (tmpBBox(4)+tmpBBox(5))/2.0 );
		switch (m_direction) {
		case SAGITTAL: 
			camera->SetPosition( sceneBBox(0)-m_cameraOffset, (tmpBBox(2)+tmpBBox(3))/2.0, (tmpBBox(4)+tmpBBox(5))/2.0 ); 
			camera->SetViewUp(0,0,1);
			camera->SetParallelScale( (tmpBBox(5)-tmpBBox(4)) / 2.0 );
			camera->SetClippingRange( 0, m_cameraOffset + sceneBBox(1)-sceneBBox(0) );
			break; //if the observation direction is SAGITTAL, then the axes of the output image correspond to the (y,z) plane in 3D
		case CORONAL:
			camera->SetPosition( (tmpBBox(0)+tmpBBox(1))/2.0, sceneBBox(2)-m_cameraOffset, (tmpBBox(4)+tmpBBox(5))/2.0 );
			camera->SetViewUp(0,0,1);
			camera->SetParallelScale( (tmpBBox(5)-tmpBBox(4)) / 2.0 );
			camera->SetClippingRange( 0, m_cameraOffset + sceneBBox(3)-sceneBBox(2) );
			break; //CORONAL -> (x,z) plane
		case HORIZONTAL:
			camera->SetPosition( (tmpBBox(0)+tmpBBox(1))/2.0, (tmpBBox(2)+tmpBBox(3))/2.0, sceneBBox(4)-m_cameraOffset ); 
			camera->SetViewUp(0,1,0);
			camera->SetParallelScale( (tmpBBox(3)-tmpBBox(2)) / 2.0 );
			camera->SetClippingRange( 0, m_cameraOffset + sceneBBox(5)-sceneBBox(4) ); //doing so, the zubbfer
			break; //HORIZONTAL -> (x,y)
		}

		renderer->SetActiveCamera(camera);
		renderer->RemoveAllLights(); renderer->AddLight(m_light);

		vtkMatrix4x4 *cameraTransformMat = camera->GetCompositeProjectionTransformMatrix( renderer->GetTiledAspectRatio(), 0, 1 );

		//Generate the image
		renderWindow->Render();

		//Extract the corresponding LabelMap
		float *zbuff = renderWindow->GetZbufferData(0, 0, outputRegion.GetSize(0)-1, outputRegion.GetSize(1)-1);

		double dist;

		OutputImageType::PixelType *outBuff = image->GetBufferPointer();

		int jw;
		switch(m_direction) {
			case SAGITTAL:
				for (unsigned j=0 ; j<outputRegion.GetSize(1) ; j++) {
					jw = j*outputRegion.GetSize(0);
					for (unsigned i=0 ; i<outputRegion.GetSize(0) ; i++) {
						dist = (zbuff[i+jw]-cameraTransformMat->Element[2][3])/cameraTransformMat->Element[2][0] - sceneBBox(0);
						outBuff[(outputRegion.GetSize(0)-i-1)+jw] = m_function->Evaluate( dist );
					}
				}
				break;
			case CORONAL:
				for (unsigned j=0 ; j<outputRegion.GetSize(1) ; j++) {
					jw = j*outputRegion.GetSize(0);
					for (unsigned i=0 ; i<outputRegion.GetSize(0) ; i++) {
						dist = (zbuff[i+jw]-cameraTransformMat->Element[2][3])/cameraTransformMat->Element[2][1] - sceneBBox(2);
						outBuff[i+jw] = m_function->Evaluate( dist );
					}
				}
				break;
			case HORIZONTAL:
				for (unsigned j=0 ; j<outputRegion.GetSize(1) ; j++) {
					jw = j*outputRegion.GetSize(0);
					for (unsigned i=0 ; i<outputRegion.GetSize(0) ; i++) {
						dist = (zbuff[i+jw]-cameraTransformMat->Element[2][3])/cameraTransformMat->Element[2][2] - sceneBBox(4);
						outBuff[(outputRegion.GetSize(0)-i-1)+jw] = m_function->Evaluate( dist );
					}
				}
				break;
		}

		delete [] zbuff;

		renderer->Delete();
		map->Delete();
		actor->Delete();
		renderWindow->Delete();
		camera->Delete();

		return true;
	}

protected:
	SimpleSEMSensor() : ImageSensor_Base() {
		m_direction = HORIZONTAL; m_dirAxis0=0;m_dirAxis1=1;m_dirAxis2=2;
		m_orientationParams.set_size(1); m_orientationParams(0)=m_direction;

		GPF_ExpDecrFunction<double, double>::Pointer exp_fct = GPF_ExpDecrFunction<double, double>::New();	
		exp_fct->SetParameter(250);
		m_function = exp_fct;
		m_appearanceFunctionFlag = true;

		m_renderWindow = vtkSmartPointer<vtkRenderWindow>::New(); 
		m_renderWindow->SetOffScreenRendering(1);

		m_camera = vtkSmartPointer<vtkCamera>::New();
		m_camera->ParallelProjectionOn();
		//m_cameraOffset = 10;
		//m_cameraTransformMat = vtkSmartPointer<vtkMatrix4x4>::New();

		m_light = vtkSmartPointer<vtkLight>::New();
		m_light->SetAmbientColor(1,1,1);
		m_light->SetSpecularColor(0,0,0);
		m_light->SetDiffuseColor(0,0,0);
		m_light->SetPosition( 0, 0, 0); //actually, the light position is irrelevant for ambient lights.

		//m_offContextRenderer = vtkSmartPointer<vtkRenderer>::New(); m_offContextRenderer->SetBackground(0,0,0);
		//m_offContextRenderer->RemoveAllLights(); m_offContextRenderer->AddLight(m_light);
		//m_offContextRenderWindow = vtkSmartPointer<vtkRenderWindow>::New(); m_offContextRenderWindow->SetOffScreenRendering(1);
		//m_offContextCamera->ParallelProjectionOn();
		//m_offContextRenderer->SetActiveCamera(m_offContextCamera);
		//m_offContextMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		//m_offContextActor = vtkSmartPointer<vtkActor>::New(); m_offContextActor->SetMapper(m_offContextMapper);
	}
	~SimpleSEMSensor() {};	

	ObservationDirectionType m_direction;
	unsigned int m_dirAxis0, m_dirAxis1, m_dirAxis2; //maps the sensor image axes with the original scene axes ; see SetObservationDirectionType

	vtkSmartPointer<vtkRenderWindow> m_renderWindow;
	vtkSmartPointer<vtkCamera> m_camera;
	vtkSmartPointer<vtkLight> m_light;
	double m_cameraOffset;
	vtkSmartPointer<vtkMatrix4x4> m_cameraTransformMat;

	vtkSmartPointer<vtkRenderer> m_offContextRenderer;
	vtkSmartPointer<vtkRenderWindow> m_offContextRenderWindow;
	vtkSmartPointer<vtkCamera> m_offContextCamera;
	vtkSmartPointer<vtkPolyDataMapper> m_offContextMapper;
	vtkSmartPointer<vtkActor> m_offContextActor;

	/** forces the output image to have the same resolution, dimension, etc... as the scene */
	void AllocateOutputImage() {	
		if (!m_scene) throw DeformableModelException("SimpleUniformSensor::AllocateOutputImage the scene must be set first!");

		//Get the useful characteristics from the scene
		InputLabelImageType::SpacingType	inputSpacing= m_scene->GetSceneSpacing();
		InputLabelImageType::PointType		inputOrigin = m_scene->GetSceneOrigin();
		InputLabelImageType::RegionType		sceneRegion = m_scene->GetSceneImageRegion();
		InputLabelImageType::SizeType		sceneSize	= sceneRegion.GetSize();

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
		m_renderWindow->SetSize(m_outputSize[0], m_outputSize[1]);
		m_renderWindow->AddRenderer( m_scene->GetSceneRenderer() ); 

		vnl_vector<double> sceneBBox = m_scene->GetSceneBoundingBox();

		//always look toward the center of the scene
		m_camera->SetFocalPoint( (sceneBBox(0)+sceneBBox(1))/2.0, (sceneBBox(2)+sceneBBox(3))/2.0, (sceneBBox(4)+sceneBBox(5))/2.0 );
		//IDEA: authorize also viewing from the other side, in this case the zbuffer will be ordered the other way around
		switch (m_direction) {
		case SAGITTAL: 
			m_cameraOffset = (sceneBBox(1)-sceneBBox(0))*0.5;
			m_camera->SetPosition( sceneBBox(0)-m_cameraOffset, (sceneBBox(2)+sceneBBox(3))/2.0, (sceneBBox(4)+sceneBBox(5))/2.0 ); 
			m_camera->SetViewUp(0,0,1);
			m_camera->SetParallelScale( (sceneBBox(5)-sceneBBox(4)) / 2.0 );
			m_camera->SetClippingRange( 0, m_cameraOffset + sceneBBox(1)-sceneBBox(0) );
			break; //if the observation direction is SAGITTAL, then the axes of the output image correspond to the (y,z) plane in 3D
		case CORONAL:
			m_cameraOffset = (sceneBBox(3)-sceneBBox(2))*0.25;
			m_camera->SetPosition( (sceneBBox(0)+sceneBBox(1))/2.0, sceneBBox(2)-m_cameraOffset, (sceneBBox(4)+sceneBBox(5))/2.0 );
			m_camera->SetViewUp(0,0,1);
			m_camera->SetParallelScale( (sceneBBox(5)-sceneBBox(4)) / 2.0 );
			m_camera->SetClippingRange( 0, m_cameraOffset + sceneBBox(3)-sceneBBox(2) );
			break; //CORONAL -> (x,z) plane
		case HORIZONTAL:
			m_cameraOffset = (sceneBBox(5)-sceneBBox(4))*0.25;
			m_camera->SetPosition( (sceneBBox(0)+sceneBBox(1))/2.0, (sceneBBox(2)+sceneBBox(3))/2.0, sceneBBox(4)-m_cameraOffset ); 
			m_camera->SetViewUp(0,1,0);
			m_camera->SetParallelScale( (sceneBBox(3)-sceneBBox(2)) / 2.0 );
			m_camera->SetClippingRange( 0, m_cameraOffset + sceneBBox(5)-sceneBBox(4) ); //doing so, the zubbfer
			break; //HORIZONTAL -> (x,y)
		}

		m_scene->GetSceneRenderer()->SetActiveCamera(m_camera);
		m_scene->GetSceneRenderer()->RemoveAllLights();
		m_scene->GetSceneRenderer()->AddLight(m_light);

		m_cameraTransformMat = m_camera->GetCompositeProjectionTransformMatrix( m_scene->GetSceneRenderer()->GetTiledAspectRatio(), 0, 1 );

		m_imageAllocated = true;
	}


	/** Update data in the requested region (CLEAN: use itk -> RequestedRegion instead?) */
	void Update_Data(typename InputLabelImageType::RegionType inputRegion = m_scene->m_regionWholeScene) {
		//TODO: optimize things by updating only the requested region, instead of the whole image...
		//even more I could refocus the camera to generate only the portion of interest
		//a first step however would be to get only a smaller portion of the zbuffer
		//not trivial for the labelmap , see other comments (top of the file, and functions below) about it.
std::cout<<"UPDATE_DATA, SIMPLESEMSENSOR... using vtk ; seems broken..."<<std::endl;
		vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New(); renderer->SetBackground(0,0,0);
		renderer->RemoveAllLights(); renderer->AddLight(m_light);

		SceneObjectIterator<SceneType> it(m_scene);
		for (it.GoToBegin() ; !it.IsAtEnd() ; ++it) { 
			//TODO: check that actor is uptodate... and remember that the object may not be instanciated!
			renderer->AddActor( it.GetObject()->actor ); 
		}
		
		vtkRenderWindow *renderWindow = vtkRenderWindow::New(); 
		renderWindow->SetOffScreenRendering(1);

		renderWindow->SetSize(m_outputSize[0], m_outputSize[1]);
		renderWindow->AddRenderer( m_scene->GetSceneRenderer() ); 

		renderWindow->Render(); // THE PROBLEM APPARENTLY COMES FROM CALLING ->Render multiple times on the same renderWindow...
		float *zbuff = renderWindow->GetZbufferData(0, 0, m_outputSize[0]-1, m_outputSize[1]-1);
		unsigned char *pixbuff = renderWindow->GetPixelData(0, 0, m_outputSize[0]-1, m_outputSize[1]-1, 0);

		//m_renderWindow->Render(); // THE LEAK IS RELATED TO THIS CALL!
		//float *zbuff = m_renderWindow->GetZbufferData(0, 0, m_outputSize[0]-1, m_outputSize[1]-1);
		//unsigned char *pixbuff = m_renderWindow->GetPixelData(0, 0, m_outputSize[0]-1, m_outputSize[1]-1, 0);
		double dist;

		vnl_vector<double> sceneBBox = m_scene->GetSceneBoundingBox();

		OutputImageType::PixelType *outBuff = m_outputImage->GetBufferPointer();

		OutputLabelMapType::IndexType index;
		OutputLabelMapType::LabelType label, currentLabel;
		OutputLabelMapType::LengthType length;
		m_outputLabelMap->ClearLabels();

		currentLabel = 0;
		switch(m_direction) {
			case SAGITTAL:
				for (unsigned j=0 ; j<m_outputSize[1] ; j++) {
					if (currentLabel!=0) { //finalize a run
						index[0] = index[0]-length+1; 
						if ( !m_outputLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
							OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
							labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); m_outputLabelMap->AddLabelObject(labelObject);
						}
						else { m_outputLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
					}
					currentLabel = 0;
					for (unsigned i=0 ; i<m_outputSize[0] ; i++) {
						dist = (zbuff[i+j*m_outputSize[0]]-m_cameraTransformMat->GetElement(2,3))/m_cameraTransformMat->GetElement(2,0) - sceneBBox(0);
						outBuff[(m_outputSize[0]-i-1)+j*m_outputSize[0]] = m_function->Evaluate( dist );

						//not sure why this /2 is necessary, but it does in the ToyApplication to recover the correct label / id
						label = (255*255*(unsigned short)(pixbuff[3*(i+j*m_outputSize[0])])+255*(unsigned short)(pixbuff[3*(i+j*m_outputSize[0])+1])+(unsigned short)(pixbuff[3*(i+j*m_outputSize[0])+2]))/2;
						if (label!=currentLabel) { //start a new run
							if (currentLabel!=0) { //need to finalize the current run if any
								index[0] = index[0]-length+1; //flip the indices							
								if ( !m_outputLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
									OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
									labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); m_outputLabelMap->AddLabelObject(labelObject);
								}
								else { m_outputLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
							}
							//we start a new run
							currentLabel = label; index[0] = (m_outputSize[0]-i-1); index[1] = j; length = 1;
						}
						else { length++; } //continuing the current run
					}
				}
				if (currentLabel!=0) { //finalize the current run
					index[0] = index[0]-length+1; 
					if ( !m_outputLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
						OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
						labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length);	m_outputLabelMap->AddLabelObject(labelObject);
					}
					else { m_outputLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
				}
				break;
			case CORONAL:
				for (unsigned j=0 ; j<m_outputSize[1] ; j++) {
					if (currentLabel!=0) { //finalize a run
						if ( !m_outputLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
							OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
							labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); m_outputLabelMap->AddLabelObject(labelObject);
						}
						else { m_outputLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
					}
					currentLabel = 0;
					for (unsigned i=0 ; i<m_outputSize[0] ; i++) {
						dist = (zbuff[i+j*m_outputSize[0]]-m_cameraTransformMat->GetElement(2,3))/m_cameraTransformMat->GetElement(2,1) - sceneBBox(2);
						outBuff[i+j*m_outputSize[0]] = m_function->Evaluate( dist );

						//not sure why this /2 is necessary, but it does in the ToyApplication to recover the correct label / id
						label = (255*255*(unsigned short)(pixbuff[3*(i+j*m_outputSize[0])])+255*(unsigned short)(pixbuff[3*(i+j*m_outputSize[0])+1])+(unsigned short)(pixbuff[3*(i+j*m_outputSize[0])+2]))/2;
						if (label!=currentLabel) { //starting a new run							
							if (currentLabel!=0) { //need to finalize the current run if any
								if ( !m_outputLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
									OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
									labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); m_outputLabelMap->AddLabelObject(labelObject);
								}
								else { m_outputLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
							}
							//we start a new run
							currentLabel = label; index[0] = i; index[1] = j; length = 1;
						}
						else { length++; } //continuing the current run
					}
				}
				if (currentLabel!=0) { //finalize a run
					if ( !m_outputLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
						OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
						labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); m_outputLabelMap->AddLabelObject(labelObject);
					}
					else { m_outputLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
				}
				break;
			case HORIZONTAL:
				for (unsigned j=0 ; j<m_outputSize[1] ; j++) {
					if (currentLabel!=0) { //finalize a run
						index[0] = index[0]-length+1; 
						if ( !m_outputLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
							OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
							labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); m_outputLabelMap->AddLabelObject(labelObject);
						}
						else { m_outputLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
					}
					currentLabel = 0;
					for (unsigned i=0 ; i<m_outputSize[0] ; i++) {
						dist = (zbuff[i+j*m_outputSize[0]]-m_cameraTransformMat->GetElement(2,3))/m_cameraTransformMat->GetElement(2,2) - sceneBBox(4);
						outBuff[(m_outputSize[0]-i-1)+j*m_outputSize[0]] = m_function->Evaluate( dist );

						//not sure why this /2 is necessary, but it does in the ToyApplication to recover the correct label / id
						label = (255*255*(unsigned short)(pixbuff[3*(i+j*m_outputSize[0])])+255*(unsigned short)(pixbuff[3*(i+j*m_outputSize[0])+1])+(unsigned short)(pixbuff[3*(i+j*m_outputSize[0])+2]))/2;
						if (label!=currentLabel) { //start a new run
							if (currentLabel!=0) { //need to finalize the current run if any 
								index[0] = index[0]-length+1; 
								if ( !m_outputLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
									OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
									labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); m_outputLabelMap->AddLabelObject(labelObject);
								}
								else { m_outputLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
							}
							//we start a new run
							currentLabel = label; index[0] = (m_outputSize[0]-i-1); index[1] = j; length = 1;
						}
						else { length++; }//continuing the current run
					}
				}
				if (currentLabel!=0) { //finalize a run
					index[0] = index[0]-length+1; 
					if ( !m_outputLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
						OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
						labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); m_outputLabelMap->AddLabelObject(labelObject);
					}
					else { m_outputLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
				}
				break;
		}

		delete [] zbuff;
		delete [] pixbuff;
		renderWindow->Delete();
	}

	//The following is an unfinished attempt for updating the images
	//this should work for the direct sensor output, but not the label output... 
	//I would need to identify which objects are in the region
	//delete these parts from the label objects
	//append the new bits of lines to old objects, etc...
	//A possibility would be to generate a true label Image... and then convert it as a labelmap, ....

	//void SetCameraRequestedRegion(typename InputLabelImageType::RegionType requestedRegion) {
	//	vnl_matrix<double> requestedBBox(6) = m_scene->GetSceneBoundingBox();
	//	//Get the useful characteristics from the scene
	//	InputLabelImageType::PointType		inputOrigin = m_scene->GetSceneOrigin();

	//	//
	//	requestedBBox(2*m_dirAxis0) = inputOrigin[m_dirAxis0]+requestedRegion.GetIndex(m_dirAxis0)*inputSpacing[m_dirAxis0];
	//	requestedBBox(2*m_dirAxis0+1) = requestedBBox(2*m_dirAxis0) + requestedRegion.GetSize(m_dirAxis0)*m_outputSpacing[0];
	//	requestedBBox(2*m_dirAxis1) = inputOrigin[m_dirAxis1]+requestedRegion.GetIndex(m_dirAxis1)*inputSpacing[m_dirAxis1];
	//	requestedBBox(2*m_dirAxis1+1) = requestedBBox(2*m_dirAxis1) + requestedRegion.GetSize(m_dirAxis1)*m_outputSpacing[1];

	//	//
	//	m_renderWindow->SetSize(requestedRegion.GetSize(m_dirAxis0), requestedRegion.GetSize(m_dirAxis1));

	//	//always look toward the center of the scene
	//	m_camera->SetFocalPoint( (requestedBBox(0)+requestedBBox(1))/2.0, (requestedBBox(2)+requestedBBox(3))/2.0, (requestedBBox(4)+requestedBBox(5))/2.0 );
	//	//IDEA: authorize also viewing from the other side, in this case the zbuffer will be ordered the other way around
	//	switch (m_direction) {
	//	case SAGITTAL: 
	//		m_camera->SetPosition( requestedBBox(0)-m_cameraOffset, (requestedBBox(2)+requestedBBox(3))/2.0, (requestedBBox(4)+requestedBBox(5))/2.0 ); 
	//		m_camera->SetParallelScale( (requestedBBox(5)-requestedBBox(4)) / 2.0 );
	//		break; //if the observation direction is SAGITTAL, then the axes of the output image correspond to the (y,z) plane in 3D
	//	case CORONAL:
	//		m_camera->SetPosition( (requestedBBox(0)+requestedBBox(1))/2.0, requestedBBox(2)-m_cameraOffset, (requestedBBox(4)+requestedBBox(5))/2.0 );
	//		m_camera->SetParallelScale( (requestedBBox(5)-requestedBBox(4)) / 2.0 );
	//		break; //CORONAL -> (x,z) plane
	//	case HORIZONTAL:
	//		m_camera->SetPosition( (requestedBBox(0)+requestedBBox(1))/2.0, (requestedBBox(2)+requestedBBox(3))/2.0, requestedBBox(4)-m_cameraOffset ); 
	//		m_camera->SetParallelScale( (requestedBBox(3)-requestedBBox(2)) / 2.0 );
	//		break; //HORIZONTAL -> (x,y)
	//	}

	//	m_scene->GetSceneRenderer()->SetActiveCamera(m_camera);
	//}

	///** Update data in the requested region (CLEAN: use itk -> RequestedRegion instead?) */
	//void Update_Data(typename InputLabelImageType::RegionType inputRegion = m_scene->m_regionWholeScene) {

	//	SetCameraRequestedRegion(inputRegion);

	//	m_renderWindow->Render();
	//	float *zbuff = m_renderWindow->GetZbufferData(0, 0, inputRegion.GetSize(m_dirAxis0)-1, inputRegion.GetSize(m_dirAxis1)-1);
	//	unsigned char *pixbuff = m_renderWindow->GetPixelData(0, 0, inputRegion.GetSize(m_dirAxis0)-1, inputRegion.GetSize(m_dirAxis1)-1, 0);
	//	double dist;

	//	vnl_vector<double> sceneBBox = m_scene->GetSceneBoundingBox();

	//	OutputImageType::PixelType *outBuff = m_outputImage->GetBufferPointer();

	//	OutputLabelMapType::IndexType index;
	//	OutputLabelMapType::LabelType label, currentLabel;
	//	OutputLabelMapType::LengthType length;
	//	m_outputLabelMap->ClearLabels();

	//	currentLabel = 0;
	//	long add_cam, add_out;
	//	switch(m_direction) {
	//		case SAGITTAL:
	//			//browse the camera output, and fill in the output images of the sensor ; at least the requested regions
	//			// TODO => modify accordingly the start and end point indices of the loops
	//			// TODO ; do the same for the other directions.
	//			for (unsigned j=0 ; j<inputRegion.GetSize(m_dirAxis1) ; j++) {
	//				if (currentLabel!=0) { //finalize a run
	//					index[0] = index[0]-length+1; //flip the start and end indices (vtk camera image is left/right flipped in this case)
	//					if ( !m_outputLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
	//						OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
	//						labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); m_outputLabelMap->AddLabelObject(labelObject);
	//					}
	//					else { m_outputLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
	//				}
	//				currentLabel = 0;
	//				for (unsigned i=0 ; i<inputRegion.GetSize(m_dirAxis0) ; i++) {
	//					add_cam = i+j*inputRegion.GetSize(m_dirAxis0);
	//					add_out = (m_outputSize[0]-inputRegion.GetIndex(m_dirAxis0)-i-1)+(j+inputRegion.GetIndex(m_dirAxis1))*m_outputSize[0];
	//					dist = (zbuff[add_cam]-m_cameraTransformMat->GetElement(2,3))/m_cameraTransformMat->GetElement(2,0) - sceneBBox(0);
	//					outBuff[add_out] = m_function->Evaluate( dist );

	//					//not sure why this /2 is necessary, but it does in the ToyApplication to recover the correct label / id
	//					label = (255*255*(unsigned short)(pixbuff[3*add_cam])+255*(unsigned short)(pixbuff[3*add_cam+1])+(unsigned short)(pixbuff[3*add_cam+2]))/2;
	//					if (label!=currentLabel) { //start a new run
	//						if (currentLabel!=0) { //need to finalize the current run if any
	//							index[0] = index[0]-length+1; //flip the start and end indices (vtk camera image is left/right flipped in this case)
	//							if ( !m_outputLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
	//								OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
	//								labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); m_outputLabelMap->AddLabelObject(labelObject);
	//							}
	//							else { m_outputLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
	//						}
	//						//we start a new run - index is relative to the output image, not the requested region
	//						currentLabel = label;  
	//						index[0] = (m_outputSize[0]-inputRegion.GetIndex(m_dirAxis0)-i-1);
	//						index[1] = j+inputRegion.GetIndex(m_dirAxis1); length = 1;
	//					}
	//					else { length++; } //continuing the current run
	//				}
	//			}
	//			if (currentLabel!=0) { //finalize the current run
	//				index[0] = index[0]-length+1; 
	//				if ( !m_outputLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
	//					OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
	//					labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length);	m_outputLabelMap->AddLabelObject(labelObject);
	//				}
	//				else { m_outputLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
	//			}
	//			break;
	//		case CORONAL:
	//			for (unsigned j=0 ; j<inputRegion.GetSize(m_dirAxis1) ; j++) {
	//				if (currentLabel!=0) { //finalize a run
	//					if ( !m_outputLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
	//						OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
	//						labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); m_outputLabelMap->AddLabelObject(labelObject);
	//					}
	//					else { m_outputLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
	//				}
	//				currentLabel = 0;
	//				for (unsigned i=0 ; i<inputRegion.GetSize(m_dirAxis0) ; i++) {
	//					add_cam = i+j*inputRegion.GetSize(m_dirAxis0);
	//					add_out = (i+inputRegion.GetIndex(m_dirAxis0)) + (j+inputRegion.GetIndex(m_dirAxis1))*m_outputSize[0];
	//					dist = (zbuff[add_cam]-m_cameraTransformMat->GetElement(2,3))/m_cameraTransformMat->GetElement(2,0) - sceneBBox(0);
	//					outBuff[add_out] = m_function->Evaluate( dist );

	//					//not sure why this /2 is necessary, but it does in the ToyApplication to recover the correct label / id
	//					label = (255*255*(unsigned short)(pixbuff[3*add_cam])+255*(unsigned short)(pixbuff[3*add_cam+1])+(unsigned short)(pixbuff[3*add_cam+2]))/2;
	//					if (label!=currentLabel) { //starting a new run							
	//						if (currentLabel!=0) { //need to finalize the current run if any
	//							if ( !m_outputLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
	//								OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
	//								labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); m_outputLabelMap->AddLabelObject(labelObject);
	//							}
	//							else { m_outputLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
	//						}
	//						//we start a new run
	//						currentLabel = label; 
	//						index[0] = i+inputRegion.GetIndex(m_dirAxis0); 
	//						index[1] = j+inputRegion.GetIndex(m_dirAxis1); length = 1;
	//					}
	//					else { length++; } //continuing the current run
	//				}
	//			}				
	//			if (currentLabel!=0) { //finalize a run
	//				if ( !m_outputLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
	//					OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
	//					labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); m_outputLabelMap->AddLabelObject(labelObject);
	//				}
	//				else { m_outputLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
	//			}
	//			break;
	//		case HORIZONTAL:
	//			for (unsigned j=0 ; j<inputRegion.GetSize(m_dirAxis1) ; j++) {
	//				if (currentLabel!=0) { //finalize a run
	//					index[0] = index[0]-length+1; 
	//					if ( !m_outputLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
	//						OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
	//						labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); m_outputLabelMap->AddLabelObject(labelObject);
	//					}
	//					else { m_outputLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
	//				}
	//				currentLabel = 0;
	//				for (unsigned i=0 ; i<inputRegion.GetSize(m_dirAxis0) ; i++) {
	//					//add = (inputRegion.GetSize(m_dirAxis0)-i-1)+j*inputRegion.GetSize(m_dirAxis0);
	//					add_cam = i+j*inputRegion.GetSize(m_dirAxis0);
	//					add_out = (m_outputSize[0]-inputRegion.GetIndex(m_dirAxis0)-i-1)+(j+inputRegion.GetIndex(m_dirAxis1))*m_outputSize[0];
	//					dist = (zbuff[add_cam]-m_cameraTransformMat->GetElement(2,3))/m_cameraTransformMat->GetElement(2,0) - sceneBBox(0);
	//					outBuff[add_out] = m_function->Evaluate( dist );

	//					//not sure why this /2 is necessary, but it does in the ToyApplication to recover the correct label / id
	//					label = (255*255*(unsigned short)(pixbuff[3*add_cam])+255*(unsigned short)(pixbuff[3*add_cam+1])+(unsigned short)(pixbuff[3*add_cam+2]))/2;
	//					if (label!=currentLabel) { //start a new run
	//						if (currentLabel!=0) { //need to finalize the current run if any 
	//							index[0] = index[0]-length+1; 
	//							if ( !m_outputLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
	//								OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
	//								labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); m_outputLabelMap->AddLabelObject(labelObject);
	//							}
	//							else { m_outputLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
	//						}
	//						//we start a new run
	//						currentLabel = label; 
	//						index[0] = (m_outputSize[0]-inputRegion.GetIndex(m_dirAxis0)-i-1);
	//						index[1] = j+inputRegion.GetIndex(m_dirAxis1); length = 1;
	//					}
	//					else { length++; }//continuing the current run
	//				}
	//			}
	//			if (currentLabel!=0) { //finalize a run
	//				index[0] = index[0]-length+1; 
	//				if ( !m_outputLabelMap->HasLabel( currentLabel ) ) { //create an object and set a new line for it
	//					OutputLabelObjectType::Pointer labelObject = OutputLabelObjectType::New();
	//					labelObject->SetLabel( currentLabel ); labelObject->AddLine(index, length); m_outputLabelMap->AddLabelObject(labelObject);
	//				}
	//				else { m_outputLabelMap->GetLabelObject( currentLabel )->AddLine(index, length); }
	//			}
	//			break;
	//	}
	//}


	////select the interesting region in the sensor image
	//InputLabelImageType *sceneLabelImage = m_scene->GetSceneAsLabelImage();
	//InputLabelImageType::SpacingType	inputSpacing = sceneLabelImage->GetSpacing();
	//InputLabelImageType::SizeType		inputSize    = inputRegion.GetSize();
	//InputLabelImageType::IndexType		inputStart	 = inputRegion.GetIndex();

	////set the region to scan in the output image
	//OutputImageType::SizeType	outputSize;			outputSize[0]  = inputSize[m_dirAxis0];	 outputSize[1]  = inputSize[m_dirAxis1];
	//OutputImageType::IndexType	outputStart;		outputStart[0] = inputStart[m_dirAxis0]; outputStart[1]	= inputStart[m_dirAxis1];
	//OutputImageType::RegionType	outputRegion;		outputRegion.SetSize(outputSize);		 outputRegion.SetIndex(outputStart);
	////std::cout<<"updating sensor region: start = "<<outputStart[0]<<", "<<outputStart[1]<<", size = "<<outputSize[0]<<", "<<outputSize[1]<<std::endl;
	//typedef itk::ImageRegionIteratorWithIndex<OutputImageType>		OutputIteratorType;		OutputIteratorType outputIt( m_outputImage, outputRegion );
	//typedef itk::ImageRegionIterator<OutputLabelImageType>			LabelOutputIteratorType;LabelOutputIteratorType labelOutputIt( m_outputLabelImage, outputRegion );
	//typedef itk::ImageRegionIteratorWithIndex<InputLabelImageType>	InputIteratorType;

	////define a neighborhood: any object that comes too close to a pixel that gets modified may see its dataCost invalidated
	//typedef itk::ConstNeighborhoodIterator< OutputLabelImageType > NeighborhoodIteratorType;
	//NeighborhoodIteratorType::RadiusType radius; 
	//OutputLabelImageType::RegionType neighbRegion; OutputLabelImageType::RegionType neighbIndex; OutputLabelImageType::SizeType neighbSize;
	//for (unsigned i=0 ; i<OutputLabelImageType::ImageDimension ; i++) { radius[i] = m_contextRadius; neighbSize[i]=1; }
	//NeighborhoodIteratorType neighbIt( radius, m_outputLabelImage, outputRegion );
	//neighbRegion.SetSize(neighbSize);

	//InputLabelImageType::RegionType lineRegion;
	//InputLabelImageType::SizeType lineSize;	lineSize[m_dirAxis0]=1;lineSize[m_dirAxis1]=1;lineSize[m_dirAxis2]=sceneLabelImage->GetLargestPossibleRegion().GetSize()[m_dirAxis2];
	//lineRegion.SetSize(lineSize); //size is fixed to a full line of voxels in the chosen direction

	//InputLabelImageType::IndexType lineStart, zindex;
	//lineStart[m_dirAxis2]=0; //always start a line at the 'first' pixel of the scene, indeed, there could be something in front of the requested region !
	////WARNING: value 0, and use of ++ assumes that the 'camera' is positioned at a coordinate <= min coordinate of the image.

	//OutputLabelImageType::PixelType initialLabel;
	////std::cout<<"  time to prepare the image "<<(clock()-t0)/((double)CLOCKS_PER_SEC)<<" s. ; line start = "<<lineStart[m_dirAxis2]<<", line size = "<<lineSize[m_dirAxis2]<<std::endl;
	////the output image is defined as a kind of depth map of the observed 'face' of the 3D scene.
	//bool ok; InputLabelImageType::PixelType inValue;
	//std::set<InputLabelImageType::PixelType> labelObjectsToInvalidate; std::set<InputLabelImageType::PixelType>::iterator setIt;

	//OutputImageType::PixelType initialValue, finalValue = 0;  OutputLabelImageType::PixelType label, finalLabel = 0;//default output value

	//for ( outputIt.GoToBegin(), labelOutputIt.GoToBegin() ; !outputIt.IsAtEnd(); ++outputIt, ++labelOutputIt) { //Iterate over the pixels of the output image
	//	initialLabel = labelOutputIt.Get();
	//	outputStart = outputIt.GetIndex();							//coordinates in the output image
	//	lineStart[m_dirAxis0]=outputStart[0];lineStart[m_dirAxis1]=outputStart[1];	//
	//	lineRegion.SetIndex(lineStart);
	//	InputIteratorType inputIt( sceneLabelImage, lineRegion );		//IDEA: could use a ImageLinearIterator instead
	//	//now, scan the line and set the output pixel value
	//	inputIt.GoToBegin(); ok = false; 
	//	initialValue = outputIt.Get(); finalValue = 0; finalLabel = 0;
	//	while (!ok) {
	//		inValue = inputIt.Get();
	//		if (inValue!=0) {
	//			finalLabel = (OutputLabelImageType::PixelType) inValue;
	//			zindex = inputIt.GetIndex(); finalValue = saturate_cast<double,OutputImageType::PixelType>(m_function->Evaluate( 0.5*zindex[m_dirAxis2]*inputSpacing[m_dirAxis2] )); 
	//			ok = true; //stop as soon as something as been found
	//		}
	//		if (inputIt.IsAtEnd()) ok=true; //IDEA: could go faster (maybe) by checking the maximum distance before the function value vanishes to 0...
	//		++inputIt;
	//	}
	//	outputIt.Set(finalValue);
	//	labelOutputIt.Set(finalLabel);

	//	//if the pixel has changed, invalidate the datacost of objects around...
	//	if ( (initialValue!=finalValue) || (initialLabel!=finalLabel) ) { //something has changed on the sensor image => look around and invalidate the data cost of objects around
	//		if (initialLabel!=0) labelObjectsToInvalidate.insert(initialLabel);
	//		if (m_contextRadius>0) {//look if there are any objects within a radius of m_radiusInvalidatingObjectDataCost ; and invalidate them too
	//			neighbRegion.SetIndex( outputIt.GetIndex() );
	//			neighbIt = NeighborhoodIteratorType( radius, m_outputLabelImage, neighbRegion ); neighbIt.GoToBegin();
	//			for (unsigned int i = 0; i < neighbIt.Size(); ++i) { label = neighbIt.GetPixel(i); if (label!=0) labelObjectsToInvalidate.insert(label); }
	//		}
	//		else { if (finalLabel!=0) labelObjectsToInvalidate.insert(finalLabel); }								
	//	}
	//}

	//for (setIt = labelObjectsToInvalidate.begin() ; setIt!= labelObjectsToInvalidate.end() ; setIt++) {
	//	ObjectInScene *obj = m_scene->GetObjectByLabel(*setIt);
	//	if ( obj !=0 ) { obj->dataCostFlag = false; }
	//}

	//}

public:

	//other possibility is to generate also a small image of that object, as if it was alone in the scene.
	/*double GetSingleObjectVisibilityPercentage( ObjectInScene *objectPtr ) {
	return 0;
	*/	//get the pointers to the useful images
	//SceneType::BinaryImageType *objectImage = objectIt->second->object->GetObjectAsBinaryImage();
	//SceneType::LabelImageType *sceneLabelImage = m_scene->GetSceneAsLabelImage();
	////get the regions containing the object in these images
	//SceneType::BinaryImageType::RegionType	objectSceneRegion = objectImage->GetLargestPossibleRegion();
	//SceneType::LabelImageType::RegionType	sceneBBRegion     = sceneLabelImage->GetLargestPossibleRegion();
	////look at the intersection = region potentially occupied by the object within the scene ; if it is not in the scene, than the percentage of visible pixels is 0
	//if (!ITKImagesIntersectionRegions<SceneType::LabelImageType, SceneType::BinaryImageType>(sceneLabelImage, objectImage, &sceneBBRegion, &objectSceneRegion))  return 0; 
	////get the sensed bounding box of the object, to get the region where it could possibly be on the sensed image
	//OutputLabelImageType::RegionType objectSensedImageRegion = this->GetSensedBoundingBox( sceneBBRegion );
	////number of visible pixels for this object: look at the GetLabelOutput()
	//unsigned int nbVisiblePixels = 0, nbPotentialPixels = 0;

	////nbVisiblePixels: look at the sensed label image, and count how many pixels have the right label.
	//typedef itk::ImageRegionConstIteratorWithIndex< OutputLabelImageType > SensedIteratorType;
	//SensedIteratorType labelIt( this->GetLabelOutput(), objectSensedImageRegion );
	//OutputLabelImageType::PixelType requestedLabel = objectIt->first;
	//bool ok; InputLabelImageType::PixelType inValue;
	//for ( labelIt.GoToBegin() ; !labelIt.IsAtEnd(); ++labelIt) { //Iterate over the pixels in the requested region of the sensed label image
	//	if (labelIt.Get() == requestedLabel) { nbVisiblePixels++; }
	//}

	////nbPotentialPixels: if it has a different label (but not zero), look at the list of pixels in objectIt->second->pixelSet, and see how many SHOULD be visible after the projection
	//itk::Image<unsigned char, OutputImageType::ImageDimension>::Pointer singleObjectBinaryImage = itk::Image<unsigned char, OutputImageType::ImageDimension>::New();
	//itk::Image<unsigned char, OutputImageType::ImageDimension>::SpacingType tmpSpacing= m_outputLabelImage->GetSpacing();
	//singleObjectBinaryImage->SetSpacing( tmpSpacing );
	//itk::Image<unsigned char, OutputImageType::ImageDimension>::PointType	tmpOrigin = m_outputLabelImage->GetOrigin();
	//itk::Image<unsigned char, OutputImageType::ImageDimension>::RegionType	tmpRegion = objectSensedImageRegion;
	//itk::Image<unsigned char, OutputImageType::ImageDimension>::IndexType	tmpIndex, refIndex = objectSensedImageRegion.GetIndex();
	//for (unsigned i=0 ; i<OutputImageType::ImageDimension ; i++) { 
	//	tmpIndex[i] = 0; 
	//	tmpOrigin[0] += refIndex[i] * tmpSpacing[i];
	//} 
	//tmpRegion.SetIndex( tmpIndex );
	//singleObjectBinaryImage->SetOrigin( tmpOrigin);
	//singleObjectBinaryImage->SetRegions(tmpRegion); //size is already set, as the region is initialized with objectSensedImageRegion

	//try { singleObjectBinaryImage->Allocate(); }
	//catch( itk::ExceptionObject &e) { std::cerr << "Exception raised allocating the single object image: \n"<< e << std::endl; }
	//singleObjectBinaryImage->FillBuffer(0);
	//SceneType::LabelImageType::IndexType sceneIndex;
	//typedef itk::ImageRegionIteratorWithIndex< itk::Image<unsigned char, OutputImageType::ImageDimension> > TmpIteratorType;
	//TmpIteratorType tmpIt( singleObjectBinaryImage, tmpRegion );
	////for (unsigned i=0 ; i<objectIt->second->pixelSet.size() ; i++) {
	//SceneType::PixelSetType *set = objectIt->second->object->GetObjectAsLabelMap()->GetPixelSet();
	//for (SceneType::PixelSetType::iterator it = set->begin() ; it != set->end() ; it++) {		
	//	sceneIndex = m_scene->FlatIndexToIndex( *it );
	//	tmpIndex[0] = sceneIndex[m_dirAxis0] - refIndex[0];	tmpIndex[1] = sceneIndex[m_dirAxis1] - refIndex[1];
	//	tmpIt.SetIndex( tmpIndex );
	//	if (tmpIt.Get()==0) { tmpIt.Set(1); nbPotentialPixels++; }
	//}

	//return static_cast<double>(nbVisiblePixels)/static_cast<double>(nbPotentialPixels);
	//}


private:
	SimpleSEMSensor(const Self&);			//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};



} // namespace psciob

#endif /* IMAGE3D2DSENSOR_H_ */
