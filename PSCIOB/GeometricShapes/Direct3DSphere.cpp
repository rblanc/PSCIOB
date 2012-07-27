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
* \file Direct3DSphere.cpp
* \author Rémi Blanc 
* \date 1. December 2011
*/

#include "Direct3DSphere.h"
#include <itkLabelImageToLabelMapFilter.h>

using namespace psciob;

const std::string Direct3DSphere::m_name = "Direct3DSphere";

Direct3DSphere::Direct3DSphere() : BinaryDeformableModel() {
	SetDefaultParameters();
	//
	m_sphereSource = vtkSmartPointer<vtkSphereSource>::New();
	m_sphereSource->SetCenter(m_parameters(0), m_parameters(1), m_parameters(2));
	m_sphereSource->SetRadius(m_parameters(3));

	m_vtkResolution.set_size(2); m_vtkResolution(0) = 16; m_vtkResolution(1) = 16;
	m_sphereSource->SetPhiResolution(m_vtkResolution(0));
	m_sphereSource->SetThetaResolution(m_vtkResolution(1));
}

//default parameters : sphere with unit radius, center at 0
vnl_vector<double> Direct3DSphere::GetDefaultParameters() const {
	vnl_vector<double> p(m_nbParams); 
	p(0)=0; p(1)=0; p(2)=0; //centered at 0
	p(3)=1; 
	return p;
}

//Check Parameters
inline 
bool Direct3DSphere::CheckParameters(const vnl_vector<double> &p) const {
	if (p.size()!=m_nbParams) return false;
	if (p(3)<TINY) return false; //radius must be positive
	return true;
}


vtkPolyData* Direct3DSphere::GetObjectAsVTKPolyData() {
	if (!m_uptodatePolyData) {
		m_sphereSource->SetCenter(m_parameters(0), m_parameters(1), m_parameters(2));
		m_sphereSource->SetRadius(m_parameters(3));
		m_sphereSource->Update();
		m_outputPolyData = m_sphereSource->GetOutput();
		m_uptodatePolyData = true;
	}
	return m_outputPolyData.GetPointer();
}

Direct3DSphere::BinaryImageType* Direct3DSphere::GetObjectAsBinaryImage() {
	if (!m_uptodateBinaryImage) {
		if (!m_uptodateLabelMap) {//update them both
			if (!m_outputBinaryImage) m_outputBinaryImage = BinaryImageType::New();
			if(!m_outputLabelMap) m_outputLabelMap = LabelMapType::New();

			GetImageBoundingBox();
			m_outputBinaryImage->SetOrigin(m_imageOrigin);   m_outputLabelMap->SetOrigin(m_imageOrigin);
			m_outputBinaryImage->SetRegions(m_imageRegion);  m_outputLabelMap->SetRegions(m_imageRegion);
			m_outputBinaryImage->SetSpacing(m_imageSpacing); m_outputLabelMap->SetSpacing(m_imageSpacing);
			m_outputBinaryImage->Allocate();                 m_outputLabelMap->ClearLabels();

			BinaryImageType::PixelType *buffer = m_outputBinaryImage->GetBufferPointer();
			LabelObjectType::Pointer labelObject = LabelObjectType::New(); labelObject->SetLabel(1);		
			m_outputLabelMap->AddLabelObject(labelObject);

			LabelObjectType::IndexType lineIndex; LabelObjectType::LengthType lineLength;
			bool inRun = false;

			double d2max = m_parameters(3)*m_parameters(3);
			double dx, dy, dz, dx2, dy2, dz2, tmp;
			//browse the image ; at each point, compute the distance to the center
			BinaryImageType::IndexType index;
			BinaryImageType::PointType ref;
			ref[0] = m_imageOrigin[0]-m_parameters(0);
			ref[1] = m_imageOrigin[1]-m_parameters(1);
			ref[2] = m_imageOrigin[2]-m_parameters(2);
			unsigned long add = 0;
			for (index[2] = 0, dz = ref[2] ; index[2]<m_imageRegion.GetSize(2) ; index[2]++, dz+=m_imageSpacing[2]) {
				if (inRun) { inRun = false; labelObject->AddLine(lineIndex, lineLength); } //finalize the run
				dz2 = dz*dz;
				for (index[1] = 0, dy = ref[1] ; index[1]<m_imageRegion.GetSize(1) ; index[1]++, dy+=m_imageSpacing[1]) {
					if (inRun) { inRun = false; labelObject->AddLine(lineIndex, lineLength); } //finalize the run
					dy2 = dy*dy; tmp = dz2+dy2;
					for (index[0] = 0, dx = ref[0] ; index[0]<m_imageRegion.GetSize(0) ; index[0]++, dx+=m_imageSpacing[0]) {
						dx2 = dx*dx;
						if (dx2+tmp<d2max) {
							buffer[add]=1; //1 inside
							if (!inRun) { inRun = true; lineIndex = index; lineLength = 0; } //start a run if necessary
							lineLength++; //elongate the current run.
						}
						else {
							buffer[add]=0; //0 outside
							if (inRun) { inRun = false; labelObject->AddLine(lineIndex, lineLength); } //finalize the run
						}
						add++;
					}
				}
			}
			m_uptodateBinaryImage=true;
			m_uptodateLabelMap=true;
		}
		else { //update only the image
			if (!m_outputBinaryImage) m_outputBinaryImage = BinaryImageType::New();
			GetImageBoundingBox();
			m_outputBinaryImage->SetOrigin(m_imageOrigin);
			m_outputBinaryImage->SetRegions(m_imageRegion);
			m_outputBinaryImage->SetSpacing(m_imageSpacing);
			m_outputBinaryImage->Allocate();

			BinaryImageType::PixelType *buffer = m_outputBinaryImage->GetBufferPointer();
			double d2max = m_parameters(3)*m_parameters(3);
			double dx, dy, dz, dx2, dy2, dz2, tmp;
			//browse the image ; at each point, compute the distance to the center
			BinaryImageType::IndexType index;
			BinaryImageType::PointType ref;
			ref[0] = m_imageOrigin[0]-m_parameters(0);
			ref[1] = m_imageOrigin[1]-m_parameters(1);
			ref[2] = m_imageOrigin[2]-m_parameters(2);
			unsigned long add = 0;
			for (index[2] = 0, dz = ref[2] ; index[2]<m_imageRegion.GetSize(2) ; index[2]++, dz+=m_imageSpacing[2]) {
				dz2 = dz*dz;
				for (index[1] = 0, dy = ref[1] ; index[1]<m_imageRegion.GetSize(1) ; index[1]++, dy+=m_imageSpacing[1]) {
					dy2 = dy*dy; tmp = dz2+dy2;
					for (index[0] = 0, dx = ref[0] ; index[0]<m_imageRegion.GetSize(0) ; index[0]++, dx+=m_imageSpacing[0]) {
						dx2 = dx*dx;
						if (dx2+tmp<d2max) buffer[add]=1; //1 inside
						else               buffer[add]=0; //0 outside
						add++;
					}
				}
			}
			m_uptodateBinaryImage=true;
		}
	}
	return m_outputBinaryImage.GetPointer();
}

Direct3DSphere::LabelMapType* Direct3DSphere::GetObjectAsLabelMap() {
	if (!m_uptodateLabelMap) { 
		if(!m_outputLabelMap) m_outputLabelMap = LabelMapType::New();
		GetImageBoundingBox();
		m_outputLabelMap->SetOrigin(m_imageOrigin);
		m_outputLabelMap->SetRegions(m_imageRegion);
		m_outputLabelMap->SetSpacing(m_imageSpacing);
		m_outputLabelMap->ClearLabels();

		LabelObjectType::Pointer labelObject = LabelObjectType::New(); labelObject->SetLabel(1);		
		m_outputLabelMap->AddLabelObject(labelObject);

		LabelObjectType::IndexType lineIndex; LabelObjectType::LengthType lineLength;
		bool inRun = false;

		double d2max = m_parameters(3)*m_parameters(3);
		double dx, dy, dz, dx2, dy2, dz2, tmp;
		//browse the image ; at each point, compute the distance to the center
		BinaryImageType::IndexType index;
		BinaryImageType::PointType ref;
		ref[0] = m_imageOrigin[0]-m_parameters(0);
		ref[1] = m_imageOrigin[1]-m_parameters(1);
		ref[2] = m_imageOrigin[2]-m_parameters(2);

		//could be slightly optimized again, by reformatting the loops on y and x, selecting only indices of interest.
		for (index[2] = 0, dz = ref[2] ; index[2]<m_imageRegion.GetSize(2) ; index[2]++, dz+=m_imageSpacing[2]) {
			if (inRun) { inRun = false; labelObject->AddLine(lineIndex, lineLength); } //finalize the run
			dz2 = dz*dz;
			for (index[1] = 0, dy = ref[1] ; index[1]<m_imageRegion.GetSize(1) ; index[1]++, dy+=m_imageSpacing[1]) {
				if (inRun) { inRun = false; labelObject->AddLine(lineIndex, lineLength); } //finalize the run
				dy2 = dy*dy; tmp = dz2+dy2;
				if (tmp<=d2max) {
					for (index[0] = 0, dx = ref[0] ; index[0]<m_imageRegion.GetSize(0) ; index[0]++, dx+=m_imageSpacing[0]) {
						dx2 = dx*dx;
						if (dx2+tmp<d2max) {
							if (!inRun) { inRun = true; lineIndex = index; lineLength = 0; } //start a run if necessary
							lineLength++; //elongate the current run.
						}
						else {
							if (inRun) { inRun = false; labelObject->AddLine(lineIndex, lineLength); } //finalize the run
						}
					}
				}
			}
		}
		m_uptodateLabelMap=true;		

	}
	return m_outputLabelMap.GetPointer();
}
