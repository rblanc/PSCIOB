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
 * \file shape3DCylinder.cpp
 * \author Cédric Chapoullié 
 * \date 13. October 2011
*/


#include "shape3DCylinder.h"

using namespace psciob;

const std::string shape3DCylinder::m_name = "shape3DCylinder";

shape3DCylinder::shape3DCylinder() : BinaryShape<3>() {
	SetDefaultParameters();
	//
	m_cylinderSource = vtkSmartPointer<vtkCylinderSource>::New();
	m_Resolution=36;		
	m_cylinderSource->SetResolution(m_Resolution);
	m_cylinderSource->SetCenter(0,0,0);
	m_cylinderSource->SetRadius(m_parameters(0));
	m_cylinderSource->SetHeight(m_parameters(1));

	m_vtkResolution.set_size(1);
	m_vtkResolution(0) = m_Resolution;
}


vnl_vector<double> shape3DCylinder::GetDefaultParameters() const {
	vnl_vector<double> p(m_nbParams); p.fill(1);
	return p;
}

inline 
bool shape3DCylinder::CheckParameters(const vnl_vector<double> &p) const {
	if (p.size()!=m_nbParams) return false;
	for (unsigned i=0 ; i<m_nbParams ; i++) { if (p(i)<TINY) return false; }
	return true;
}


void shape3DCylinder::SetVTKPolyDataResolution(unsigned int res) {
	if (m_Resolution!=res) {Modified();}
	if (res>6)		m_Resolution = res;		else m_Resolution = 6;
	m_vtkResolution(0) = m_Resolution;
	m_cylinderSource->SetResolution(m_Resolution);
}



vtkPolyData* shape3DCylinder::GetObjectAsVTKPolyData() {
	if (!m_uptodatePolyData) {
		m_cylinderSource->SetRadius(m_parameters(0));
		m_cylinderSource->SetHeight(m_parameters(1));

		m_cylinderSource->Update();
		m_outputPolyData = m_cylinderSource->GetOutput();

		m_uptodatePolyData = true;
	}
	return m_outputPolyData.GetPointer();
}


//
//void shape3DCylinder::UpdateBinaryImage_Internal() {
//	AllocateITKImageFromPhysicalBoundingBoxAndSpacing<BinaryImageType>( this->GetPhysicalBoundingBox(), this->GetImageSpacing(), m_internalBinaryImage );
//
//	BinaryImageType::PointType pointCoords;
//	double d2, d2max = m_parameters(0)*m_parameters(0);
//	typedef itk::ImageRegionIteratorWithIndex< BinaryImageType > IteratorType;
//	IteratorType it( m_internalBinaryImage, m_internalBinaryImage->GetLargestPossibleRegion() );
//	it.GoToBegin();
//	while(!it.IsAtEnd()) {
//		m_internalBinaryImage->TransformIndexToPhysicalPoint(it.GetIndex(), pointCoords);
//		d2 = pointCoords[0]*pointCoords[0] + pointCoords[2]*pointCoords[2];
//		if (( d2<=d2max )&&(fabs(pointCoords[1])<(m_parameters(1)/2.0))) it.Set(1); //for a point, all pixels in the pixelBoundingBox should be ON
//		else it.Set(0);
//		++it;
//	}
//}
//
