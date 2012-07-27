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
 * \file shape3DCuboid.cpp
 * \author Rémi Blanc 
 * \date 24. August 2011
*/


#include "shape3DCuboid.h"

using namespace psciob;

const std::string shape3DCuboid::m_name = "shape3DCuboid";

shape3DCuboid::shape3DCuboid() : BinaryShape<3>() {
	SetDefaultParameters(); 
	//
	m_cubeSource = vtkSmartPointer<vtkCubeSource>::New();
	m_cubeSource->SetCenter(0,0,0);
	m_cubeSource->SetXLength(m_parameters(0));
	m_cubeSource->SetYLength(m_parameters(1));
	m_cubeSource->SetZLength(m_parameters(2));
}


//default parameters: unit sides
vnl_vector<double> shape3DCuboid::GetDefaultParameters() const {
	vnl_vector<double> p(m_nbParams); p.fill(1);
	return p;
}

//Check Parameters
inline 
bool shape3DCuboid::CheckParameters(const vnl_vector<double> &p) const {
	if (p.size()!=m_nbParams) return false;
	for (unsigned i=0 ; i<m_nbParams ; i++) { if (p(i)<TINY) return false; }
	return true;
}


vtkPolyData* shape3DCuboid::GetObjectAsVTKPolyData() {
	if (!m_uptodatePolyData) {
		m_cubeSource->SetXLength(m_parameters(0));
		m_cubeSource->SetYLength(m_parameters(1));
		m_cubeSource->SetZLength(m_parameters(2));

		m_cubeSource->Update();
		m_outputPolyData = m_cubeSource->GetOutput();

		m_uptodatePolyData = true;
	}
	return m_outputPolyData.GetPointer();
}

//
//void shape3DCuboid::UpdateBinaryImage_Internal() {
//	AllocateITKImageFromPhysicalBoundingBoxAndSpacing<BinaryImageType>( this->GetPhysicalBoundingBox(), this->GetImageSpacing(), m_internalBinaryImage );
//
//	BinaryImageType::PointType pointCoords;
//	vnl_vector<double> d(m_nbDimensions), dmax(m_nbDimensions);
//	for (unsigned i=0 ; i<m_nbDimensions ; i++) dmax(i) = m_parameters(i)/2.0;
//	typedef itk::ImageRegionIteratorWithIndex< BinaryImageType > IteratorType;
//	IteratorType it( m_internalBinaryImage, m_internalBinaryImage->GetLargestPossibleRegion() );
//	it.GoToBegin();
//	BinaryPixelType val;
//	while(!it.IsAtEnd()) {
//		m_internalBinaryImage->TransformIndexToPhysicalPoint(it.GetIndex(), pointCoords);
//		val=1;
//		for (unsigned i=0 ; i<m_nbDimensions ; i++) {
//			if (fabs(pointCoords[i])>dmax(i)) {val=0; break;}
//		}
//		it.Set(val);
//		++it;
//	}
//}
