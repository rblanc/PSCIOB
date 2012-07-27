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
 * \file shape2DEllipse.cpp
 * \author Rémi Blanc 
 * \date 29. August 2011
*/


#include "shape2DEllipse.h"

using namespace psciob;

const std::string shape2DEllipse::m_name = "shape2DEllipse";

shape2DEllipse::shape2DEllipse() : BinaryShape<2>() {
	m_outputPolyData = vtkSmartPointer<vtkPolyData>::New();
	SetDefaultParameters();
	//
	m_thetaResolution = 36;
	m_vtkResolution.set_size(1);
	m_vtkResolution(0) = m_thetaResolution;
}


//default ellipse parameters: disk with unit radius
vnl_vector<double> shape2DEllipse::GetDefaultParameters() const {
	vnl_vector<double> p(m_nbParams); p.fill(1);
	return p;
}

//Check Parameters
inline 
bool shape2DEllipse::CheckParameters(const vnl_vector<double> &p) const {
	if (p.size()!=m_nbParams) return false;
	if (p(0)<TINY) return false;//long length must be positive
	if (p(1)<1) return false;	//elongation must be >=1
	return true;
}

//
void shape2DEllipse::SetVTKPolyDataResolution(unsigned int theta) {
	if ( (m_thetaResolution!=theta) ) {Modified();}
	if (theta>3)	m_thetaResolution = theta;	else m_thetaResolution = 3;
	m_vtkResolution(0) = m_thetaResolution;
}

//
vtkPolyData* shape2DEllipse::GetObjectAsVTKPolyData() {
	if (!m_uptodatePolyData) {
		m_thetaResolution = m_vtkResolution(0);
		m_outputPolyData->Initialize();
		vtkPoints *points = vtkPoints::New();	points->Allocate(m_thetaResolution);
		vtkCellArray *cells = vtkCellArray::New();	cells->Allocate( cells->EstimateSize(m_thetaResolution,2) ); 

		GetPhysicalBoundingBox();
		//points must be ordered
		double theta, longaxis = m_parameters(0) * m_parameters(1);
		vtkIdType segm[2];

		//1st point
		points->InsertNextPoint( longaxis, 0, 0 );
		//all points and all segments but the last
		for (unsigned i=1 ; i<m_thetaResolution ; i++) {
			theta = 2.0*i*PI/((double)m_thetaResolution);
			points->InsertNextPoint( longaxis*cos(theta), m_parameters(0)*sin(theta), 0 ); 
			segm[0] = i-1; 	segm[1] = i; cells->InsertNextCell(2, segm);
		}
		//last segment
		segm[0] = m_thetaResolution-1; segm[1] = 0; cells->InsertNextCell(2, segm);

		m_outputPolyData->SetPoints(points);
		cells->Squeeze();
		m_outputPolyData->SetLines(cells);

		m_uptodatePolyData = true;
		points->Delete();
		cells->Delete();
	}
	return m_outputPolyData;
}


////
//void shape2DEllipse::UpdateBinaryImage_Internal() {
//	AllocateITKImageFromPhysicalBoundingBoxAndSpacing<BinaryImageType>( this->GetPhysicalBoundingBox(), this->GetImageSpacing(), m_internalBinaryImage );
//
//	BinaryImageType::PointType pointCoords;
//	double d2, b2 = m_parameters(0)*m_parameters(0), a2 = b2*m_parameters(1)*m_parameters(1);
//	typedef itk::ImageRegionIteratorWithIndex< BinaryImageType > IteratorType;
//	IteratorType it( m_internalBinaryImage, m_internalBinaryImage->GetLargestPossibleRegion() );
//	it.GoToBegin();
//	while(!it.IsAtEnd()) {
//		m_internalBinaryImage->TransformIndexToPhysicalPoint(it.GetIndex(), pointCoords);
//		d2 = pointCoords[0]*pointCoords[0]/a2 + pointCoords[1]*pointCoords[1]/b2;
//		if ( d2<=1 ) it.Set(1); //for a point, all pixels in the pixelBoundingBox should be ON
//		else it.Set(0);
//		++it;
//	}
//}
