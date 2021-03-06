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
* \file Fast2DRectangle.cpp
* \author R�mi Blanc 
* \date 1. December 2011
*/

#include "Fast2DRectangle.h"

using namespace psciob;

const std::string Fast2DRectangle::m_name = "Fast2DRectangle";

Fast2DRectangle::Fast2DRectangle() : Binary2DConvexModel() {
	m_outputPolyData = vtkSmartPointer<vtkPolyData>::New();
	SetDefaultParameters();
}


//default parameters
vnl_vector<double> Fast2DRectangle::GetDefaultParameters() const {
	vnl_vector<double> p(m_nbParams); 
	p(0)=0; p(1)=0; //
	p(2)=5; p(3)=5;
	return p;
}

//Check Parameters
inline 
bool Fast2DRectangle::CheckParameters(const vnl_vector<double> &p) const {
	if (p.size()!=m_nbParams) return false;
	for (unsigned i=2 ; i<4 ; i++) { if (p(i)<0) return false; }
	return true;
}



vtkPolyData* Fast2DRectangle::GetObjectAsVTKPolyData() {
	if (!m_uptodatePolyData) {
		m_outputPolyData->Initialize();
		vtkPoints *points = vtkPoints::New();	points->Allocate(4);
		vtkCellArray *cells = vtkCellArray::New();	cells->Allocate( cells->EstimateSize(4,2) ); 

		GetPhysicalBoundingBox();
		//points must be ordered
		points->InsertNextPoint( m_physicalBoundingBox(0) , m_physicalBoundingBox(2), 0 ); 
		points->InsertNextPoint( m_physicalBoundingBox(0) , m_physicalBoundingBox(3), 0 );
		points->InsertNextPoint( m_physicalBoundingBox(1) , m_physicalBoundingBox(3), 0 );
		points->InsertNextPoint( m_physicalBoundingBox(1) , m_physicalBoundingBox(2), 0 );

		vtkIdType segm[2];
		segm[0] = 0; segm[1] = 1; cells->InsertNextCell(2, segm);
		segm[0] = 1; segm[1] = 2; cells->InsertNextCell(2, segm);
		segm[0] = 2; segm[1] = 3; cells->InsertNextCell(2, segm);
		segm[0] = 3; segm[1] = 0; cells->InsertNextCell(2, segm);

		m_outputPolyData->SetPoints(points);
		cells->Squeeze();
		m_outputPolyData->SetLines(cells);

		m_uptodatePolyData = true;
		points->Delete();
		cells->Delete();
	}
	return m_outputPolyData.GetPointer();
}
