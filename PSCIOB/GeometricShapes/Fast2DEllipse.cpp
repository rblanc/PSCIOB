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
* \file Fast2DEllipse.cpp
* \author Rémi Blanc 
* \date 19. December 2011
*/

#include "Fast2DEllipse.h"

using namespace psciob;

const std::string Fast2DEllipse::m_name = "Fast2DEllipse";

Fast2DEllipse::Fast2DEllipse() : Binary2DConvexModel() {
	m_outputPolyData = vtkSmartPointer<vtkPolyData>::New();
	SetDefaultParameters();

	m_vtkResolution.set_size(1);
	m_vtkResolution(0) = 36;
}


//default parameters : disk with unit radius, center at 0
vnl_vector<double> Fast2DEllipse::GetDefaultParameters() const {
	vnl_vector<double> p(m_nbParams); 
	p(0)=0; p(1)=0; //
	p(2)=1; p(3)=1; 
	p(4)=0; 
	return p;
}

//Check Parameters
inline 
bool Fast2DEllipse::CheckParameters(const vnl_vector<double> &p) const {
	if (p.size()!=m_nbParams) return false;
	if (p(2)<TINY) return false; //no negative length
	if ( (p(3)<TINY) || (p(3)>1) ) return false; // elongation must be in [0,1]
	return true;
}


void Fast2DEllipse::SetVTKPolyDataResolution(unsigned int theta) {
	if ( (m_vtkResolution(0)!=theta) ) {Modified();}
	if (theta>3)	m_vtkResolution(0) = theta;	else m_vtkResolution(0) = 3;	 
}


vtkPolyData* Fast2DEllipse::GetObjectAsVTKPolyData() {
	if (!m_uptodatePolyData) {
		unsigned int thetaResolution = m_vtkResolution(0);
		m_outputPolyData->Initialize();
		vtkPoints *points = vtkPoints::New();      points->Allocate(thetaResolution);
		vtkCellArray *cells = vtkCellArray::New(); cells->Allocate( cells->EstimateSize(thetaResolution,2) ); 

		GetPhysicalBoundingBox();
		//points must be ordered
		double theta, longaxis = m_parameters(2) / m_parameters(3);
		double ct = cos(-m_parameters(4)), st = sin(-m_parameters(4)), xx, yy;
		double fact = 2.0*PI/static_cast<double>(m_vtkResolution(0));
		vtkIdType segm[2];

		//1st point
		xx = longaxis;
		yy = 0;
		points->InsertNextPoint( m_parameters(0) + xx*ct - yy*st, m_parameters(1) + xx*st + yy*ct, 0 ); 
		//all points and all segments but the last
		for (unsigned i=1 ; i<thetaResolution ; i++) {
			theta = fact*static_cast<double>(i);//theta = 2.0*i*PI/static_cast<double>(thetaResolution);
			xx = longaxis*cos(theta);
			yy = m_parameters(2)*sin(theta);
			points->InsertNextPoint( m_parameters(0) + xx*ct - yy*st,
				m_parameters(1) + xx*st + yy*ct, 0 ); 
			segm[0] = i-1; 	segm[1] = i; cells->InsertNextCell(2, segm);
		}
		//last segment
		segm[0] = thetaResolution-1; segm[1] = 0; cells->InsertNextCell(2, segm);

		m_outputPolyData->SetPoints(points);
		//cells->Squeeze();
		m_outputPolyData->SetLines(cells);

		m_uptodatePolyData = true;
		points->Delete();
		cells->Delete();
	}
	return m_outputPolyData.GetPointer();
}

