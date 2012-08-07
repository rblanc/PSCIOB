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

/*
 * \file shape3DPoint.cpp
 * \author Rémi Blanc 
 * \date 25. July 2011
*/


#include "shape3DPoint.h"

using namespace psciob;

const std::string shape3DPoint::m_name = "shape3DPoint";

shape3DPoint::shape3DPoint() : BinaryShape<3>() {
	m_outputPolyData = vtkSmartPointer<vtkPolyData>::New();
	m_point = vtkSmartPointer<vtkPoints>::New();
	m_vertex = vtkSmartPointer<vtkCellArray>::New();
}



vtkPolyData* shape3DPoint::GetObjectAsVTKPolyData() {
	if (!m_uptodatePolyData) {
		vtkIdType pid[1];
		pid[0] = m_point->InsertNextPoint(0,0,0);
		m_vertex->InsertNextCell(1, pid);

		m_outputPolyData->SetPoints(m_point);
		m_outputPolyData->SetVerts(m_vertex);

		m_uptodatePolyData = true;
	}
	return m_outputPolyData.GetPointer();
}
