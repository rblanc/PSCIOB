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
	m_cubeSource->SetXLength(1);
	m_cubeSource->SetYLength(1);
	m_cubeSource->SetZLength(1);
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
	for (unsigned i=0 ; i<m_nbParams ; i++) { if (p(i)<1) return false; }
	return true;
}


vtkPolyData* shape3DCuboid::GetObjectAsVTKPolyData() {
	if (!m_uptodatePolyData) {
		double x = 1.0;
		double y = x/m_parameters(0);
		double z = x/m_parameters(1);

		m_cubeSource->SetXLength(x);
		m_cubeSource->SetYLength(y);
		m_cubeSource->SetZLength(z);

		m_cubeSource->Update();
		m_outputPolyData = m_cubeSource->GetOutput();

		m_uptodatePolyData = true;
	}
	return m_outputPolyData.GetPointer();
}
