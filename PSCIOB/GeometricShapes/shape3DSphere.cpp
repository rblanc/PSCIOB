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
 * \file shape3DSphere.cpp
 * \author Rémi Blanc 25. July 2011
 * \date 25. July 2011
*/


#include "shape3DSphere.h"

using namespace psciob;

const std::string shape3DSphere::m_name = "shape3DSphere";

shape3DSphere::shape3DSphere() : BinaryShape<3>() {
	SetDefaultParameters();
	//
	m_sphereSource = vtkSmartPointer<vtkSphereSource>::New();
	m_phiResolution=16;		m_sphereSource->SetPhiResolution(m_phiResolution);
	m_thetaResolution=16;	m_sphereSource->SetThetaResolution(m_thetaResolution);
	m_sphereSource->SetCenter(0,0,0);
	m_sphereSource->SetRadius(0.5);

	m_vtkResolution.set_size(2);
	m_vtkResolution(0) = m_phiResolution;
	m_vtkResolution(1) = m_thetaResolution;
}


vnl_vector<double> shape3DSphere::GetDefaultParameters() const {
	vnl_vector<double> p(m_nbParams); p.fill(1);
	return p;
}

inline 
bool shape3DSphere::CheckParameters(const vnl_vector<double> &p) const {
	return true;
}


void shape3DSphere::SetVTKPolyDataResolution(unsigned int phi, unsigned int theta) {
	if ( (m_phiResolution!=phi) || (m_thetaResolution!=theta) ) {Modified();}
	if (phi>3)		m_phiResolution = phi;		else m_phiResolution = 3;
	if (theta>3)	m_thetaResolution = theta;	else m_thetaResolution = 3;
	m_vtkResolution(0) = m_phiResolution;
	m_vtkResolution(1) = m_thetaResolution;
	m_sphereSource->SetPhiResolution(m_phiResolution);
	m_sphereSource->SetThetaResolution(m_thetaResolution);
}



vtkPolyData* shape3DSphere::GetObjectAsVTKPolyData() {
	if (!m_uptodatePolyData) {
		m_sphereSource->SetRadius(0.5);

		m_sphereSource->Update();
		m_outputPolyData = m_sphereSource->GetOutput();

		m_uptodatePolyData = true;
	}
	return m_outputPolyData.GetPointer();
}
