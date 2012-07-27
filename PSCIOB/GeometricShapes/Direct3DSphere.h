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
* \file Direct3DSphere.h
* \author Remi Blanc 
* \date 25. July 2011
* 
*/

#ifndef DIRECT3DSPHERE_H_
#define DIRECT3DSPHERE_H_

#include "BinaryDeformableModel.h"
#include <vtkSphereSource.h>

namespace psciob {

/** 
 * \class Direct3DSphere
 * \brief Direct3DSphere is a class for generating directly a sphere representation.
 * 4 parameters: 3 centers, and radius
 */

//CONCRETE CLASS
class Direct3DSphere : public BinaryDeformableModel<3> {
public:
	/** Standard class typedefs. */
	typedef Direct3DSphere                Self;
	typedef BinaryDeformableModel         Superclass;
	typedef itk::SmartPointer<Self>       Pointer;
	typedef itk::SmartPointer<const Self> ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(Direct3DSphere, BinaryDeformableModel);
	itkNewMacro(Self);

	/** Give Information about Self */
	std::string GetClassName()		const	{return m_name;}
	void PrintInfo() const { std::cout<<"Direct3DSphere with parameters: "<<m_parameters<<std::endl; }

	/** Get Number of parameters */
	inline unsigned int GetNumberOfParameters() const {return m_nbParams;}

	/** Get Default Parameters */
	vnl_vector<double> GetDefaultParameters() const;

	/** Check Validity of the parameters, returns false if parameters are not valid */
	inline bool CheckParameters(const vnl_vector<double> &p) const;

	/** Set the sphere radius */
	void SetRadius(float p) { 
		if (p<0) throw DeformableModelException("Error in shape2DDisk: trying to set a negative radius!!");
		if ( abs(m_parameters(0)-p)>TINY ) Modified();
		m_parameters(0) = p;
	} 

	/** Number of vertices of the polygon approximating the shape */
	void SetVTKPolyDataResolution(unsigned int theta);

	/** Takes a single parameter, corresponding to the angular resolution */
	bool SetVTKResolution(const vnl_vector<double> &res) { 
		if (res.size() != 2) return false;
		if (res(0)<TINY) return false;
		if (res(1)<TINY) return false;
		m_vtkResolution = res;
		m_sphereSource->SetPhiResolution(m_vtkResolution(0));
		m_sphereSource->SetThetaResolution(m_vtkResolution(1));
		return true;
	}

	/** Gets the object bounding box */
	vnl_vector<double> GetPhysicalBoundingBox() {
		if (!m_physicalBBoxUpToDate) {
			m_physicalBoundingBox(0) = m_parameters(0) - m_parameters(3); //xmin
			m_physicalBoundingBox(1) = m_parameters(0) + m_parameters(3); //xmax
			m_physicalBoundingBox(2) = m_parameters(1) - m_parameters(3); //ymin
			m_physicalBoundingBox(3) = m_parameters(1) + m_parameters(3); //ymax
			m_physicalBoundingBox(4) = m_parameters(2) - m_parameters(3); //zmin
			m_physicalBoundingBox(5) = m_parameters(2) + m_parameters(3); //zmax
			m_physicalBBoxUpToDate = true;
		}
		return m_physicalBoundingBox;
	}

	/** Get the corresponding representation of the object */
	vtkPolyData* GetObjectAsVTKPolyData();
	
	/** Get the binary image representing the object */
	BinaryImageType* GetObjectAsBinaryImage();

	/** Get the sphere as a pixel set */
	LabelMapType* GetObjectAsLabelMap();

protected:
	Direct3DSphere();
	virtual ~Direct3DSphere() {};

	static const std::string m_name;
	static const unsigned int m_nbParams = 4;

	vtkSmartPointer<vtkSphereSource> m_sphereSource;

private:
	Direct3DSphere(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};

} // namespace psciob

#endif /* DIRECT3DSPHERE_H_ */



