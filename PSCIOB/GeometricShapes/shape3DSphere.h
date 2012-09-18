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
 * \file shape3DSphere.h
 * \author Rémi Blanc 
 * \date 25. July 2011
 * \brief shape3DSphere: 1 parameter = radius >0
*/

#ifndef SHAPE3DSPHERE_H_
#define SHAPE3DSPHERE_H_

#include "BinaryShape.h"
#include <vtkSphereSource.h>

namespace psciob {

/** 
 * \class shape3DSphere
 * \brief shape3DSphere is a class for generating a sphere, usually in combination with a PoseTransform
 * unit sphere centered at (0,0,0) ; 0 parameters
*/

class shape3DSphere : public BinaryShape<3> { //: public ParametricShape {
public:
	/** Standard class typedefs. */
	typedef shape3DSphere                 Self;
	typedef BinaryShape                   Superclass;
	typedef itk::SmartPointer<Self>       Pointer;
	typedef itk::SmartPointer<const Self> ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(shape3DSphere, BinaryShape);
	itkNewMacro(Self);


	/** Give Information about Self */
	std::string GetClassName()		const	{return m_name;}
	void PrintInfo() const { std::cout<<"shape3DSphere "<<std::endl; }

	/** Get Number of parameters */
	inline unsigned int GetNumberOfParameters() const {return m_nbParams;}

	/** Get Parameters */
	vnl_vector<double> GetDefaultParameters() const;

	/** Check Validity of the parameters, returns false if parameters are not valid */
	inline bool CheckParameters(const vnl_vector<double> &p) const;

	/** number of division in latitude and longitude */
	void SetVTKPolyDataResolution(unsigned int phi, unsigned int theta);

	/** Takes two parameters, corresponding to the angular resolution (phi, theta) 
	 * phi corresponds to the number of latitude points, theta is the number of points in longitude
	 */
	bool SetVTKResolution(const vnl_vector<double> &res) { 
		if (res.size() != 2) return false;
		SetVTKPolyDataResolution(res(0), res(1));
		m_vtkResolution(0) = m_thetaResolution;
		return true;
	}

	/** Physical bounding box of the object */
	vnl_vector<double> GetPhysicalBoundingBox() {
		if (!m_physicalBBoxUpToDate) {				
			m_physicalBoundingBox(0) = -0.5; //xmin
			m_physicalBoundingBox(1) = +0.5; //xmax
			m_physicalBoundingBox(2) = -0.5; //ymin
			m_physicalBoundingBox(3) = +0.5; //ymax
			m_physicalBoundingBox(4) = -0.5; //zmin
			m_physicalBoundingBox(5) = +0.5; //zmax
			m_physicalBBoxUpToDate=true;
		}
		return m_physicalBoundingBox;
	}

	/** Get the corresponding representation of the object */
	vtkPolyData* GetObjectAsVTKPolyData();

	//
	void ApplyScalingToParameters(double scaleFactor, vnl_vector<double> &params) {}
	void ApplyRotationToParameters(vnl_matrix<double> rot, vnl_vector<double> &params) {}

protected:
	shape3DSphere();
	virtual ~shape3DSphere() {};

	static const std::string m_name;
	static const unsigned int m_nbParams = 0;

	unsigned int m_phiResolution, m_thetaResolution;
	vtkSmartPointer<vtkSphereSource> m_sphereSource;

	void ComputeObjectCenter() { m_center.fill(0); m_centerFlag = true; }
	//void ComputeObjectInertia();
	//void ComputeObjectInertiaEigenVectors();

private:
	shape3DSphere(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};


} // namespace psciob

#endif /* SHAPE3DSPHERE_H_ */
