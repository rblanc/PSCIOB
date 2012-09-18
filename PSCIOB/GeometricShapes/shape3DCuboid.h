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
 * \file shape3DCuboid.h
 * \author Rémi Blanc 
 * \date 24. August 2011
*/

#ifndef SHAPE3DCUBOID_H_
#define SHAPE3DCUBOID_H_

#include "BinaryShape.h"
#include "vtkCubeSource.h"

namespace psciob {

/** 
 * \class shape3DCuboid
 * \brief shape3DCuboid is a class for generating a cuboid, usually in combination with a PoseTransform
 * cuboid centered at (0,0,0), with X length = 1, Y and Z 
 * 2 parameters = elongation ( = x/y >=1 ), thickness( = x/z >=1 )
 * -> y = x/e ; z = x/t
*/

class shape3DCuboid : public BinaryShape<3> {
public:
	/** Standard class typedefs. */
	typedef shape3DCuboid                 Self;
	typedef BinaryShape                   Superclass;
	typedef itk::SmartPointer<Self>       Pointer;
	typedef itk::SmartPointer<const Self> ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(shape3DCuboid, BinaryShape);
	itkNewMacro(Self);


	/** Give Information about Self */
	std::string GetClassName()		const	{return m_name;}
	void PrintInfo() const { std::cout<<"shape3DCuboid with parameters: "<<m_parameters<<std::endl; }

	/** Get Number of parameters */
	inline unsigned int GetNumberOfParameters() const {return m_nbParams;}

	/** Get Parameters */
	vnl_vector<double> GetDefaultParameters() const;

	/** Check Validity of the parameters, returns false if parameters are not valid */
	inline bool CheckParameters(const vnl_vector<double> &p) const;

	/** No effect */
	bool SetVTKResolution(const vnl_vector<double> &res) { return true; }

	/** Physical bounding box of the object */
	vnl_vector<double> GetPhysicalBoundingBox() {
		if (!m_physicalBBoxUpToDate) {
			double x = 1.0;
			double y = x/m_parameters(0);
			double z = x/m_parameters(1);
			m_physicalBoundingBox(0) = -x/2.0; //xmin
			m_physicalBoundingBox(1) = +x/2.0; //xmax
			m_physicalBoundingBox(2) = -y/2.0; //ymin
			m_physicalBoundingBox(3) = +y/2.0; //ymax
			m_physicalBoundingBox(4) = -z/2.0; //zmin
			m_physicalBoundingBox(5) = +z/2.0; //zmax
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
	shape3DCuboid();
	virtual ~shape3DCuboid() {};

	static const std::string m_name;
	static const unsigned int m_nbParams = 2;

	vtkSmartPointer<vtkCubeSource> m_cubeSource;

	void ComputeObjectCenter() { m_center.fill(0); m_centerFlag = true; }
	//void ComputeObjectInertia();
	//void ComputeObjectInertiaEigenVectors();

private:
	shape3DCuboid(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};


} // namespace psciob

#endif /* shape3DCuboid_H_ */
