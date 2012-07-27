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
 * \file shape3DCylinder.h
 * \author Cédric Chapoullié 
 * \date 13. October 2011
 * \brief shape3DCylinder: 2 parameters: radius of the basis, length of the cylinder
 * TODO: there is a mismatch between the direction of the cylinder in the vtk and image representations (90° rotation)
 *
*/

#ifndef SHAPE3DCYLINDER_H_
#define SHAPE3DCYLINDER_H_

#include "BinaryShape.h"
#include <vtkCylinderSource.h>

namespace psciob {


class shape3DCylinder : public BinaryShape<3> { //: public ParametricShape {
public:
	/** Standard class typedefs. */
	typedef shape3DCylinder               Self;
	typedef BinaryShape                   Superclass;
	typedef itk::SmartPointer<Self>       Pointer;
	typedef itk::SmartPointer<const Self> ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(shape3DCylinder, BinaryShape);
	itkNewMacro(Self);

	/** Give Information about Self */
	std::string GetClassName() const {return m_name;}
	void PrintInfo() const { std::cout<<"shape3DCylinder with parameters: "<<m_parameters<<std::endl; }

	/** Get Number of parameters */
	inline unsigned int GetNumberOfParameters() const {return m_nbParams;}

	/** Get Parameters */
	vnl_vector<double> GetDefaultParameters() const;

	/** Check Validity of the parameters, returns false if parameters are not valid */
	inline bool CheckParameters(const vnl_vector<double> &p) const;

	/** Set the radius of the cylinder */
	void SetRadius(float p) { 
		if (p<0) throw DeformableModelException("Error in shape3DCylinder: trying to set a negative radius!!");
		if ( abs(m_parameters(0)-p)>TINY ) Modified();
		m_parameters(0) = p;
	} 

	/** Set the height of the cylinder */
	void SetHeight(float p) { 
		if (p<0) throw DeformableModelException("Error in shape3DCylinder: trying to set a negative height!!");
		if ( abs(m_parameters(0)-p)>TINY ) Modified();
		m_parameters(1) = p;
	}

	void SetVTKPolyDataResolution(unsigned int res);

	/** Takes a single parameter, corresponding to the angular resolution */
	bool SetVTKResolution(const vnl_vector<double> &res) { 
		if (res.size() != 1) return false;
		SetVTKPolyDataResolution(res(0));
		m_vtkResolution(0) = m_Resolution;
		return true;
	}

	/** Physical bounding box of the object */
	vnl_vector<double> GetPhysicalBoundingBox() {
		if (!m_physicalBBoxUpToDate) {				
			m_physicalBoundingBox(0) = -m_parameters(0); //xmin
			m_physicalBoundingBox(1) = +m_parameters(0); //xmax
			m_physicalBoundingBox(2) = -m_parameters(1)/2.0; //ymin
			m_physicalBoundingBox(3) = +m_parameters(1)/2.0; //ymax
			m_physicalBoundingBox(4) = -m_parameters(0); //zmin
			m_physicalBoundingBox(5) = +m_parameters(0); //zmax
			m_physicalBBoxUpToDate=true;
		}
		return m_physicalBoundingBox;
	}

	/** Get the corresponding representation of the object */
	vtkPolyData* GetObjectAsVTKPolyData();

protected:
	shape3DCylinder();
	virtual ~shape3DCylinder() {};

	static const std::string m_name;
	static const unsigned int m_nbParams = 2;

	unsigned int m_Resolution;
	vtkSmartPointer<vtkCylinderSource> m_cylinderSource;

private:
	shape3DCylinder(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};


} // namespace psciob

#endif /* SHAPE3DCYLINDER_H_ */
