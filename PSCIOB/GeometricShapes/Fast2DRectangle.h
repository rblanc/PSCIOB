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
* \file Fast2DRectangle.h
* \author Rémi Blanc 
* \date 1. December 2011
* 
*/

#ifndef FAST2DRECTANGLE_H_
#define FAST2DRECTANGLE_H_

#include "Binary2DConvexModel.h"

namespace psciob {

/** 
* \class Fast2DRectangle
* \brief Fast2DRectangle concrete class for a rectangular shape
*
* parameters: 2 center, 1/2 width, 1/2 height
* 
*/


//CONCRETE CLASS
class Fast2DRectangle : public Binary2DConvexModel {
public:
	/** Standard class typedefs. */
	typedef Fast2DRectangle               Self;
	typedef Binary2DConvexModel           Superclass;
	typedef itk::SmartPointer<Self>       Pointer;
	typedef itk::SmartPointer<const Self> ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(Fast2DRectangle,Binary2DConvexModel);
	itkNewMacro(Self);


	/** Give Information about Self */
	std::string GetClassName()		const	{return m_name;}
	void PrintInfo() const { std::cout<<"Fast2DRectangle with parameters: "<<m_parameters<<std::endl; }

	/** Get Number of parameters */
	inline unsigned int GetNumberOfParameters() const {return m_nbParams;}

	/** Check the validity of the parameters (e.g. non negative length) */
	inline bool CheckParameters(const vnl_vector<double> &p) const;

	/** Get Default Parameters */
	vnl_vector<double> GetDefaultParameters() const;

	/** No effect */
	bool SetVTKResolution(const vnl_vector<double> &res) { return true; }

	/** Physical bounding box of the object */
	const vnl_vector<double>& GetPhysicalBoundingBox() {
		if (!m_physicalBBoxUpToDate) {	
			m_physicalBoundingBox(0) = m_parameters(0) - m_parameters(2); //xmin
			m_physicalBoundingBox(1) = m_parameters(0) + m_parameters(2); //xmax
			m_physicalBoundingBox(2) = m_parameters(1) - m_parameters(3); //ymin
			m_physicalBoundingBox(3) = m_parameters(1) + m_parameters(3); //ymax
			m_physicalBBoxUpToDate = true;
		}
		return m_physicalBoundingBox;
	}

	/** Get the corresponding representation of the object */
	vtkPolyData* GetObjectAsVTKPolyData();
	
	/** \param scaling to apply 
	* \param params is a vector of object parameters 
	* The function modifies these input parameters such that the new parameters correspond to the scaled object
	* \warning: no check are perform to verify the validity of the inputs
	*/
	void ApplyScalingToParameters(double scaleFactor, vnl_vector<double> &params) {	
		params(2)*=scaleFactor;	params(3)*=scaleFactor;	
	}
	
	/** \param rotation matrix to apply (pre-compose: rotate the object around its center)
	* \param params is a vector of object parameters 
	* The function modifies these input parameters such that the new parameters correspond to the rotated object
	* \warning: no check are perform to verify the validity of the inputs
	*/
	void ApplyRotationToParameters(vnl_matrix<double> rot, vnl_vector<double> &params) {}

protected:
	Fast2DRectangle();
	virtual ~Fast2DRectangle() {};

	static const std::string m_name;
	static const unsigned int m_nbParams = 4; 

private:
	Fast2DRectangle(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};

} // namespace psciob

#endif /* FAST2DRECTANGLE_H_ */