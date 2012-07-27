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
 * \file shape2DEllipse.h
 * \author Rémi Blanc 
 * \date 29. August 2011
 * 
 * \brief shape2DEllipse: 2 parameters = short axis & elongation (elongation = long/small axis >=1)
 * these are the half-length... ; the long axis is horizontal
 *
*/

#ifndef SHAPE2DELLIPSE_H_
#define SHAPE2DELLIPSE_H_

#include "BinaryShape.h"

namespace psciob {

class shape2DEllipse : public BinaryShape<2> {
public:
	/** Standard class typedefs. */
	typedef shape2DEllipse                  Self;
	typedef BinaryShape                     Superclass;
	typedef itk::SmartPointer<Self>         Pointer;
	typedef itk::SmartPointer<const Self>   ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(shape2DEllipse,BinaryShape);
	itkNewMacro(Self);

	/** Give Information about Self */
	std::string GetClassName()		const	{return m_name;}
	void PrintInfo() const { std::cout<<"shape2DEllipse with parameters: "<<m_parameters<<std::endl; }

	/** Get Number of parameters */
	inline unsigned int GetNumberOfParameters() const {return m_nbParams;}

	/** Get Parameters */
	vnl_vector<double> GetDefaultParameters() const;

	/** Check Validity of the parameters, returns false if parameters are not valid */
	inline bool CheckParameters(const vnl_vector<double> &p) const;


	/** Number of vertices of the polygon approximating the shape */
	void SetVTKPolyDataResolution(unsigned int theta);

	/** Number of vertices of the polygon approximating the shape */
	unsigned int GetVTKPolyDataResolution() {return m_thetaResolution;}

	/** Takes a single parameter, corresponding to the angular resolution */
	bool SetVTKResolution(const vnl_vector<double> &res) { 
		if (res.size() != 1) return false;
		SetVTKPolyDataResolution(res(0));
		m_vtkResolution(0) = m_thetaResolution;
		return true;
	}

	/** Gets the object bounding box */
	vnl_vector<double> GetPhysicalBoundingBox() {
		if (!m_physicalBBoxUpToDate) {			
			double longaxis = m_parameters(0) * m_parameters(1);
			m_physicalBoundingBox(0) = -longaxis; //xmin
			m_physicalBoundingBox(1) = +longaxis; //xmax
			m_physicalBoundingBox(2) = -m_parameters(0); //ymin
			m_physicalBoundingBox(3) = +m_parameters(0); //ymax
			m_physicalBBoxUpToDate = true;
		}
		return m_physicalBoundingBox;
	}

	/** Get the corresponding representation of the object */
	vtkPolyData* GetObjectAsVTKPolyData();

protected:
	shape2DEllipse();
	~shape2DEllipse() {}

	static const std::string m_name;
	static const unsigned int m_nbParams = 2;

	unsigned int m_thetaResolution; //nb of points of the polygon approximating the disk

private:
	shape2DEllipse(const Self&);			//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};


} // namespace psciob

#endif /* SHAPE2DELLIPSE_H_ */
