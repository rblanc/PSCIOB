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
 * \file BinaryShape.h
 * \author Rémi Blanc 
 * \date 25. July 2011
 */

#ifndef BINARYSHAPE_H_
#define BINARYSHAPE_H_

//IDEA: create a wrapper for the Statismo statistical model class ! 


#include "BinaryDeformableModel.h"

namespace psciob {

/**
 * \class BinaryShape
 * \brief BinaryShape: abstract class for a shape 
 * Shapes (all classes finishing with this suffix) are special objects that cannot be moved about
 * The first VDimension parameters do not correspond to translation, they are spatially referenced in space about the origin, and lie in a predefined orientation
 *
 * They are intended as a base class for BinaryPoseTransformedBinaryShape rather than direct instanciation by the user
 *
 * The base output is the vtkPolyData ; the BinaryImage and LabelMap are constructed from it
 *
*/


//ABSTRACT
template < unsigned int VDimension > 
class BinaryShape : public BinaryDeformableModel<VDimension>  {
template<unsigned int D> friend class PoseTransformedBinaryShape;
public:
	/** Standard class typedefs. */
	typedef BinaryShape                   Self;
	typedef BinaryDeformableModel         Superclass;
	typedef itk::SmartPointer<Self>       Pointer;
	typedef itk::SmartPointer<const Self> ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(BinaryShape, BinaryDeformableModel);

	/** Set Parameters 
	* If a single parameters is different, then the whole shape needs to be update
	*/
	bool SetParameters(const vnl_vector<double> &params) {
		if (!CheckParameters(params)) { return false;}
		for (unsigned i=0 ; i<params.size() ; i++) { 
			if ( fabs(m_parameters(i)-params(i))>TINY ) {
				Modified(); 
				m_parameters = params;
				return true;	
			}
		}
		return true;
	}

	/** Set Position - no effect in this class */
	inline bool PositionAt(const vnl_vector<double> &pos) {return true;}
	/** Apply Translation - no effect in this class */
	inline bool Translate(const vnl_vector<double> &translation) {return true;}
	/** Apply an integer-grid translation - no effect in this class */
	inline bool IntegerGridTranslate(const vnl_vector<int> &translation) {return true;}

protected:
	BinaryShape() : BinaryDeformableModel() {};
	virtual ~BinaryShape() {};

private:
	BinaryShape(const Self&);			//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};




} // namespace psciob

#endif /* BINARYSHAPE_H_ */
