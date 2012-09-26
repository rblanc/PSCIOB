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
 * \file GenericParametricFunctions.h
 * \author Rémi Blanc 
 * \date 4. October 2011
*/

#ifndef GENERICFUNCTIONS_H_
#define GENERICFUNCTIONS_H_

#include "GeneralUtils.h"
#include <math.h> 

namespace psciob {

//TODO create a .txx file to make this file more readable... + document the classes in here.

/** \class GenericParametricFunctions
* \brief GenericParametricFunctions : base class for generic parametric functions
*/
template<class TInputType, class TOutputType>
class GenericParametricFunctions : public itk::LightObject {
public: 
	typedef GenericParametricFunctions			Self;
	typedef typename itk::LightObject			Superclass;
	typedef typename itk::SmartPointer<Self>	Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(GenericParametricFunctions, itk::LightObject);

	virtual TOutputType Evaluate( TInputType input ) = 0;

	virtual unsigned int GetNumberOfParameters() = 0;

	//TODO: check validity of the parameters ; return false if the parameters are invalid.
	virtual bool SetParameters(vnl_vector<double> p) {
		if (p.size() != GetNumberOfParameters()) return false;
		m_params = p;
		return true;
	}
	vnl_vector<double> GetParameters() { return m_params; }
protected:
	GenericParametricFunctions() {}
	virtual ~GenericParametricFunctions() {};

	vnl_vector<double> m_params; 
private:
	GenericParametricFunctions(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};


/** \class GPF_GreyLevelContrastFunction
* \brief GPF_GreyLevelContrastFunction : not implemented
* intended to be used as a rescaling function for intensities ... (grayscale images)
*/
template<class TInputType, class TOutputType>
class GPF_GreyLevelContrastFunction : public GenericParametricFunctions<TInputType, TOutputType> {
public: 
	static const unsigned int nbParams = 2;
	typedef GPF_GreyLevelContrastFunction			Self;
	typedef GenericParametricFunctions			Superclass;
	typedef itk::SmartPointer<Self>				Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(GPF_GreyLevelContrastFunction, GenericParametricFunctions);
	itkNewMacro(Self);

	unsigned int GetNumberOfParameters() {return nbParams};

	inline double Evaluate( double input ) {
		throw DeformableModelException("GPF_GreyLevelContrastFunction: todo! ");
	}
protected:
	GPF_GreyLevelContrastFunction() : GenericParametricFunctions() {};
	~GPF_GreyLevelContrastFunction() {};
private:
	GPF_GreyLevelContrastFunction(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};

/** \class GPF_IdentityFunction
* \brief GPF_IdentityFunction : just outputs the input (used as default in some classes...)
*/
template<class TInputType = double, class TOutputType = double>
class GPF_IdentityFunction : public GenericParametricFunctions<TInputType, TOutputType> {
public:
	static const unsigned int nbParams = 0;
	typedef GPF_IdentityFunction        Self;
	typedef GenericParametricFunctions  Superclass;
	typedef itk::SmartPointer<Self>     Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(GPF_IdentityFunction, GenericParametricFunctions);
	itkNewMacro(Self);

	unsigned int GetNumberOfParameters() {return nbParams;}
	inline TOutputType Evaluate( TInputType input ) { return static_cast<TOutputType>( input ); }

protected:
	GPF_IdentityFunction() : GenericParametricFunctions() {};
	~GPF_IdentityFunction() {};
private:
	GPF_IdentityFunction(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};

/** \class GPF_SqrtFunction
* \brief GPF_SqrtFunction : just wraps the sqrt function 
* unsure what happens if the input is negative...
*/
template<class TInputType = double, class TOutputType = double>
class GPF_SqrtFunction : public GenericParametricFunctions<TInputType, TOutputType> {
public:
	static const unsigned int nbParams = 0;
	typedef GPF_SqrtFunction        Self;
	typedef GenericParametricFunctions  Superclass;
	typedef itk::SmartPointer<Self>     Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(GPF_SqrtFunction, GenericParametricFunctions);
	itkNewMacro(Self);

	unsigned int GetNumberOfParameters() {return nbParams;}
	inline TOutputType Evaluate( TInputType input ) { 
		return sqrt(input);
		//if (input>=0) return sqrt(input);
		//else throw DeformableModelException("GPF_SqrtFunction: square root of negative number is undefined !");
	}
protected:
	GPF_SqrtFunction() : GenericParametricFunctions() {};
	~GPF_SqrtFunction() {};
private:
	GPF_SqrtFunction(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};

/** \class GPF_ConstantFunction
* \brief GPF_ConstantFunction : outputs a constant of the specified type...
* default is 1.
*/
template<class TInputType = double, class TOutputType = double>
class GPF_ConstantFunction : public GenericParametricFunctions<TInputType, TOutputType> {
public:
	static const unsigned int nbParams = 1;

	typedef GPF_ConstantFunction					Self;
	typedef GenericParametricFunctions		Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(GPF_ConstantFunction, GenericParametricFunctions);
	itkNewMacro(Self);

	unsigned int GetNumberOfParameters() {return nbParams;}

	//overload for easy user access //TODO: check validity of the parameters...
	bool SetParameter(double p) { m_params(0)=p; return true; } 
	inline TOutputType Evaluate( TInputType input ) { return static_cast<TOutputType>( m_params(0) ); }

protected:
	GPF_ConstantFunction() : GenericParametricFunctions() {m_params.set_size(nbParams); m_params(0) = 1;};
	~GPF_ConstantFunction() {};
private:
	GPF_ConstantFunction(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};

/** \class GPF_ExpDecrFunction
* \brief GPF_ExpDecrFunction: p0*exp(-x)
* default: p0 = 1
*/
template<class TInputType, class TOutputType>
class GPF_ExpDecrFunction : public GenericParametricFunctions<TInputType, TOutputType> {
public:
	static const unsigned int nbParams = 1;

	typedef GPF_ExpDecrFunction					Self;
	typedef GenericParametricFunctions		Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(GPF_ExpDecrFunction, GenericParametricFunctions);
	itkNewMacro(Self);


	unsigned int GetNumberOfParameters() {return nbParams;}

	//overload for easy user access //TODO: check validity of the parameters...
	bool SetParameter(double p) { 
		m_params(0)=p; return true;
	} 
	inline TOutputType Evaluate( TInputType input ) { return static_cast<TOutputType>( m_params(0) * exp( - static_cast<double>(input)) ); }

protected:
	GPF_ExpDecrFunction() : GenericParametricFunctions() {m_params.set_size(nbParams); m_params(0) = 1;};
	~GPF_ExpDecrFunction() {};
private:
	GPF_ExpDecrFunction(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};

/** \class GPF_ScExpDecrFunction
* \brief GPF_ScExpDecrFunction: p0*exp(-p1*x)
* default: p0 = 1, p1 = 1
*/
template<class TInputType, class TOutputType>
class GPF_ScExpDecrFunction : public GenericParametricFunctions<TInputType, TOutputType> {
public:
	static const unsigned int nbParams = 2;

	typedef GPF_ScExpDecrFunction      Self;
	typedef GenericParametricFunctions Superclass;
	typedef itk::SmartPointer<Self>    Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(GPF_ScExpDecrFunction, GenericParametricFunctions);
	itkNewMacro(Self);

	unsigned int GetNumberOfParameters() {return nbParams;}

	inline TOutputType Evaluate( TInputType input ) { return static_cast<TOutputType>( m_params(0) * exp( - m_params(1) * static_cast<double>(input)) ); }

protected:
	GPF_ScExpDecrFunction() : GenericParametricFunctions() {m_params.set_size(nbParams); m_params.fill(1);};
	~GPF_ScExpDecrFunction() {};
private:
	GPF_ScExpDecrFunction(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};


/** \class GPF_LinearDecrFunction
* \brief GPF_LinearDecrFunction: p0 - p1*x
* default: p0 = 250, p1 = 1
*/
template<class TInputType, class TOutputType>
class GPF_LinearDecrFunction : public GenericParametricFunctions<TInputType, TOutputType> {
public:
	static const unsigned int nbParams = 2;

	typedef GPF_LinearDecrFunction					Self;
	typedef GenericParametricFunctions		Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(GPF_ExpDecrFunction, GenericParametricFunctions);
	itkNewMacro(Self);


	unsigned int GetNumberOfParameters() {return nbParams;}

	//overload for easy user access //TODO: check validity of the parameters...
	bool SetParameter(double p0, double p1) { 
		m_params(0)=p0; m_params(1)=p1; return true;
	} 
	
	inline TOutputType Evaluate( TInputType input ) { return static_cast<TOutputType>( m_params(0) - m_params(1) * input ); }

protected:
	GPF_LinearDecrFunction() : GenericParametricFunctions() {m_params.set_size(nbParams); m_params(0) = 250; m_params(1) = 1;};
	~GPF_LinearDecrFunction() {};
private:
	GPF_LinearDecrFunction(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};


/** \class GPF_ScaledExpFunction
* \brief GPF_ScaledExpFunction: exp(p0*x)
* default: p0=1
*/
template<class TInputType, class TOutputType>
class GPF_ScaledExpFunction : public GenericParametricFunctions<TInputType, TOutputType> {
public:
	static const unsigned int nbParams = 1;

	typedef GPF_ScaledExpFunction          Self;
	typedef GenericParametricFunctions Superclass;
	typedef itk::SmartPointer<Self>	   Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(GPF_ScaledExpFunction, GenericParametricFunctions);
	itkNewMacro(Self);

	unsigned int GetNumberOfParameters() {return nbParams;}

	//overload for easy user access ; remember to check the validity of the parameters...
	bool SetParameter(double p) { 
		m_params(0)=p; return true;
	}

	inline TOutputType Evaluate( TInputType input ) { return static_cast<TOutputType>( exp( m_params(0)*static_cast<double>(input)) ); }

protected:
	GPF_ScaledExpFunction() : GenericParametricFunctions() {m_params.set_size(nbParams); m_params(0) = 1;};
	~GPF_ScaledExpFunction() {};
private:
	GPF_ScaledExpFunction(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};

/** \class GPF_ScaledOffsetExpFunction
* \brief GPF_ScaledOffsetExpFunction: p0*exp(-(p1-x)/p2)
* default: p0=1, p1=1, p2=1
*/
template<class TInputType, class TOutputType>
class GPF_ScaledOffsetExpFunction : public GenericParametricFunctions<TInputType, TOutputType> {
public:
	static const unsigned int nbParams = 3;

	typedef GPF_ScaledOffsetExpFunction Self;
	typedef GenericParametricFunctions  Superclass;
	typedef itk::SmartPointer<Self>     Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(GPF_ScaledOffsetExpFunction, GenericParametricFunctions);
	itkNewMacro(Self);

	unsigned int GetNumberOfParameters() {return nbParams;}

	//overload for easy user access ; remember to check the validity of the parameters...
	bool SetParameter(double p0, double p1, double p2) { 
		m_params(0)=p0; m_params(1)=p1; m_params(2)=p2; return true;
	}

	inline TOutputType Evaluate( TInputType input ) { return static_cast<TOutputType>( m_params(0)*exp( static_cast<double>(input-m_params(1))/m_params(2)) ); }

protected:
	GPF_ScaledOffsetExpFunction() : GenericParametricFunctions() {m_params.set_size(nbParams); m_params.fill(1);};
	~GPF_ScaledOffsetExpFunction() {};
private:
	GPF_ScaledOffsetExpFunction(const Self&); //purposely not implemented
	const Self & operator=( const Self & );   //purposely not implemented
};

/** \class GPF_ScaledExpPlusConstantFunction
* \brief GPF_ScaledExpPlusConstantFunction: p0+exp(p1*x)
* default: p0=0, p1=1
*/
template<class TInputType, class TOutputType>
class GPF_ScaledExpPlusConstantFunction : public GenericParametricFunctions<TInputType, TOutputType> {
public:
	static const unsigned int nbParams = 2;

	typedef GPF_ScaledExpPlusConstantFunction Self;
	typedef GenericParametricFunctions    Superclass;
	typedef itk::SmartPointer<Self>	      Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(GPF_ScaledExpPlusConstantFunction, GenericParametricFunctions);
	itkNewMacro(Self);

	unsigned int GetNumberOfParameters() {return nbParams;}

	//overload for easy user access ; remember to check the validity of the parameters...
	bool SetParameter(double p0, double p1) { 
		m_params(0)=p0; m_params(1)=p1; return true;
	}

	inline TOutputType Evaluate( TInputType input ) { return static_cast<TOutputType>( m_params(0) + exp( m_params(1)*static_cast<double>(input)) ); }

protected:
	GPF_ScaledExpPlusConstantFunction() : GenericParametricFunctions() {m_params.set_size(nbParams); m_params(0) = 0;m_params(1) = 1;};
	~GPF_ScaledExpPlusConstantFunction() {};
private:
	GPF_ScaledExpPlusConstantFunction(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};


/** \class GPF_ScaledOffsetExpPlusConstantFunction
* \brief GPF_ScaledOffsetExpPlusConstantFunction: p0+exp(p1*(x-p2))
* default: p0=0, p1=1, p2=0
*/
template<class TInputType, class TOutputType>
class GPF_ScaledOffsetExpPlusConstantFunction : public GenericParametricFunctions<TInputType, TOutputType> {
public:
	static const unsigned int nbParams = 3;

	typedef GPF_ScaledOffsetExpPlusConstantFunction Self;
	typedef GenericParametricFunctions    Superclass;
	typedef itk::SmartPointer<Self>	      Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(GPF_ScaledOffsetExpPlusConstantFunction, GenericParametricFunctions);
	itkNewMacro(Self);

	unsigned int GetNumberOfParameters() {return nbParams;}

	//overload for easy user access ; remember to check the validity of the parameters...
	bool SetParameter(double p) { 
		m_params(0)=p0; m_params(1)=p1; m_params(2)=p2; return true;
	}

	inline TOutputType Evaluate( TInputType input ) { return static_cast<TOutputType>( m_params(0) + exp( m_params(1)*static_cast<double>(input-m_params(2))) ); }

protected:
	GPF_ScaledOffsetExpPlusConstantFunction() : GenericParametricFunctions() {m_params.set_size(nbParams); m_params(0) = 0; m_params(1) = 1; m_params(2) = 0;};
	~GPF_ScaledOffsetExpPlusConstantFunction() {};
private:
	GPF_ScaledOffsetExpPlusConstantFunction(const Self&); //purposely not implemented
	const Self & operator=( const Self & );               //purposely not implemented
};


/** \class GPF_PieceWiseLinearExponentialRescaling
* \brief GPF_PieceWiseLinearExponentialRescaling: 
* if in<p0  : output = (in-p0)/p0 
* otherwise: output = exp(-(in-p0)/p1)-1
* default: p0=0, p1=1
* This function is typical defined for pin in [0 ; INF] to [-1 ; 1], increasing linearly in [0 p0] (from -1 to 1), then exponentially from 0 to 1 in [p0 INF], faster when p1 is small
*/
template<class TInputType, class TOutputType>
class GPF_PieceWiseLinearExponentialRescaling : public GenericParametricFunctions<TInputType, TOutputType> {
public:
	static const unsigned int nbParams = 2;

	typedef GPF_PieceWiseLinearExponentialRescaling Self;
	typedef GenericParametricFunctions          Superclass;
	typedef itk::SmartPointer<Self>	            Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(GPF_PieceWiseLinearExponentialRescaling, GenericParametricFunctions);
	itkNewMacro(Self);

	unsigned int GetNumberOfParameters() {return nbParams;}

	inline TOutputType Evaluate( TInputType input ) { 
		if (input<m_params(0)) return ( (input-m_params(0))/m_params(0));
		else return ( 1.0 - exp(-(input-m_params(0))/m_params(1)) );
	}

protected:
	GPF_PieceWiseLinearExponentialRescaling() : GenericParametricFunctions() {m_params.set_size(nbParams); m_params(0) = 0;m_params(1) = 1;};
	~GPF_PieceWiseLinearExponentialRescaling() {};
private:
	GPF_PieceWiseLinearExponentialRescaling(const Self&); //purposely not implemented
	const Self & operator=( const Self & );		      //purposely not implemented
};

/** \class GPF_DecreasingPieceWiseLinearExponentialRescaling
* \brief GPF_DecreasingPieceWiseLinearExponentialRescaling: 
* if in<t  : output = 1-in/p0 
* otherwise: output = exp(-(in-p0)/p1)-1
* default: p0=0, p1=1
* This function is typical defined for pin in [0 ; INF] to [-1 ; 1], decreasing linearly in [0 p0] (from 1 to 0), then exponentially from 0 to -1 in [p0 INF], faster when p1 is large
*/
template<class TInputType, class TOutputType>
class GPF_DecreasingPieceWiseLinearExponentialRescaling : public GenericParametricFunctions<TInputType, TOutputType> {
public:
	static const unsigned int nbParams = 2;

	typedef GPF_DecreasingPieceWiseLinearExponentialRescaling Self;
	typedef GenericParametricFunctions          Superclass;
	typedef itk::SmartPointer<Self>	            Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(GPF_DecreasingPieceWiseLinearExponentialRescaling, GenericParametricFunctions);
	itkNewMacro(Self);

	unsigned int GetNumberOfParameters() {return nbParams;}

	inline TOutputType Evaluate( TInputType input ) { 
		if (input<m_params(0)) return (1.0-(m_params(0)-input)/m_params(0));
		else return ( exp(-(input-m_params(0))/m_params(1)) - 1.0);
	}

protected:
	GPF_DecreasingPieceWiseLinearExponentialRescaling() : GenericParametricFunctions() {m_params.set_size(nbParams); m_params(0) = 0;m_params(1) = 1;};
	~GPF_DecreasingPieceWiseLinearExponentialRescaling() {};
private:
	GPF_DecreasingPieceWiseLinearExponentialRescaling(const Self&); //purposely not implemented
	const Self & operator=( const Self & );		      //purposely not implemented
};

/** \class GPF_LinearRescaling
* \brief GPF_LinearRescaling: 
* output = p0+p1*in
* default: p0=0, p1=1;
*/
template<class TInputType, class TOutputType>
class GPF_LinearRescaling : public GenericParametricFunctions<TInputType, TOutputType> {
public:
	static const unsigned int nbParams = 2;

	typedef GPF_LinearRescaling         Self;
	typedef GenericParametricFunctions  Superclass;
	typedef itk::SmartPointer<Self>	    Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(GPF_LinearRescaling, GenericParametricFunctions);
	itkNewMacro(Self);

	unsigned int GetNumberOfParameters() {return nbParams;}

	inline TOutputType Evaluate( TInputType input ) { return ( m_params(0) + m_params(1)*input ); }

protected:
	GPF_LinearRescaling() : GenericParametricFunctions() {m_params.set_size(nbParams); m_params(0) = 0;m_params(1) = 1;};
	~GPF_LinearRescaling() {};
private:
	GPF_LinearRescaling(const Self&);            //purposely not implemented
	const Self & operator=( const Self & );	 //purposely not implemented
};


/** \class GPF_Offset
* \brief GPF_Offset: 
* output = p0+in
* default: p0=0;
*/
template<class TInputType, class TOutputType>
class GPF_Offset : public GenericParametricFunctions<TInputType, TOutputType> {
public:
	static const unsigned int nbParams = 1;

	typedef GPF_Offset             Self;
	typedef GenericParametricFunctions  Superclass;
	typedef itk::SmartPointer<Self>	    Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(GPF_Offset, GenericParametricFunctions);
	itkNewMacro(Self);

	unsigned int GetNumberOfParameters() {return nbParams;}

	inline TOutputType Evaluate( TInputType input ) { return ( m_param(0) + input ); }

protected:
	GPF_Offset() : GenericParametricFunctions() {m_params.set_size(nbParams); m_params(0) = 0; };
	~GPF_Offset() {};
private:
	GPF_Offset(const Self&);            //purposely not implemented
	const Self & operator=( const Self & );	 //purposely not implemented
};


/** \class GPF_PseudoSigmoid_TruncatedDecreasingExponentials
* \brief GPF_PseudoSigmoid_TruncatedDecreasingExponentials: 
* piecewise, decreasing function, mapping [-INF ; p0] to [1 ; 0.5] and [p0 p1] to [0.5 ; 0]
* each piece is exponential, with inflexion point at p0, and the function can be quite steep at this point
* if in<p0 :     output = 1-0.5*exp( (in-p0)/p2 )
* else if in<p1: output = 0.5*exp( (p1-in)/(p3*(p1-p0)) )/exp(1/p3)
* else if in>p1: output = 0;
* default: p0=0 => 'inflexion point'
*          p1=1 => maximum value of the input before capping.
*          p2 = 1 (smaller means the function is steeper when approaching p0, and quickly reaches 1 with decreasing pin)
*          p3 = 1 (smaller means the function is steeper when approaching p0, and quickly decreases to 0 with increasing pin)
*/
template<class TInputType, class TOutputType>
class GPF_PseudoSigmoid_TruncatedDecreasingExponentials : public GenericParametricFunctions<TInputType, TOutputType> {
public:
	static const unsigned int nbParams = 4;

	typedef GPF_PseudoSigmoid_TruncatedDecreasingExponentials         Self;
	typedef GenericParametricFunctions  Superclass;
	typedef itk::SmartPointer<Self>	    Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(GPF_LinearRescaling, GenericParametricFunctions);
	itkNewMacro(Self);

	unsigned int GetNumberOfParameters() {return nbParams;}

	inline TOutputType Evaluate( TInputType input ) { 
		if (input>=m_params(1)) return 0;
		if (input<m_params(0)) return (1.0-0.5*exp( (input-m_params(0))/m_params(2)));
		return ( exp( (m_params(1)-input)*m_exp_div )-1.0 ) * m_fact ;
	}

	//TODO: check parameters (p2 and p3 are positive, p0 is less than p1...
	bool SetParameters(vnl_vector<double> p) {
		if (p.size() != GetNumberOfParameters()) return false;
		if (p(2)<=0) return false;
		if (p(3)<=0) return false;
		if (p(1)<=p(0)) return false;
		m_params = p;
		m_exp_div = 1.0 / (m_params(3)*(m_params(1) - m_params(0)));
		m_fact = 1.0 / ( 2.0*(exp(1.0/m_params(3))-1.0) );
		return true;
	}

protected:
	GPF_PseudoSigmoid_TruncatedDecreasingExponentials() : GenericParametricFunctions() {
		m_params.set_size(nbParams); m_params(0) = 0;m_params(1) = 1; m_params(2) = 1;m_params(3) = 0.1;
		m_exp_div = 1.0 / (m_params(3)*(m_params(1) - m_params(0)));
		m_fact = 1.0 / ( 2.0*(exp(1.0/m_params(3))-1.0) );
	};
	~GPF_PseudoSigmoid_TruncatedDecreasingExponentials() {};
	double m_exp_div, m_fact;
private:
	GPF_PseudoSigmoid_TruncatedDecreasingExponentials(const Self&);            //purposely not implemented
	const Self & operator=( const Self & );	 //purposely not implemented
};

} // namespace psciob

#endif //GENERICFUNCTIONS_H_