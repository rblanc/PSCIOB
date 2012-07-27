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
 * \file UnivariatePDF.h
 * \author Rémi Blanc 
 * \date 26. September 2011
*/

#ifndef UNIVARIATEPDF_H_
#define UNIVARIATEPDF_H_

#include <PDFBase.h>
#include <math.h> 

namespace psciob {


/** \class IntegerUnivariatePDF
* \brief IntegerUnivariatePDF base class for integer univariate PDF
*/
class IntegerUnivariatePDF : public PDFBase<long> {
public:
	/** Standard class typedefs. */
	typedef IntegerUnivariatePDF					Self;
	typedef PDFBase<long>				Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(IntegerUnivariatePDF, PDFBase);

	unsigned int GetNumberOfDimensions();
protected:
	IntegerUnivariatePDF();
	virtual ~IntegerUnivariatePDF();
	static const unsigned int m_dimension=1;
};

/** \class UniformIntegerUnivariatePDF
* \brief UniformIntegerUnivariatePDF uniform pdf for integer variables in a specified range
* the range is inclusive x is in [min max] ; default is [0 1]
* p(x) = 1/(max-min+1)
*/
class UniformIntegerUnivariatePDF : public IntegerUnivariatePDF {
public:
	/** Standard class typedefs. */
	typedef UniformIntegerUnivariatePDF		Self;
	typedef IntegerUnivariatePDF			Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(UniformIntegerUnivariatePDF,IntegerUnivariatePDF);
	itkNewMacro(Self);

	/** Set minimum and maximum values */
	void SetParameters(long min = 0.0, long max = 1.0);
	/** p(x) = 1/(max-min+1) */
	double GetLikelihood(const long &x);
	double GetMaximumLikelihoodValue();
	/** p(x) = 1/(max-min+1) */
	double GetLogLikelihood(const long &x);
protected:
	UniformIntegerUnivariatePDF();
	~UniformIntegerUnivariatePDF();

	long m_min, m_max, m_diff;
	long _DrawNewSample();
private:
	UniformIntegerUnivariatePDF(const Self&);//purposely not implemented
	const Self & operator=( const Self & );	 //purposely not implemented
};

/**\class PoissonPDF
* \brief PoissonPDF Poisson distribution using Knuth algorithm
* Integer random variable following a Poisson distribution with given parameter - default is 1
*/
class PoissonPDF : public IntegerUnivariatePDF {
public:
	/** Standard class typedefs. */
	typedef PoissonPDF					Self;
	typedef IntegerUnivariatePDF		Superclass;
	typedef itk::SmartPointer<Self>		Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(PoissonPDF,IntegerUnivariatePDF);
	itkNewMacro(Self);

	void SetLambda(long lambda = 1);

	/** more stable estimation than the direct one... from http://en.wikipedia.org/wiki/Poisson_distribution */
	double GetLikelihood(const long &x);
	double GetMaximumLikelihoodValue();
	double GetLogLikelihood(const long &x);
protected:
	PoissonPDF();
	~PoissonPDF();

	double m_lambda;
	/** using Knuth method http://en.wikipedia.org/wiki/Poisson_distribution */
	long _DrawNewSample();
private: 
	PoissonPDF(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};


//
//
//

/** \class UnivariatePDF
* \brief UnivariatePDF Base class for continuous univariate PDF
*/
class UnivariatePDF : public PDFBase<double> {
public:
	/** Standard class typedefs. */
	typedef UnivariatePDF					Self;
	typedef PDFBase<double>				Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(UnivariatePDF,PDFBase);

	unsigned int GetNumberOfDimensions();
protected:
	UnivariatePDF();
	virtual ~UnivariatePDF() {};
	static const unsigned int m_dimension=1;
};



/** \class DiracPDF
* \brief DiracPDF Dirac distribution, default position is 0
*/
class DiracPDF : public UnivariatePDF {
public:
	/** Standard class typedefs. */
	typedef DiracPDF						Self;
	typedef UnivariatePDF					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(DiracPDF,UnivariatePDF);
	itkNewMacro(Self);

	/** position of the dirac impulsion*/
	void SetParameters(double value = 0.0);

	double GetLikelihood(const double &x);
	double GetMaximumLikelihoodValue();
	double GetLogLikelihood(const double &x);
protected:
	DiracPDF();
	~DiracPDF();

	double _DrawNewSample();
private:
	DiracPDF(const Self&);//purposely not implemented
	const Self & operator=( const Self & );	 //purposely not implemented
};

/** \class NormalPDF
* \brief NormalPDF Normal Gaussian distribution, default is mean=0, var=1;
*/
class NormalPDF : public UnivariatePDF {
public:
	/** Standard class typedefs. */
	typedef NormalPDF						Self;
	typedef UnivariatePDF					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(NormalPDF,UnivariatePDF);
	itkNewMacro(Self);

	/** Specify the mean and variance of the distribution */
	void SetParameters(double mean=0.0, double var=1.0);
	double GetLikelihood(const double &x);
	double GetMaximumLikelihoodValue();
	double GetLogLikelihood(const double &x);

	vnl_vector<double> GetParameters();

protected:
	NormalPDF();
	~NormalPDF();

	double _DrawNewSample();
	double m_mean, m_var, m_std;
private:
	NormalPDF(const Self&); //purposely not implemented
	const Self & operator=( const Self & );	 //purposely not implemented
};



/** \class UniformPDF
* \brief UniformPDF Continuous uniform distribution, default is [min = 0 ; max=1]
*/
class UniformPDF : public UnivariatePDF {
public:
	/** Standard class typedefs. */
	typedef UniformPDF						Self;
	typedef UnivariatePDF					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(UniformPDF,UnivariatePDF);
	itkNewMacro(Self);

	void SetParameters(double min=0, double max=1);
	double GetLikelihood(const double &x);
	double GetMaximumLikelihoodValue();
	double GetLogLikelihood(const double &x);
protected:
	UniformPDF();
	~UniformPDF();

	double _DrawNewSample();
	double m_min, m_max;
private:
	UniformPDF(const Self&); //purposely not implemented
	const Self & operator=( const Self & );	 //purposely not implemented
};

/** \class LogNormalPDF
* \brief LogNormalPDF LogNormal distribution, default is [m = 0 ; s2 =1]
*/
class LogNormalPDF : public UnivariatePDF {
public:
	/** Standard class typedefs. */
	typedef LogNormalPDF					Self;
	typedef UnivariatePDF					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(LogNormalPDF,UnivariatePDF);
	itkNewMacro(Self);

	void SetParameters(double m=0.0, double s2=1.0);
	void SetParametersViaMeanAndMode(double mean=1.648721270700128, double mode=0.367879441171442);
	double GetLikelihood(const double &x);
	double GetMaximumLikelihoodValue();
	double GetLogLikelihood(const double &x);

protected:
	LogNormalPDF();
	~LogNormalPDF();

	double _DrawNewSample();
	double m_mean, m_var, m_std;
private:
	LogNormalPDF(const Self&); //purposely not implemented
	const Self & operator=( const Self & );	 //purposely not implemented
};


/** \class TriangularPDF
* \brief TriangularPDF Triangular distribution, with parameters {min, mode, max}
* the PDF is zero outside [min max], and maximum at mode 
* default is {0  0.5  1}
* http://en.wikipedia.org/wiki/Triangular_distribution
*/
class TriangularPDF : public UnivariatePDF {
public:
	/** Standard class typedefs. */
	typedef TriangularPDF						Self;
	typedef UnivariatePDF					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(TriangularPDF,UnivariatePDF);
	itkNewMacro(Self);

	void SetParameters(double min=0.0, double mode=0.5, double max=1.0);
	double GetLikelihood(const double &x);
	double GetMaximumLikelihoodValue();
	double GetLogLikelihood(const double &x);
protected:
	TriangularPDF();
	~TriangularPDF();

	double _DrawNewSample();
	double m_min, m_mode, m_max;
	double m_totalDiff, m_leftDiff, m_rightDiff, m_diffRatio, m_p1, m_p2; //useful temporary variables.

private:
	TriangularPDF(const Self&); //purposely not implemented
	const Self & operator=( const Self & );	 //purposely not implemented
};

/** \class TrapezoidalPDF
* \brief TrapezoidalPDF TrapezoidalPDF distribution, with parameters {a, b, c, d}
* the PDF is zero outside [a d], flat in [b c], and linear in the extremities
* default is {0  0.1 0.9 1}
* http://metgen.pagesperso-orange.fr/metrologiefr24.htm
*/
class TrapezoidalPDF : public UnivariatePDF {
public:
	/** Standard class typedefs. */
	typedef TrapezoidalPDF						Self;
	typedef UnivariatePDF					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(TrapezoidalPDF,UnivariatePDF);
	itkNewMacro(Self);

	void SetParameters(double a=0.0, double b=0.1, double c=0.9, double d=1.0);
	double GetLikelihood(const double &x);
	double GetMaximumLikelihoodValue();
	double GetLogLikelihood(const double &x);
protected:
	TrapezoidalPDF();
	~TrapezoidalPDF();

	double _DrawNewSample();
	double m_a, m_b, m_c, m_d;
	double m_invdiff1, m_invdiff2, m_u, m_th1, m_th2, m_f1, m_o1, m_f2; //intermediate variables for speeding up computations.
private:
	TrapezoidalPDF(const Self&); //purposely not implemented
	const Self & operator=( const Self & );	 //purposely not implemented
};


/** \class LinearCombinationPDF
* \brief LinearCombinationPDF Univariate continuous PDF defined as a linear combination of other PDFs
* generates the random variable y = a*x+b, where a, x and b are univariate pdfs
* by default, a and b are DiracPDF with positions 0
* \todo add method to access the involved distributions a,b and x, to simplify usage by allowing modifications of their parameters instead of setting new distributions
* \todo GetLikelihood and GetLogLikelihood member methods are not implemented
*/
class LinearCombinationPDF : public UnivariatePDF {
public:
	/** Standard class typedefs. */
	typedef LinearCombinationPDF			Self;
	typedef UnivariatePDF					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(LinearCombinationPDF,UnivariatePDF);
	itkNewMacro(Self);
	
	void SetParameters(UnivariatePDF *x = static_cast<UnivariatePDF *>(m_default_x),
		UnivariatePDF *a = static_cast<UnivariatePDF *>(m_default_a), 
		UnivariatePDF *b = static_cast<UnivariatePDF *>(m_default_b));
	/** NOT IMPLEMENTED*/
	double GetLikelihood(const double &x);
	double GetMaximumLikelihoodValue();
	/** NOT IMPLEMENTED*/
	double GetLogLikelihood(const double &x);
protected:
	LinearCombinationPDF();
	~LinearCombinationPDF();

	double _DrawNewSample();
	UnivariatePDF::Pointer m_a, m_b, m_x;
	static NormalPDF::Pointer m_default_x;
	static DiracPDF::Pointer m_default_a, m_default_b;
private:
	LinearCombinationPDF(const Self&); //purposely not implemented
	const Self & operator=( const Self & );	 //purposely not implemented
};

/** \class UnivariateMixturePDF
* \brief UnivariateMixturePDF mixture of univariate pdfs
* Instances of this class shall be feed with one or more input univariate pdfs, each with an associated relative weight
* To generate a random variable, one of the input pdfs is randomly selected with a probability proportional to its associated weight, and a sample is drawn from this one
* \warning By default, no PDF is available - however, no check is performed, so the program may crash...
*/
class UnivariateMixturePDF : public UnivariatePDF {
public:
	/** Standard class typedefs. */
	typedef UnivariateMixturePDF			Self;
	typedef UnivariatePDF					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(UnivariateMixturePDF,UnivariatePDF);
	itkNewMacro(Self);

	void AddPDF(UnivariatePDF *pdf, double weight = 1.0);
	double GetLikelihood(const double &x);
	double GetMaximumLikelihoodValue();
	double GetLogLikelihood(const double &x);
protected:
	UnivariateMixturePDF();
	~UnivariateMixturePDF();

	std::vector<UnivariatePDF::Pointer> m_univ_pdfs;
	std::vector<double> m_weights; double m_sumWeights;

	double _DrawNewSample();
private:
	UnivariateMixturePDF(const Self&); //purposely not implemented
	const Self & operator=( const Self & );	 //purposely not implemented
};



} // namespace psciob


#endif //UNIVARIATEPDF_H_