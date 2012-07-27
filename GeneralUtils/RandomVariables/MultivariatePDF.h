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
 * \file MultivariatePDF.h
 * \author Rémi Blanc 
 * \26. September 2011
*/

#ifndef MULTIVARIATEPDF_H_
#define MULTIVARIATEPDF_H_

#include "UnivariatePDF.h"
#include <vnl/algo/vnl_symmetric_eigensystem.h>

//#include "2DTransformUtils.h"
#include "3DTransformUtils.h"

namespace psciob {


//IDEA: enable drawing just one variable (/ a subset of variables) out of a multivariate distribution
//-> for composite multivariate distribution (i.e. of sub-type IndependentPDFs), draw only from the relevant distribution


/** \class MultivariatePDF
* \brief MultivariatePDF Base Class for multivariate probability density functions
*/
class MultivariatePDF : public PDFBase< vnl_vector<double> > {
public:
	/** Standard class typedefs. */
	typedef MultivariatePDF					Self;
	typedef PDFBase<vnl_vector<double>>	Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(MultivariatePDF,PDFBase);

	inline unsigned int GetNumberOfDimensions();
protected:
	MultivariatePDF();
	~MultivariatePDF();

	unsigned int m_dimension;
private: 
	MultivariatePDF(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};


/** \class MVN_PDF
* \brief MVN_PDF Multivariate Normal Probability density function, characterized by its mean and covariance matrix
* Default dimensionality is 2 , with 0 mean and identity covariance
*/
class MVN_PDF : public MultivariatePDF {
public:
	/** Standard class typedefs. */
	typedef MVN_PDF							Self;
	typedef MultivariatePDF					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(MVN_PDF,MultivariatePDF);
	itkNewMacro(Self);

	/** Set the dimensionality of the distribution, and initialize it with 0 mean and identity covariance */
	void SetNumberOfDimensions(unsigned int n);

	/** Set the mean vector and covariance matrix - resets the dimensionality if necessary */
	void SetParameters(const vnl_vector<double> &mean, const vnl_matrix<double> &cov);

	/** this interface sets a mean vector with all elements equal to mean
	* and a covariance which is cov * ID
	* keeping the current dimensionality of the PDF.
	*/
	void SetParameters(double mean = 0, double cov = 1);
	double GetLikelihood(const vnl_vector<double> &x);
	double GetMaximumLikelihoodValue();
	double GetLogLikelihood(const vnl_vector<double> &x);
protected:
	MVN_PDF();
	~MVN_PDF();

	vnl_vector<double> _DrawNewSample();

	vnl_matrix<double> m_mean;	vnl_matrix<double> m_cov;
	vnl_matrix<double> m_std, m_invcov;
	double m_det, m_normalizationFactor;
private: 
	MVN_PDF(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};


/** \class IndependentPDFs
* \brief IndependentPDFs concatenates an arbitrary number of univariate and/or multivariate PDFs to form a multivariate distribution
* The random variable is formed by drawing independently from the input PDFs
* The order in which the PDFs are inserted define the order of the variables in the generated samples, and cannot be modified
* \warning by default, no pdf are given, so trying to draw a sample may result in a crash (not tested)
* \todo It could potentially be useful to get access to the stored PDFs, to be able to modify their parameters.
*/
class IndependentPDFs : public MultivariatePDF {
public:
	/** Standard class typedefs. */
	typedef IndependentPDFs					Self;
	typedef MultivariatePDF					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(IndependentPDFs,MultivariatePDF);
	itkNewMacro(IndependentPDFs);

	/** Add another univariate PDF */
	void AddUnivariatePDF(UnivariatePDF *pdf);
	/** Add another multivariate PDF */
	void AddMultivariatePDF(MultivariatePDF *pdf);
	double GetLikelihood(const vnl_vector<double> &x);
	double GetMaximumLikelihoodValue();
	double GetLogLikelihood(const vnl_vector<double> &x);

protected:
	IndependentPDFs();
	~IndependentPDFs();

	vnl_vector<double> _DrawNewSample();

	std::vector<bool> m_univ_multiv_law; //true = multivariate law / false = univariate law... necessary in case of 'multivariate' distr of dimensionality 1 ......
	std::vector<unsigned int> m_dim_laws;
	std::vector<UnivariatePDF::Pointer> m_univ_pdfs;
	std::vector<MultivariatePDF::Pointer> m_multiv_pdfs;
private: 
	IndependentPDFs(const Self&);           //purposely not implemented
	const Self & operator=( const Self & ); //purposely not implemented
};


/** \class UniformBoxPDF
* \brief UniformBoxPDF multidimensional uniform continuous variable
* Given a vector v of size 2N, it concatenates N independent univariate uniform distributions, each in the interval [v(2*i) v(2*i+1)]
* default is a [0 1; 0 1] box
*/
class UniformBoxPDF : public MultivariatePDF {
public:
	/** Standard class typedefs. */
	typedef UniformBoxPDF					Self;
	typedef MultivariatePDF					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(UniformBoxPDF,MultivariatePDF);
	itkNewMacro(UniformBoxPDF);

	/** Set the box in which to draw variables 
	* dimension of the vector must be pair, and formatted as [min_d1 max_d1 min_d2 max_d2...]
	*/
	void SetBox(const vnl_vector<double> &bbox);

	double GetLikelihood(const vnl_vector<double> &x);
	double GetMaximumLikelihoodValue();
	double GetLogLikelihood(const vnl_vector<double> &x);

protected:
	UniformBoxPDF();
	~UniformBoxPDF();

	vnl_vector<double> _DrawNewSample();
	IndependentPDFs::Pointer m_pdf;
private: 
	UniformBoxPDF(const Self&);				//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};

/** \class IndependentEulerRotationsPDFs
* \brief IndependentEulerRotationsPDFs 3D distribution of the 3 euler angles resulting from a composition of an arbitrary number of independent simples rotations
* The convention for the Euler Angle rotation is 
* the angle of each simple rotation is characterized by a univariate density, and applied around either X_AXIS, Y_AXIS or Z_AXIS
*/
class IndependentEulerRotationsPDFs : public MultivariatePDF {
public:
	/** Standard class typedefs. */
	typedef IndependentEulerRotationsPDFs	Self;
	typedef MultivariatePDF					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(IndependentEulerRotationsPDFs,MultivariatePDF);
	itkNewMacro(Self);

	enum ROTATIONAROUND { X_AXIS, Y_AXIS, Z_AXIS };

	/** Around which axis to rotate, and associated density */
	void AddIndependentRotationPDF(ROTATIONAROUND dir, UnivariatePDF *pdf);

	double GetLikelihood(const vnl_vector<double> &x);
	double GetMaximumLikelihoodValue();
	double GetLogLikelihood(const vnl_vector<double> &x);

protected:
	IndependentEulerRotationsPDFs();
	~IndependentEulerRotationsPDFs();

	vnl_vector<double> _DrawNewSample();

	std::vector<ROTATIONAROUND> m_rotation_axis;
	std::vector<UnivariatePDF::Pointer > m_univ_pdfs;
	vnl_matrix<double> m_rotMat;

private: 
	IndependentEulerRotationsPDFs(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};

/** \class MultivariateMixturePDF
* \brief MultivariateMixturePDF mixture of multivariate pdfs
* Instances of this class shall be feed with one or more input multivariate pdfs of the same dimensionality, each with an associated relative weight
* To generate a random variable, one of the input pdfs is randomly selected with a probability proportional to its associated weight, and a sample is drawn from this one
* \warning By default, no PDF is available - however, no check is performed, so the program may crash...
*/
class MultivariateMixturePDF : public MultivariatePDF {
public:
	/** Standard class typedefs. */
	typedef MultivariateMixturePDF			Self;
	typedef MultivariatePDF					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(MultivariateMixturePDF,MultivariatePDF);
	itkNewMacro(Self);

	/** Add a new multivariate PDF to the mixture
	* the dimensionality is determined by the first PDF inserted 
	* An expection is raised if subsequent PDFs don't match this dimensionality
	*/
	void AddPDF(MultivariatePDF *pdf, double weight = 1.0);
	
	double GetLikelihood(const vnl_vector<double> &x);
	double GetMaximumLikelihoodValue();
	double GetLogLikelihood(const vnl_vector<double> &x);

protected:
	MultivariateMixturePDF();
	~MultivariateMixturePDF();

	std::vector<MultivariatePDF::Pointer> m_multiv_pdfs;
	std::vector<double> m_weights; double m_sumWeights;

	vnl_vector<double> _DrawNewSample();

private: 
	MultivariateMixturePDF(const Self&);	//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};



} // namespace psciob


#endif //MULTIVARIATEPDF_H_