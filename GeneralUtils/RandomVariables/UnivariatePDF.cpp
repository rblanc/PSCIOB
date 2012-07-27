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
 * \file UnivariatePDF.cpp
 * \author Rémi Blanc 
 * \date 27. February 2012
*/

#include "UnivariatePDF.h"

using namespace psciob;

//
//IntegerUnivariatePDF
//
IntegerUnivariatePDF::IntegerUnivariatePDF() : PDFBase() {}
IntegerUnivariatePDF::~IntegerUnivariatePDF() {}
unsigned int IntegerUnivariatePDF::GetNumberOfDimensions() { return m_dimension; }

//
//UniformIntegerUnivariatePDF
//
UniformIntegerUnivariatePDF::UniformIntegerUnivariatePDF() : IntegerUnivariatePDF() { m_min=0; m_max=1; m_diff = 1;}
UniformIntegerUnivariatePDF::~UniformIntegerUnivariatePDF() {}
void UniformIntegerUnivariatePDF::SetParameters(long min, long max) {
	if (m_max<m_min) throw DeformableModelException("UniformIntegerUnivariatePDF: inconsistent parameters ; max >= min");
	m_min = min; m_max = max; m_diff = m_max-m_min; 
}
inline double UniformIntegerUnivariatePDF::GetLikelihood(const long &x) { 
	return (1.0/static_cast<double>(1.0+m_diff));
}
inline double UniformIntegerUnivariatePDF::GetMaximumLikelihoodValue() {
	return (1.0/static_cast<double>(1.0+m_diff));
}
inline double UniformIntegerUnivariatePDF::GetLogLikelihood(const long &x) { 
	return (-log(static_cast<double>(1.0+m_diff))); 
}
long UniformIntegerUnivariatePDF::_DrawNewSample() { 
	return (m_min + this->m_baseGenerator->GetIntegerVariate( m_diff )); 
}

//
//PoissonPDF
//
PoissonPDF::PoissonPDF() : IntegerUnivariatePDF() { m_lambda = 1; }
PoissonPDF::~PoissonPDF() {};
void PoissonPDF::SetLambda(long lambda) {
	if (lambda<=0) throw DeformableModelException("PoissonPDF: parameter must be >0");
	m_lambda = lambda;
}
inline double PoissonPDF::GetLikelihood(const long &x) { 
	double tmp=0; 
	for (unsigned i=1 ; i<x ; i++) tmp+=log(static_cast<double>(i));
	return ( exp( x*log(m_lambda) - m_lambda - tmp ) ); 
}
inline double PoissonPDF::GetMaximumLikelihoodValue() {
	double tmp=0; long x = m_lambda;
	for (unsigned i=1 ; i<x ; i++) tmp+=log(static_cast<double>(i));
	return ( exp( x*log(m_lambda) - m_lambda - tmp ) ); 
}
inline double PoissonPDF::GetLogLikelihood(const long &x) { 
	double tmp=0; 
	for (unsigned i=1 ; i<x ; i++) tmp+=log(static_cast<double>(i));
	return ( x*log(m_lambda) - m_lambda - tmp ) ; 
}
inline long PoissonPDF::_DrawNewSample() { 
	double L, p=1; unsigned k=0;
	if (m_lambda<100) {
		L = exp(-m_lambda);	while(p>L) { k++; p*=this->m_baseGenerator->GetUniformVariate(0,1); }
	}
	else {//pb: if lambda is too large, there is a bad precision on exp(-lambda)
		L = -m_lambda;		while(p>L) { k++; p+=log(this->m_baseGenerator->GetUniformVariate(0,1)); }
	}		
	return (k-1); 
}


//
//
//
UnivariatePDF::UnivariatePDF() : PDFBase() {}
unsigned int UnivariatePDF::GetNumberOfDimensions() {return m_dimension;}

//
//DiracPDF
//
DiracPDF::DiracPDF() : UnivariatePDF() { SetParameters(0.0); }
DiracPDF::~DiracPDF() {};
void DiracPDF::SetParameters(double value) {
	m_lastSample = value;
}
inline double DiracPDF::GetLikelihood(const double &x) {
	if (x==m_lastSample) return 1; 
	else return 0; 
}
inline double DiracPDF::GetMaximumLikelihoodValue() {
	return 1; 
}
inline double DiracPDF::GetLogLikelihood(const double &x)	{
	if (x==m_lastSample) return 0;
	else return MINUSINF; 
}
inline double DiracPDF::_DrawNewSample() { 
	return m_lastSample;
}

//
//NormalPDF
//
NormalPDF::NormalPDF() : UnivariatePDF() { SetParameters(); }
NormalPDF::~NormalPDF() {}

void NormalPDF::SetParameters(double mean, double var) {
	if (var<0) throw DeformableModelException("Exception in NormalPDF::SetParameters() : cannot set a negative variance!"); 
	m_mean = mean; m_var = var; m_std = sqrt(m_var); 
}
inline double NormalPDF::GetLikelihood(const double &x)		{ 
	return ( exp(-(x-m_mean)*(x-m_mean)/(2.0*m_var)) / (m_std*SQRT2PI) ); 
}
inline double NormalPDF::GetMaximumLikelihoodValue() {
	return 1.0/(m_std*SQRT2PI); 
}
inline double NormalPDF::GetLogLikelihood(const double &x)	{ 
	return ( -(x-m_mean)*(x-m_mean)/(2.0*m_var) - log(m_std*SQRT2PI) ); 
}

vnl_vector<double> NormalPDF::GetParameters() { 
	vnl_vector<double> v(2); v(0) = m_mean; v(1) = m_var; 
	return v;
}
inline double NormalPDF::_DrawNewSample() { 
	return m_baseGenerator->GetNormalVariate(m_mean, m_var); 
}

//
//UniformPDF
//
UniformPDF::UniformPDF() : UnivariatePDF() {SetParameters();}
UniformPDF::~UniformPDF() {};
void UniformPDF::SetParameters(double min, double max) {
	if ( (min>max) ) throw DeformableModelException("Exception in NormalPDF::SetParameters() : parameters should be min<=max"); 
	m_min = min; m_max = max;
}
inline double UniformPDF::GetLikelihood(const double &x) {
	if (x<m_min) return 0;
	if (x>m_max) return 0;
	return (1.0/(m_max-m_min));
}
inline double UniformPDF::GetMaximumLikelihoodValue() {
	return (1.0/(m_max-m_min)); 
}
inline double UniformPDF::GetLogLikelihood(const double &x) {
	if (x<m_min) return MINUSINF;
	if (x>m_max) return MINUSINF;
	return (-log(m_max-m_min));
}
inline double UniformPDF::_DrawNewSample() { 
	return m_baseGenerator->GetUniformVariate(m_min, m_max);
}

//
//LogNormalPDF
//
LogNormalPDF::LogNormalPDF() : UnivariatePDF() { SetParameters(); }
LogNormalPDF::~LogNormalPDF() {};
void LogNormalPDF::SetParameters(double m, double s2) {
	if (s2<0) throw DeformableModelException("Exception in LogNormalPDF::SetParameters() : cannot set a negative variance!"); 
	m_mean = m; m_var = s2; m_std = sqrt(m_var); 
}
void LogNormalPDF::SetParametersViaMeanAndMode(double mean, double mode) {
	if (mean<mode) throw DeformableModelException("Exception in LogNormalPDF::SetParameters() : must have mean >= mode"); 
	if (mode<0)    throw DeformableModelException("Exception in LogNormalPDF::SetParameters() : the mode must be positive"); 
	m_mean = (log(mode)+2.0*log(mean))/3.0;
	m_var = 2.0*(log(mean)-log(mode))/3.0; 
	m_std = sqrt(m_var); 
}
inline double LogNormalPDF::GetLikelihood(const double &x) {
	if (x<0) return 0; //throw DeformableModelException("Exception in LogNormalPDF::Evaluate() : undefined for negative values...");
	double tmp = log(x)-m_mean;
	return ( exp(-tmp*tmp/(2.0*m_var))/(x*m_std*SQRT2PI) ); 
}
inline double LogNormalPDF::GetMaximumLikelihoodValue() {
	double x = exp(m_mean-m_var);
	double tmp = log(x)-m_mean;
	return ( exp(-tmp*tmp/(2.0*m_var))/(x*m_std*SQRT2PI) ); 
}
inline double LogNormalPDF::GetLogLikelihood(const double &x) {
	if (x<=0) return MINUSINF; //throw DeformableModelException("Exception in LogNormalPDF::Evaluate() : undefined for negative values...");
	double tmp = log(x)-m_mean;
	return ( (-tmp*tmp/(2.0*m_var)) - log(x*m_std*SQRT2PI) ); 
}
inline double LogNormalPDF::_DrawNewSample() { 
	return exp(m_mean+m_std*m_baseGenerator->GetNormalVariate(0,1)); 
}

//
//TriangularPDF
//
TriangularPDF::TriangularPDF() : UnivariatePDF() { SetParameters(); }
TriangularPDF::~TriangularPDF() {};
void TriangularPDF::SetParameters(double min, double mode, double max) {
	if ( (min>mode) || (mode>max) ) throw DeformableModelException("Exception in NormalPDF::SetParameters() : parameters should be min<=mode<=max"); 
	m_min = min;   //a
	m_mode = mode; //c
	m_max = max;   //b
	m_totalDiff = m_max-m_min; //b-a
	m_leftDiff = m_mode-m_min; //c-a
	m_rightDiff = m_max-m_mode;//b-c
	m_diffRatio = m_leftDiff/m_totalDiff; // (c-a)/(b-a)
	m_p1 = (m_totalDiff * m_leftDiff);  //   (b-a)*(c-a) 
	m_p2 = (m_totalDiff * m_rightDiff); //   (b-a)*(b-c)
}
inline double TriangularPDF::GetLikelihood(const double &x) {
	if (x<m_min) return 0;
	if (x>m_max) return 0;
	if (x<=m_mode) return ( 2.0*(x-m_min) / m_p1 ); //2*(x-a)/((b-a)*(c-a))
	return ( 2.0*(m_max-x) / m_p2 );// 2*(b-x)/((b-a)*(b-c))
}
inline double TriangularPDF::GetMaximumLikelihoodValue() {
	return ( 2.0/m_totalDiff );
}
inline double TriangularPDF::GetLogLikelihood(const double &x) {
	if (x<=m_min) return MINUSINF;
	if (x>=m_max) return MINUSINF;
	if (x<=m_mode) return log(2.0*(x-m_min) / m_p1);
	return log(2.0*(m_max-x) / m_p2);//else
}
inline double TriangularPDF::_DrawNewSample() { 
	double tmp = m_baseGenerator->GetUniformVariate(0, 1);
	if (tmp<m_diffRatio) return ( m_min + sqrt(   tmp   * m_p1) );
	else                 return ( m_max - sqrt((1.0-tmp)* m_p2) );
}

//
//TrapezoidalPDF
//
TrapezoidalPDF::TrapezoidalPDF() : UnivariatePDF() { SetParameters(); }
TrapezoidalPDF::~TrapezoidalPDF() {};
void TrapezoidalPDF::SetParameters(double a, double b, double c, double d) {
	if ( (a>b) || (b>c) || (c>d) ) throw DeformableModelException("Exception in NormalPDF::SetParameters() : parameters should be min<=mode<=max"); 
	m_a = a; m_b = b; m_c = c; m_d = d;
	m_invdiff1 = 1.0/(m_b-m_a);
	m_invdiff2 = 1.0/(m_d-m_c);
	m_u = 2.0/(m_d+m_c-m_b-m_a);
	m_th1 = m_u*(m_b-m_a)/2.0;
	m_th2 = 1.0-m_u*(m_d-m_c)/2.0;
	m_f1 = 2.0*(m_b-m_a)/m_u;
	m_o1 = (m_a+m_b)/2.0;
	m_f2 = 2.0*(m_d-m_c)/m_u;
}
inline double TrapezoidalPDF::GetLikelihood(const double &x) {
	if (x<m_a) return 0;
	if (x<m_b) return m_u*(x-m_a)*m_invdiff1;
	if (x<m_c) return m_u;
	if (x<m_d) return m_u*(m_d-x)*m_invdiff2;
	return 0;
}
inline double TrapezoidalPDF::GetMaximumLikelihoodValue() {
	return m_u;
}
inline double TrapezoidalPDF::GetLogLikelihood(const double &x) {
	if (x<m_a) return MINUSINF;
	if (x<m_b) return log(m_u*(x-m_a)*m_invdiff1);
	if (x<m_c) return log(m_u);
	if (x<m_d) return log(m_u*(m_d-x)*m_invdiff2);
	return MINUSINF;
}
inline double TrapezoidalPDF::_DrawNewSample() { 
	double tmp = m_baseGenerator->GetUniformVariate(0, 1);
	if ( tmp < m_th1 )  return ( m_a+sqrt(tmp * m_f1) );
	if ( tmp < m_th2 )  return ( m_o1 + tmp/m_u );
	return ( m_d - sqrt((1.0-tmp) * m_f2) );
}


//
//LinearCombinationPDF
//
//static members
NormalPDF::Pointer LinearCombinationPDF::m_default_x = NormalPDF::New();
DiracPDF::Pointer LinearCombinationPDF::m_default_a = DiracPDF::New();
DiracPDF::Pointer LinearCombinationPDF::m_default_b = DiracPDF::New();
LinearCombinationPDF::LinearCombinationPDF() : UnivariatePDF() { SetParameters( m_default_x, m_default_b, m_default_a); }
LinearCombinationPDF::~LinearCombinationPDF() {};
void LinearCombinationPDF::SetParameters(UnivariatePDF *x, UnivariatePDF *a, UnivariatePDF *b) {
	m_a=a; m_b=b; m_x=x;
}
inline double LinearCombinationPDF::GetLikelihood(const double &x) { 
	//sum of RV => pdf = convolution of indiv. pdfs.		//product => integral (http://en.wikipedia.org/wiki/Product_distribution)
	//TODO: learn the pdf! questions is how to learn it? how many samples, kernel distrib? simple histogram? ...
	throw DeformableModelException("LinearCombinationPDF::GetLikelihood() : not implemented... and not easy to implement in general..."); 
}
inline double LinearCombinationPDF::GetMaximumLikelihoodValue() {
	throw DeformableModelException("LinearCombinationPDF::GetMaximumLikelihoodValue() : not implemented... and not easy to implement in general..."); 
}
inline double LinearCombinationPDF::GetLogLikelihood(const double &x) { 
	//sum of RV => pdf = convolution of indiv. pdfs.		//product => integral (http://en.wikipedia.org/wiki/Product_distribution)
	//TODO: learn the pdf! questions is how to learn it? how many samples, kernel distrib? simple histogram? ...
	throw DeformableModelException("LinearCombinationPDF::GetLogLikelihood() : not implemented... and not easy to implement in general..."); 
}
inline double LinearCombinationPDF::_DrawNewSample() { 
	return (m_a->DrawSample() * m_x->DrawSample() + m_b->DrawSample() ); 
}


//
//UnivariateMixturePDF
//
UnivariateMixturePDF::UnivariateMixturePDF() : UnivariatePDF() { m_sumWeights=0; }
UnivariateMixturePDF::~UnivariateMixturePDF() {};
void UnivariateMixturePDF::AddPDF(UnivariatePDF *pdf, double weight) {
	m_univ_pdfs.push_back(pdf);
	m_weights.push_back(weight);
	m_sumWeights += weight;
}
inline double UnivariateMixturePDF::GetLikelihood(const double &x) { 
	double likelihood=0, p_pdf, likelihood_pdf;
	for (unsigned i = 0 ; i<m_univ_pdfs.size() ; i++) {
		p_pdf = m_weights[i]/m_sumWeights;
		likelihood_pdf = m_univ_pdfs[i]->GetLikelihood(x);
		likelihood += p_pdf * likelihood_pdf;
	}
	return likelihood;
}
inline double UnivariateMixturePDF::GetMaximumLikelihoodValue() {
	throw DeformableModelException("UnivariateMixturePDF::GetMaximumLikelihoodValue() : not implemented... and not easy to implement in general..."); 
	/* if the distribution are separated, something like this could work
	// in the general case, the maximum of the mixture is not necessarily related to the maximum of any of its components...
	// it would certainly be necessary to implement a learning scheme ; drawing lots and samples and estimating a distribution out of it...
	double maxLikelihood=0;
	for (unsigned i = 0 ; i<m_univ_pdfs.size() ; i++) {
		maxLikelihood = std::max(maxLikelihood, m_univ_pdfs[i]->GetMaximumLikelihoodValue(x) * m_weights[i]/m_sumWeights
		p_pdf = m_weights[i]/m_sumWeights;
		likelihood_pdf = m_univ_pdfs[i]->GetLikelihood(x);
		likelihood += p_pdf * likelihood_pdf;
	}
	return maxLikelihood;
	*/
}
inline double UnivariateMixturePDF::GetLogLikelihood(const double &x) { 
	double tmp = GetLikelihood(x);
	if (tmp==0) return MINUSINF;
	return log(tmp); 
}
inline double UnivariateMixturePDF::_DrawNewSample() { 
	//select the generating distribution based on the weights...
	double rnd = m_baseGenerator->GetVariateWithOpenUpperRange(m_sumWeights);
	unsigned int n_pdf;
	for (n_pdf = 0; n_pdf<m_univ_pdfs.size() ; n_pdf++) {
		if (rnd<m_weights[n_pdf]) break;
		else rnd-=m_weights[n_pdf];
	}
	return m_univ_pdfs[n_pdf]->DrawSample();
}
