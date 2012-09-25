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
UniformIntegerUnivariatePDF::UniformIntegerUnivariatePDF() : IntegerUnivariatePDF() { m_params.set_size(2); m_min=0; m_max=1; m_diff = 1; m_params(0) = m_min; m_params(1) = m_max;}
UniformIntegerUnivariatePDF::~UniformIntegerUnivariatePDF() {}
bool UniformIntegerUnivariatePDF::SetParameters(const vnl_vector<double> &p) {
	if (p.size()!=2) return false;
	return SetParameters(p(0), p(1));
}
bool UniformIntegerUnivariatePDF::SetParameters(long min, long max) {
	if (m_max<m_min) return false;
	m_min = min; m_max = max; m_diff = m_max-m_min; 
	m_params(0) = m_min; m_params(1) = m_max;
	return true;
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
inline void UniformIntegerUnivariatePDF::_DrawNewSample() { 
	m_lastSample = m_min + this->m_baseGenerator->GetIntegerVariate( m_diff );
}

//
//PoissonPDF
//
PoissonPDF::PoissonPDF() : IntegerUnivariatePDF() { m_params.set_size(1); m_lambda = 1; m_params(0) = m_lambda;}
PoissonPDF::~PoissonPDF() {};
bool PoissonPDF::SetParameters(const vnl_vector<double> &p) {
	if (p.size()!=1) return false;
	return SetLambda(p(0));
}
bool PoissonPDF::SetLambda(long lambda) {
	if (lambda<=0) return false;
	m_lambda = lambda; m_params(0) = m_lambda;
	return true;
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
inline void PoissonPDF::_DrawNewSample() { 
	double L, p=1; 
	m_lastSample = 0;
	if (m_lambda<100) {
		L = exp(-m_lambda);	while(p>L) { m_lastSample++; p*=this->m_baseGenerator->GetUniformVariate(0,1); }
	}
	else {//pb: if lambda is too large, there is a bad precision on exp(-lambda)
		L = -m_lambda;		while(p>L) { m_lastSample++; p+=log(this->m_baseGenerator->GetUniformVariate(0,1)); }
	}		
	m_lastSample--;
}


//
//
//
UnivariatePDF::UnivariatePDF() : PDFBase() {}
unsigned int UnivariatePDF::GetNumberOfDimensions() {return m_dimension;}

//
//DiracPDF
//
DiracPDF::DiracPDF() : UnivariatePDF() { m_params.set_size(1); SetParameters(0.0); }
DiracPDF::~DiracPDF() {};
bool DiracPDF::SetParameters(const vnl_vector<double> &p) {
	if (p.size()!=1) return false;
	return SetParameters(p(0));
}
bool DiracPDF::SetParameters(double value) {
	m_lastSample = value;
	m_params(0) = value;
	return true;
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
inline void DiracPDF::_DrawNewSample() {}

//
//NormalPDF
//
NormalPDF::NormalPDF() : UnivariatePDF() { m_params.set_size(2); SetParameters(); }
NormalPDF::~NormalPDF() {}
bool NormalPDF::SetParameters(const vnl_vector<double> &p) {
	if (p.size()!=2) return false;
	return SetParameters(p(0), p(1));
}
bool NormalPDF::SetParameters(double mean, double var) {
	if (var<0) return false;
	m_mean = mean; m_var = var; m_std = sqrt(m_var); 
	m_params(0) = m_mean; m_params(1) = m_var;
	return true;
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

inline void NormalPDF::_DrawNewSample() { 
	m_lastSample = m_baseGenerator->GetNormalVariate(m_mean, m_var); 
}

//
//UniformPDF
//
UniformPDF::UniformPDF() : UnivariatePDF() { m_params.set_size(2); SetParameters(); }
UniformPDF::~UniformPDF() {};
bool UniformPDF::SetParameters(const vnl_vector<double> &p) {
	if (p.size()!=2) return false;
	return SetParameters(p(0), p(1));
}
bool UniformPDF::SetParameters(double min, double max) {
	if ( (min>max) ) return false;
	m_min = min; m_max = max; m_params(0) = m_min; m_params(1) = m_max;
	return true;
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
inline void UniformPDF::_DrawNewSample() { 
	m_lastSample = m_baseGenerator->GetUniformVariate(m_min, m_max);
}

//
//LogNormalPDF
//
LogNormalPDF::LogNormalPDF() : UnivariatePDF() { m_params.set_size(2); SetParameters(); }
LogNormalPDF::~LogNormalPDF() {};
bool LogNormalPDF::SetParameters(const vnl_vector<double> &p) {
	if (p.size()!=2) return false;
	return SetParameters(p(0), p(1));
}
bool LogNormalPDF::SetParameters(double m, double s2) {
	if (s2<0) return false;
	m_mean = m; m_var = s2; m_std = sqrt(m_var); 
	m_params(0) = m_mean; m_params(1) = m_var;
	return true;
}
bool LogNormalPDF::SetParametersViaMeanAndMode(double mean, double mode) {
	if (mean<mode) return false; //throw DeformableModelException("Exception in LogNormalPDF::SetParameters() : must have mean >= mode"); 
	if (mode<0)    return false; //throw DeformableModelException("Exception in LogNormalPDF::SetParameters() : the mode must be positive"); 
	m_mean = (log(mode)+2.0*log(mean))/3.0;
	m_var = 2.0*(log(mean)-log(mode))/3.0; 
	m_params(0) = m_mean; m_params(1) = m_var;
	m_std = sqrt(m_var); 
	return true;
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
inline void LogNormalPDF::_DrawNewSample() { 
	m_lastSample = exp(m_mean+m_std*m_baseGenerator->GetNormalVariate(0,1)); 
}

//
//TriangularPDF
//
TriangularPDF::TriangularPDF() : UnivariatePDF() { m_params.set_size(3); SetParameters(); }
TriangularPDF::~TriangularPDF() {};
bool TriangularPDF::SetParameters(const vnl_vector<double> &p) {
	if (p.size()!=3) return false;//throw DeformableModelException("Exception in NormalPDF::SetParameters() : parameters should be min<=mode<=max"); 
	return SetParameters(p(0), p(1), p(2));
}
bool TriangularPDF::SetParameters(double min, double mode, double max) {
	if ( (min>mode) || (mode>max) ) return false;//throw DeformableModelException("Exception in NormalPDF::SetParameters() : parameters should be min<=mode<=max"); 
	m_min = min;   //a
	m_mode = mode; //c
	m_max = max;   //b
	m_params(0) = m_min; m_params(1) = m_mode; m_params(2) = m_max;
	m_totalDiff = m_max-m_min; //b-a
	m_leftDiff = m_mode-m_min; //c-a
	m_rightDiff = m_max-m_mode;//b-c
	m_diffRatio = m_leftDiff/m_totalDiff; // (c-a)/(b-a)
	m_p1 = (m_totalDiff * m_leftDiff);  //   (b-a)*(c-a) 
	m_p2 = (m_totalDiff * m_rightDiff); //   (b-a)*(b-c)
	return true;
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
inline void TriangularPDF::_DrawNewSample() { 
	double tmp = m_baseGenerator->GetUniformVariate(0, 1);
	if (tmp<m_diffRatio) m_lastSample = ( m_min + sqrt(   tmp   * m_p1) );
	else                 m_lastSample = ( m_max - sqrt((1.0-tmp)* m_p2) );
}

//
//TrapezoidalPDF
//
TrapezoidalPDF::TrapezoidalPDF() : UnivariatePDF() { m_params.set_size(4); SetParameters(); }
TrapezoidalPDF::~TrapezoidalPDF() {};
bool TrapezoidalPDF::SetParameters(const vnl_vector<double> &p) {
	if (p.size()!=4) return false;
	return SetParameters(p(0), p(1), p(2), p(3));
}
bool TrapezoidalPDF::SetParameters(double a, double b, double c, double d) {
	if ( (a>b) || (b>c) || (c>d) ) return false;//throw DeformableModelException("Exception in NormalPDF::SetParameters() : parameters should be min<=mode<=max"); 
	m_a = a; m_b = b; m_c = c; m_d = d;
	m_params(0) = m_a; m_params(1) = m_b; m_params(2) = m_c; m_params(3) = m_d; 
	m_invdiff1 = 1.0/(m_b-m_a);
	m_invdiff2 = 1.0/(m_d-m_c);
	m_u = 2.0/(m_d+m_c-m_b-m_a);
	m_th1 = m_u*(m_b-m_a)/2.0;
	m_th2 = 1.0-m_u*(m_d-m_c)/2.0;
	m_f1 = 2.0*(m_b-m_a)/m_u;
	m_o1 = (m_a+m_b)/2.0;
	m_f2 = 2.0*(m_d-m_c)/m_u;
	return true;
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
inline void TrapezoidalPDF::_DrawNewSample() { 
	double tmp = m_baseGenerator->GetUniformVariate(0, 1);
	if ( tmp < m_th1 )  {m_lastSample = ( m_a+sqrt(tmp * m_f1) ); return;}
	if ( tmp < m_th2 )  {m_lastSample = ( m_o1 + tmp/m_u ); return;}
	m_lastSample = ( m_d - sqrt((1.0-tmp) * m_f2) );
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
bool LinearCombinationPDF::SetParameters(const vnl_vector<double> &p) {
	if (p.size()!=m_nbP_a+m_nbP_b+m_nbP_x) return false;
	//vnl_vector<double> backupParams = m_params;
	if (!m_x->SetParameters(p.extract(m_nbP_x,0))) return false;
	if (!m_a->SetParameters(p.extract(m_nbP_a, m_nbP_x))) { //undo the modifications of the previous pdfs
		if (!m_x->SetParameters(m_params.extract(m_nbP_x,0))) throw DeformableModelException("LinearCombinationPDF::SetParameters : trying to set invalid parameters, and unable to rewind to previous parameters ; SHOULD NEVER HAPPEN!");
		return false;
	}
	if (!m_b->SetParameters(p.extract(m_nbP_b, m_nbP_x+m_nbP_a))) { //undo the modifications of the previous pdfs
		if (!m_a->SetParameters(m_params.extract(m_nbP_a, m_nbP_x))) throw DeformableModelException("LinearCombinationPDF::SetParameters : trying to set invalid parameters, and unable to rewind to previous parameters ; SHOULD NEVER HAPPEN!");
		if (!m_x->SetParameters(m_params.extract(m_nbP_x,0))) throw DeformableModelException("LinearCombinationPDF::SetParameters : trying to set invalid parameters, and unable to rewind to previous parameters ; SHOULD NEVER HAPPEN!");
		return false;
	}
	m_params = p;
	return true;
}
bool LinearCombinationPDF::SetParameters(UnivariatePDF *x, UnivariatePDF *a, UnivariatePDF *b) {
	m_x=x; m_a=a; m_b=b;
	m_nbP_a = m_a->GetParameters().size();
	m_nbP_b = m_b->GetParameters().size();
	m_nbP_x = m_x->GetParameters().size();
	m_params.set_size(m_nbP_a+m_nbP_b+m_nbP_x);
	for (unsigned i=0 ; i<m_nbP_x ; i++) m_params(i) = m_x->GetParameters()(i);
	for (unsigned i=0 ; i<m_nbP_a ; i++) m_params(m_nbP_x+i) = m_a->GetParameters()(i);
	for (unsigned i=0 ; i<m_nbP_b ; i++) m_params(m_nbP_x+m_nbP_a+i) = m_b->GetParameters()(i);

	return true;
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
inline void LinearCombinationPDF::_DrawNewSample() { 
	m_lastSample = (m_a->DrawSample() * m_x->DrawSample() + m_b->DrawSample() ); 
}


//
//UnivariateMixturePDF
//
UnivariateMixturePDF::UnivariateMixturePDF() : UnivariatePDF(), m_totalNbParams(0) { m_sumWeights=0; }
UnivariateMixturePDF::~UnivariateMixturePDF() {};
bool UnivariateMixturePDF::SetParameters(const vnl_vector<double> &p) {
	if (p.size()!=m_totalNbParams) return false;
	unsigned cumSumNbParams=0;
	//set the parameters of each law, in turn...
	for (unsigned i=0 ; i<m_univ_pdfs.size() ; i++) {
		if (!m_univ_pdfs[i]->SetParameters(p.extract(m_nbP[i] ,cumSumNbParams))) {
			//rewind modifications if those parameters are invalid
			for (int j=i-1 ; j>=0 ; j--) {
				cumSumNbParams -= m_nbP[j];
				if (!m_univ_pdfs[j]->SetParameters(m_params.extract(m_nbP[j] ,cumSumNbParams))) throw DeformableModelException("UnivariateMixturePDF::SetParameters : trying to set invalid parameters, and unable to rewind to previous parameters ; SHOULD NEVER HAPPEN!");
			}
			return false;
		}
		cumSumNbParams+=m_nbP[i];
	}
	m_params = p;

	return true;
}

void UnivariateMixturePDF::AddPDF(UnivariatePDF *pdf, double weight) {
	m_univ_pdfs.push_back(pdf);
	vnl_vector<double> previousParams = m_params, tmpParams = pdf->GetParameters();
	m_nbP.push_back(tmpParams.size()); m_totalNbParams+=m_nbP.back(); m_params.set_size(m_totalNbParams);
	if (!previousParams.empty()) for (unsigned i=0 ; i<previousParams.size() ; i++) m_params(i) = previousParams(i);
	for (unsigned i=0 ; i<tmpParams.size() ; i++) m_params(previousParams.size()+i) = tmpParams(i);	
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
inline void UnivariateMixturePDF::_DrawNewSample() { 
	//select the generating distribution based on the weights...
	double rnd = m_baseGenerator->GetVariateWithOpenUpperRange(m_sumWeights);
	unsigned int n_pdf;
	for (n_pdf = 0; n_pdf<m_univ_pdfs.size() ; n_pdf++) {
		if (rnd<m_weights[n_pdf]) break;
		else rnd-=m_weights[n_pdf];
	}
	m_lastSample = m_univ_pdfs[n_pdf]->DrawSample();
}
