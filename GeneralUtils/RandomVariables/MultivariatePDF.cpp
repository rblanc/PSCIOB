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
 * \file MultivariatePDF.cpp
 * \author Rémi Blanc 
 * \date 27. February 2012
*/

#include "MultivariatePDF.h"
	
using namespace psciob;

//
//MultivariatePDF base class
//
MultivariatePDF::MultivariatePDF() : PDFBase() {};
MultivariatePDF::~MultivariatePDF() {};
unsigned int MultivariatePDF::GetNumberOfDimensions() {return m_dimension;}

//
//MVN_PDF
//

MVN_PDF::MVN_PDF() : MultivariatePDF() { SetNumberOfDimensions(2); }
MVN_PDF::~MVN_PDF() {};


void MVN_PDF::SetNumberOfDimensions(unsigned int n) {
	m_dimension=n;
	m_mean.set_size(m_dimension,1); m_mean.fill(0);
	m_cov.set_size(m_dimension,m_dimension); m_cov.set_identity();
	m_std = m_cov;m_invcov=m_cov;m_det=1;
}


void MVN_PDF::SetParameters(const vnl_vector<double> &mean, const vnl_matrix<double> &cov) {
	if ( (mean.size()!=cov.cols()) || (mean.size()!=cov.rows()) ) throw DeformableModelException("Exception in MVN_PDF::SetParameters() : inconsistent inputs");
	if (mean.size() != m_dimension) { SetNumberOfDimensions(mean.size()); }

	vnl_symmetric_eigensystem<double> eig(cov); 
	m_std = eig.square_root();	
	m_invcov = eig.pinverse();	
	m_det = eig.determinant();

	m_mean.set_column(0, mean); m_cov = cov;
	m_normalizationFactor = pow(2.0*PI, m_dimension/2.0) * sqrt(m_det);
}


void MVN_PDF::SetParameters(double mean, double cov) {
	m_mean.fill(mean); m_cov.set_identity();	m_cov*=cov;
	vnl_symmetric_eigensystem<double> eig(m_cov); 
	m_std = eig.square_root();	m_invcov = eig.pinverse();	m_det = eig.determinant();
	m_normalizationFactor = pow(2.0*PI, m_dimension/2.0) * sqrt(m_det);
}


inline double MVN_PDF::GetLikelihood(const vnl_vector<double> &x) {
	if ( x.size()!=m_dimension ) throw DeformableModelException("Exception in MVN_PDF::Evaluate() : input of wrong dimension");
	vnl_matrix<double> input(x.size(),1); input.set_column(0, x);
	vnl_matrix<double> tmp_mat; tmp_mat = (input-m_mean).transpose()*m_invcov*(input-m_mean); 
	return ( exp(-0.5*tmp_mat(0,0))  / m_normalizationFactor );	
}

inline double MVN_PDF::GetMaximumLikelihoodValue() {
	return ( 1.0 / m_normalizationFactor );	
}


inline double MVN_PDF::GetLogLikelihood(const vnl_vector<double> &x) {
	if ( x.size()!=m_dimension ) throw DeformableModelException("Exception in MVN_PDF::Evaluate() : input of wrong dimension");
	vnl_matrix<double> input(x.size(),1); input.set_column(0, x);
	vnl_matrix<double> tmp_mat; tmp_mat = (input-m_mean).transpose()*m_invcov*(input-m_mean); 
	return ( (-0.5*tmp_mat(0,0)) - log(2.0*PI))*m_dimension/2.0 + log(m_det)/2.0;		//OPTIMIZATION: cache these values...
}	


inline vnl_vector<double> MVN_PDF::_DrawNewSample() { 
	vnl_matrix<double> Z(m_dimension, 1);
	for (unsigned i=0 ; i<m_dimension ; i++) Z(i,0) = m_baseGenerator->GetNormalVariate();
	return ((m_mean + m_std*Z).get_column(0));
}


//
//IndependentPDFs
//

IndependentPDFs::IndependentPDFs() : MultivariatePDF() {m_dimension = 0;}
IndependentPDFs::~IndependentPDFs() {};


void IndependentPDFs::AddUnivariatePDF(UnivariatePDF *pdf) {
	m_dimension++;
	m_univ_multiv_law.push_back(false);
	m_dim_laws.push_back(1);
	m_univ_pdfs.push_back(pdf);
}


void IndependentPDFs::AddMultivariatePDF(MultivariatePDF *pdf) {
	m_dimension+=pdf->GetNumberOfDimensions();
	m_univ_multiv_law.push_back(true);
	m_dim_laws.push_back(pdf->GetNumberOfDimensions());
	m_multiv_pdfs.push_back(pdf);
}

inline double IndependentPDFs::GetLikelihood(const vnl_vector<double> &x) {
	if ( x.size()!=m_dimension ) throw DeformableModelException("Exception in IndependentPDFs::GetLikelihood() : input of wrong dimension");

	double likelihood = 1;

	unsigned int ind_in_x=0, ind_univ=0, ind_multiv=0; //indices of the current univariate and multivariate laws 
	for (unsigned i=0 ; i<m_dim_laws.size() ; i++) {
		if (!m_univ_multiv_law[i]) {
			likelihood *= m_univ_pdfs[ind_univ]->GetLikelihood(x(ind_in_x));
			ind_in_x++;
			ind_univ++;
		}
		else {
			likelihood *= m_multiv_pdfs[ind_multiv]->GetLikelihood(x.extract(m_dim_laws[i], ind_in_x));
			ind_in_x+=m_dim_laws[i];
			ind_multiv++;
		}
	}
	return likelihood;
}

inline double IndependentPDFs::GetMaximumLikelihoodValue() {
	double likelihood = 1;
	unsigned int ind_univ=0, ind_multiv=0; //indices of the current univariate and multivariate laws 
	for (unsigned i=0 ; i<m_dim_laws.size() ; i++) {
		if (!m_univ_multiv_law[i]) {
			likelihood *= m_univ_pdfs[ind_univ]->GetMaximumLikelihoodValue();
			ind_univ++;
		}
		else {
			likelihood *= m_multiv_pdfs[ind_multiv]->GetMaximumLikelihoodValue();
			ind_multiv++;
		}
	}
	return likelihood;
}


inline double IndependentPDFs::GetLogLikelihood(const vnl_vector<double> &x) { 
	if ( x.size()!=m_dimension ) throw DeformableModelException("Exception in IndependentPDFs::GetLogLikelihood() : input of wrong dimension");
	double loglikelihood=0;		
	unsigned int ind_in_x=0, ind_univ=0, ind_multiv=0; //indices of the current univariate and multivariate laws 
	for (unsigned i=0 ; i<m_dim_laws.size() ; i++) {
		if (!m_univ_multiv_law[i]) {
			loglikelihood += m_univ_pdfs[ind_univ]->GetLogLikelihood(x(ind_in_x));
			ind_univ++;		ind_in_x++;				
		}
		else {
			loglikelihood += m_multiv_pdfs[ind_multiv]->GetLogLikelihood(x.extract(m_dim_laws[i], ind_in_x));
			ind_multiv++;	ind_in_x+=m_dim_laws[i];
		}
	}
	return loglikelihood;
}


inline vnl_vector<double> IndependentPDFs::_DrawNewSample() { 
	vnl_vector<double> sample(m_dimension);

	unsigned int ind_in_sample=0, ind_univ=0, ind_multiv=0; //indices of the current univariate and multivariate laws 

	for (unsigned i=0 ; i<m_dim_laws.size() ; i++) {
		if (!m_univ_multiv_law[i]) {
			sample(ind_in_sample) = m_univ_pdfs[ind_univ]->DrawSample();
			ind_univ++;ind_in_sample++;
		}
		else {
			vnl_vector<double> tmp; tmp = m_multiv_pdfs[ind_multiv]->DrawSample();
			for (unsigned j=0 ; j<tmp.size() ; j++) {sample(ind_in_sample)=tmp(j);ind_in_sample++;}
			ind_multiv++;
		}
	}
	return sample;
}


//
//UniformBoxPDF
//
UniformBoxPDF::UniformBoxPDF() : MultivariatePDF() {m_dimension = 0;}
UniformBoxPDF::~UniformBoxPDF() {};
void UniformBoxPDF::SetBox(const vnl_vector<double> &bbox) {
	if ( (int)(bbox.size()/2) != bbox.size()/2.0 ) throw DeformableModelException("UniformBoxPDF : dimension of the vector must be pair: [min_d1 max_d1 min_d2 max_d2...]");
	m_dimension = bbox.size()/2;

	m_pdf = IndependentPDFs::New(); 
	for (unsigned i=0 ; i<m_dimension ; i++) {
		UniformPDF::Pointer tmp = UniformPDF::New(); tmp->SetParameters(bbox(2*i), bbox(2*i+1));		
		tmp->SetGeneratorSeed(i);
		m_pdf->AddUnivariatePDF(tmp);
	}
}
inline double UniformBoxPDF::GetLikelihood(const vnl_vector<double> &x) { 
	return m_pdf->GetLikelihood(x); 
}
inline double UniformBoxPDF::GetMaximumLikelihoodValue() {
	return m_pdf->GetMaximumLikelihoodValue();	
}
inline double UniformBoxPDF::GetLogLikelihood(const vnl_vector<double> &x) { 
	return m_pdf->GetLogLikelihood(x); 
}
inline vnl_vector<double> UniformBoxPDF::_DrawNewSample() { 
	return m_pdf->DrawSample(); 
}


//
//IndependentEulerRotationsPDFs
//
IndependentEulerRotationsPDFs::IndependentEulerRotationsPDFs() : MultivariatePDF() {
	std::cout<<"WARNING: IndependentEulerRotationsPDFs::GetLikelihood and GetLogLikelihood are not implemented and produce invalid results (flat)\n --> check the code for implementation ideas"<<std::endl;
	//LEARN an approximation of the distribution once the parameters are set?
	//-> 3D distribution, generate a few thousand samples, and get an histogram
	//this principle could be made somewhat generic, as it could be useful for other distributions as well...
	m_dimension = 3; m_rotMat.set_size(3,3);
}
IndependentEulerRotationsPDFs::~IndependentEulerRotationsPDFs() {};

void IndependentEulerRotationsPDFs::AddIndependentRotationPDF(ROTATIONAROUND dir, UnivariatePDF *pdf) {
	m_rotation_axis.push_back(dir);
	m_univ_pdfs.push_back(pdf);
}

inline double IndependentEulerRotationsPDFs::GetLikelihood(const vnl_vector<double> &x) {
	//throw DeformableModelException("Exception in IndependentEulerRotationsPDFs::GetLikelihood() : not implemented... "); 
	if ( x.size()!=m_dimension ) throw DeformableModelException("Exception in IndependentEulerRotationsPDFs::GetLikelihood() : input of wrong dimension");
	double likelihood=1;	
	return likelihood;
}
inline double IndependentEulerRotationsPDFs::GetMaximumLikelihoodValue() {
	//throw DeformableModelException("Exception in IndependentEulerRotationsPDFs::GetMaximumLikelihoodValue() : not implemented... "); 
	std::cout<<"WARNING: IndependentEulerRotationsPDFs::GetMaximumLikelihoodValue() is not properly implemented"<<std::endl;
	double likelihood=1;	
	return likelihood;
}
inline double IndependentEulerRotationsPDFs::GetLogLikelihood(const vnl_vector<double> &x) { 
	//throw DeformableModelException("Exception in IndependentEulerRotationsPDFs::GetLogLikelihood() : not implemented... ");
	if ( x.size()!=m_dimension ) throw DeformableModelException("Exception in IndependentEulerRotationsPDFs::GetLogLikelihood() : input of wrong dimension");
	double loglikelihood=0;		
	//for (unsigned i=0 ; i<m_univ_pdfs.size() ; i++) { loglikelihood += m_univ_pdfs[i]->GetLogLikelihood(x(i)); }
	return loglikelihood;
}


inline vnl_vector<double> IndependentEulerRotationsPDFs::_DrawNewSample() { 
	m_rotMat.set_identity();

	for (unsigned i=0 ; i<m_rotation_axis.size() ; i++) {
		double angle = m_univ_pdfs[i]->DrawSample();
		switch(m_rotation_axis[i]) {
				case X_AXIS: Rotate3DTransformAroundX_InPlace(m_rotMat, angle ); break;
				case Y_AXIS: Rotate3DTransformAroundY_InPlace(m_rotMat, angle ); break;
				case Z_AXIS: Rotate3DTransformAroundZ_InPlace(m_rotMat, angle ); break;
		}
	}
	return GetEulerAnglesFrom3DRotationMatrix(m_rotMat);
}


//
//MultivariateMixturePDF
//

MultivariateMixturePDF::MultivariateMixturePDF() : MultivariatePDF() { m_sumWeights=0; }
MultivariateMixturePDF::~MultivariateMixturePDF() {};

void MultivariateMixturePDF::AddPDF(MultivariatePDF *pdf, double weight) {
	if (m_multiv_pdfs.empty()) { m_dimension = pdf->GetNumberOfDimensions(); }
	else {  if (m_dimension != pdf->GetNumberOfDimensions()) throw DeformableModelException("MultivariateMixturePDF::AddPDF: input PDF has an inconsistent number of dimensions with previously set components"); }
	m_multiv_pdfs.push_back(pdf);
	m_weights.push_back(weight);
	m_sumWeights += weight;
}

double MultivariateMixturePDF::GetLikelihood(const vnl_vector<double> &x) { 
	double likelihood=0, p_pdf, likelihood_pdf;
	for (unsigned i = 0 ; i<m_multiv_pdfs.size() ; i++) {
		p_pdf = m_weights[i]/m_sumWeights;
		likelihood_pdf = m_multiv_pdfs[i]->GetLikelihood(x);
		likelihood += p_pdf * likelihood_pdf;
	}
	return likelihood;
}

inline double MultivariateMixturePDF::GetMaximumLikelihoodValue() {
	//throw DeformableModelException("Exception in MultivariateMixturePDF::GetMaximumLikelihoodValue() : not implemented... "); 
	//if ( x.size()!=m_dimension ) throw DeformableModelException("Exception in IndependentEulerRotationsPDFs::GetMaximumLikelihoodValue() : input of wrong dimension");
	std::cout<<"WARNING: MultivariateMixturePDF::GetMaximumLikelihoodValue() is not properly implemented"<<std::endl;
	double likelihood=1;	
	return likelihood;
}

inline double MultivariateMixturePDF::GetLogLikelihood(const vnl_vector<double> &x) { 
	double tmp = GetLikelihood(x);
	if (tmp==0) return MINUSINF;
	return log(tmp); 
}


inline vnl_vector<double> MultivariateMixturePDF::_DrawNewSample() { 
	//select the generating distribution based on the weights...
	double rnd = m_baseGenerator->GetVariateWithOpenUpperRange(m_sumWeights);
	unsigned int n_pdf;
	for (n_pdf = 0; n_pdf<m_multiv_pdfs.size() ; n_pdf++) {
		if (rnd<m_weights[n_pdf]) break;
		else rnd-=m_weights[n_pdf];
	}
	return m_multiv_pdfs[n_pdf]->DrawSample();
}
