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

/*
 * GeneralUtils.h
 * \author Rémi Blanc 
 * \date 29. August 2011
*/

#ifndef GENERALUTILS_H_
#define GENERALUTILS_H_

#include <math.h> 

#include <iostream>
#include <sstream>
#include <exception>
#include <string>

#include <vnl/vnl_vector.h>
#include <vnl/vnl_inverse.h>
#include "itkLightObject.h"

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>

#include <set>
#include <map>
#include <list>

#include <limits>

namespace psciob {

static const double TINY    = 1e-8;
static const double PI		= 3.1415926535897932384626433832795;
static const double SQRT2PI	= 2.506628274631000502415765284811;
static const double INF		= 1e30;
static const double MINUSINF=-1e30;

/** \class DeformableModelException
* \brief DeformableModelException class managing exceptions in the set of libraries
* \todo rename it when the final name is found
*/
class DeformableModelException : public std::exception {
public:
	DeformableModelException(std::string message) : m_message(message) {}
	DeformableModelException(const char* message) : m_message(message) {}
	virtual ~DeformableModelException() throw() {}
	const char* what() const throw() { return m_message.c_str(); }

private:
	std::string m_message;
};



inline long round(double r) {
	return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}

template <class T>
inline T max (T a, T b)  { return (a > b ? a : b) ; }

template <class T>
inline T min (T a, T b)  { return (a < b ? a : b) ; }

template <class T1, class T2>
inline T1 min (T1 a, T2 b)  { return (a < b ? a : b) ; }


inline std::string stringify(int x) {
	std::ostringstream o;
	o<<x;
	return o.str();
}

/** Cast to a different type ; in case this new type has a narrower dynamic, the value is set to the minimum/maximum possible value*/
template <class T1, class T2>
inline T2 saturate_cast(T1 a) {
	if (std::numeric_limits<T2>::is_signed) return static_cast<T2>( std::max<T1>(std::min<T1>(a, std::numeric_limits<T2>::max()), -std::numeric_limits<T2>::max()) );
	else return static_cast<T2>( std::max<T1>(std::min<T1>(a, std::numeric_limits<T2>::max()), std::numeric_limits<T2>::min()) );
}



/** \class index_less
* \brief index_less operator designed to get the indices that sort an input array
* typical use: 
* std::vector<int> a;					a.push_back(5); a.push_back(8); a.push_back(7);
* std::vector<unsigned int> ind_a;		for (unsigned i = 0; i < a.size(); ++i) { ind_a.push_back(i); }			<= ind_a = 0 1 2
* sort(ind_a.begin(), ind_a.end(), index_less<std::vector<int>&>(a));											<= ind_a = 0 2 1
*/
template<class T> struct index_less { 
	index_less(T _arr) : arr(_arr) {}
	bool operator()(const size_t a, const size_t b) const	{ return arr[a] < arr[b]; }
	T arr;
};
/** \class index_more
* \brief index_more operator designed to get the indices that sort an input array
* typical use: 
* std::vector<int> a;					a.push_back(5); a.push_back(8); a.push_back(7);
* std::vector<unsigned int> ind_a;		for (unsigned i = 0; i < a.size(); ++i) { ind_a.push_back(i); }			<= ind_a = 0 1 2
* sort(ind_a.begin(), ind_a.end(), index_more<std::vector<int>&>(a));											<= ind_a = 1 2 0
*/
template<class T> struct index_more { 
	index_more(T _arr) : arr(_arr) {}
	bool operator()(const size_t a, const size_t b) const	{ return arr[a] > arr[b]; }
	T arr;
};



/** \class key_iterator
* \brief key_iterator helper classes to iterate directly on the map keys -> enables using e.g. std::set_difference( key_begin(mymap), key_end(mymap), myset.begin(), myset.end(), result_vec.begin() );
* taken from: http://stackoverflow.com/questions/110157/how-to-retrieve-all-keys-or-values-from-a-stdmap
*/
template<class map_type>
class key_iterator : public map_type::iterator
{
public:
    typedef typename map_type::iterator map_iterator;
    typedef typename map_iterator::value_type::first_type key_type;
    key_iterator(const map_iterator& other) : map_type::iterator(other) {} ;
    key_type& operator *() { return map_type::iterator::operator*().first; }
};

/** helpers to create map iterators more easily
* \sa key_iterator 
*/
template<class map_type>
key_iterator<map_type> key_begin(map_type& m)	{ return key_iterator<map_type>(m.begin());}
/** helpers to create iterators more easily
* \sa key_iterator 
*/
template<class map_type>
key_iterator<map_type> key_end(map_type& m)		{ return key_iterator<map_type>(m.end()); }

/** Write a vnl_vector< scalar > to a txt file, readable e.g. by matlab */
template <class T>
void
WriteVNLVectorToTxtFile(std::string name, vnl_vector<T> m) {
	std::ofstream file_out;
	file_out.open(name.c_str());
	file_out << m << std::endl;
	file_out.close();
}

/** Write a vnl_matrix< scalar > to a txt file, readable e.g. by matlab */
template <class T>
void
WriteVNLMatrixToTxtFile(std::string name, vnl_matrix<T> m) {
	std::ofstream file_out;
	file_out.open(name.c_str());
	file_out << m << std::endl;
	file_out.close();
}

/** Read a txt file (space-separated) into a vnl_vector */
template <class T> 
void 
ReadVNLVectorFromTxtFile(std::string name, vnl_vector<T> *v) {
	ifstream file;
	file.open(name.c_str());
	file.clear(); 
	file.seekg(0);	
	(*v).read_ascii(file); 
	file.close(); 
}

/** Read a txt file (space-separated) into a vnl_matrix */
template <class T>
void 
ReadVNLMatrixFromTxtFile(std::string name, vnl_matrix<T> *m) { 
	ifstream file;	
	file.open(name.c_str()); 
	file.clear(); 
	file.seekg(0);	
	(*m).read_ascii(file);
	file.close(); 
}

} // namespace psciob

#endif //GENERALUTILS_H_