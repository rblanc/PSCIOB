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
 * \file ObjectPixelSet.h
 * \author Rémi Blanc 
 * \date 25. July 2011
 *
 */

#ifndef OBJECTPIXELSET_H_
#define OBJECTPIXELSET_H_


/** 
 * \class ObjectPixelSet
 * \brief ObjectPixelSet: storage class that hosts a set of pixels coordinate, as well as 
 *        information about the underlying 'image', i.e. origin, size and spacing
 *
 * IMPORTANT (DEV): though not using an ordered container, the flat indices in the set MUST be ordered
 * experiments indicated that their is a significant overhead using a ordered contained such as std::set
 * over an std::vector, even when indicating the right position for insertion in the set...
 *
 */

#include "GeneralUtils.h"
#include "ITKUtils.h"

namespace psciob {

//CONCRETE
class ObjectPixelSet : public itk::DataObject  {
public:
	/** Standard class typedefs. */
	typedef ObjectPixelSet					Self;
	typedef itk::DataObject					Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(ObjectPixelSet, itk::DataObject);
	itkNewMacro(ObjectPixelSet);

	typedef vnl_vector<unsigned int>	IndexType;
	typedef vnl_vector<double>			PointType;
	typedef vnl_vector<unsigned int>	SizeType;
	typedef vnl_vector<double>			SpacingType;

	//using a vector, because std::set is MUCH slower at inserting elements ; and there is no risk of inserting elements multiple times anyway
	//The elements are typically already sorted at creation (-> typical iteration over image)
	typedef std::vector<unsigned long>		SetType;
	typedef SetType::iterator				IteratorType;

	/** clears the set of pixels AND the image information */
	inline void Reset() { m_pixelSet.clear(); m_imageInfoIsSet = false; }

	/** clears the set of pixels but keeps the image information */
	void Initialize() { m_pixelSet.clear(); }

	//typename RegionType		GetImageRegion()	const	{return m_region;}	// a region contains a "size" and an "index" with respect to the associated image
	unsigned	GetSpaceDimension() const   {return m_Dimension;}
	PointType	GetImageOrigin()    const   {return m_origin;}	// origin = physical coordinates of the first pixel
	SizeType	GetImageSize()      const   {return m_size;}	// size: number of pixels in each dimension
	SpacingType	GetImageSpacing()   const   {return m_spacing;}	// spacing: physical dimension of the pixel, in each dimension
	inline SetType* GetPixelSet()   const   {return const_cast<SetType*>(&m_pixelSet);}
	
	inline unsigned int	GetNumberOfPixels() const {return m_pixelSet.size();}

	/** WARNING: it should update the pixel set if possible...
	 * for the moment, it is only called by PoseTransform::ApplyGridIntegerTranslation, so everything should be fine here.
	 * TODO: change the interface, so that it better reflects the usage...
	 */
	void SetNewImageOrigin(PointType origin) { 
		//return this->SetImageInformation(origin, m_size, m_spacing);
		m_origin = origin; 
		for (unsigned i=0 ; i<m_Dimension ; i++) {
			m_ImageBoundingBox(2*i) = m_origin[i] - m_spacing[i]/2.0;
			m_ImageBoundingBox(2*i+1) = m_ImageBoundingBox(2*i) + m_size[i]*m_spacing[i];
		}
	}

	/** Define, or redefine the image information related to the pixel set
	 * if the pixel set is already existing, and if the new image info corresponds to a grid-integer modification, it is reformulated and updated in the new reference system
	 * TRUE is return when the pixel set is kept valid.
	 * otherwise, it returns FALSE, indicating the PixelSet is invalidated
	 * WARNING: the pixel set may be cropped if the new image info does not contain it entirely, but the reconstruction is NOT performed in case the image was too small and is now enlarged
	 */
	bool SetImageInformation(PointType origin, SizeType size, SpacingType spacing) {	
		bool gridIntegerTranslation = true, nomodifs = true;
		if ((m_imageInfoIsSet) && (!m_pixelSet.empty())) {
			vnl_vector<long> diff_origin(m_Dimension), mult(m_Dimension); double tmpdiff;	
			for (unsigned i=0 ; i<m_Dimension ; i++) {
				//if spacing is different -> recompute
				if ( m_spacing[i]!=spacing[i] )	{ gridIntegerTranslation = false; break;}
				//if translation is not grid integer -> recompute
				tmpdiff = (origin[i] - m_origin[i]) / m_spacing[i];
				diff_origin(i) = round(tmpdiff);
				//WARNING: this formula works, but e.g. fmod doesn't work so well
				if ( fabs(tmpdiff - diff_origin(i)) > TINY ) { gridIntegerTranslation = false; break; }
				//otherwise -> continue				
				if ( diff_origin(i)!=0 )		{ nomodifs = false; }
				if ( m_size[i]!=size[i] )		{ nomodifs = false; }
				if (i==0) mult(i) = 1;
				else mult(i) = mult(i-1) * size[i-1];
			}

			if ( gridIntegerTranslation ) {
				if (!nomodifs) {
					bool ok;
					SetType newSet;
					for (unsigned i=0 ; i<m_pixelSet.size() ; i++) {
						m_tmpIndex = FlatIndexToIndex( m_pixelSet[i] );					
						ok=true;
						m_tmpSignedIndex[0] = m_tmpIndex(0)-diff_origin(0); 
						m_tmpFlatIndex = m_tmpSignedIndex[0];
						if ( m_tmpSignedIndex[0] < 0 )        { ok=false; }
						if ( m_tmpSignedIndex[0] >= size[0] ) { ok=false; }
						if (ok) {
							for (unsigned j=1 ; j<m_Dimension ; j++) { 
								m_tmpSignedIndex[j] = m_tmpIndex(j) - diff_origin(j); 
								if ( m_tmpSignedIndex[j] < 0 )        { ok=false; break; }
								if ( m_tmpSignedIndex[j] >= size[j] ) { ok=false; break; }
								m_tmpFlatIndex += m_tmpSignedIndex[j]*mult(j);
							}
							if (ok) newSet.push_back(m_tmpFlatIndex);
						}
					}
					m_pixelSet.clear();
					m_pixelSet = newSet;
				}
				else return true; //don't change anything if nothing needs to be changed...
			}//even if this was a gridIntegerTranslation, the new image info, bounding box, etc... must be refreshed, so continue with the following code
		}

		m_Dimension = origin.size();
		m_ImageBoundingBox.set_size(2*m_Dimension); m_tmpIndex.set_size(m_Dimension);m_tmpMult.set_size(m_Dimension); m_tmpSignedIndex.set_size(m_Dimension);
		m_origin	= origin;
		m_size		= size;
		m_spacing	= spacing;

		m_tmpMult[0] = 1; 
		for (unsigned i=0 ; i<m_Dimension-1 ; i++) { m_tmpMult[i+1] = m_tmpMult[i]*m_size[i]; }

		for (unsigned i=0 ; i<m_Dimension ; i++) {
			m_ImageBoundingBox(2*i) = m_origin[i] - m_spacing[i]/2.0;
			m_ImageBoundingBox(2*i+1) = m_ImageBoundingBox(2*i) + m_size[i]*m_spacing[i];
		}
		m_imageInfoIsSet = true;

		if (gridIntegerTranslation) return true;	//indicates that the pixelSet was correctly updated
		return false; //indicates the pixelset is no / no longer valid.
	}

	void SetPixelSet(SetType set) {
		m_pixelSet.clear();
		m_pixelSet = set;
	}


	inline unsigned long IndexToFlatIndex(const IndexType &index) {
		m_tmpFlatIndex = index[0]; 
		for (unsigned i=1 ; i<m_Dimension ; i++) { m_tmpFlatIndex += index[i]*m_tmpMult[i]; }
		return m_tmpFlatIndex;
	}

	inline IndexType FlatIndexToIndex(unsigned long flatIndex) {
		for (int i=m_Dimension-1 ; i>=0 ; i--) {
			m_tmpFlatIndex = flatIndex;
			flatIndex = m_tmpFlatIndex % m_tmpMult[i];
			m_tmpIndex[i] = (m_tmpFlatIndex - flatIndex) / m_tmpMult[i];
		}
		return m_tmpIndex;
	}

	//possibly faster...?
	inline void FlatIndexToIndex(unsigned long flatIndex, IndexType &index) {
		for (int i=m_Dimension-1 ; i>=0 ; i--) {
			m_tmpFlatIndex = flatIndex;
			flatIndex = m_tmpFlatIndex % m_tmpMult[i];
			index[i] = (m_tmpFlatIndex - flatIndex) / m_tmpMult[i];
		}
	}

	/** converts a flatindex (adress of the pixel in the buffer) to coordinates in the physical coordinate system */
	inline void FlatIndexToPhysicalCoordinates(unsigned long flatIndex, PointType &point) {
		for (int i=m_Dimension-1 ; i>=0 ; i--) {
			m_tmpFlatIndex = flatIndex;
			flatIndex = m_tmpFlatIndex % m_tmpMult[i];
			point(i) = m_origin(i) + ((m_tmpFlatIndex - flatIndex) / m_tmpMult[i]) * m_spacing(i);
		}
	}

	/** converts from pixel index (integer coordinates) to coordinates in the physical coordinate system */
	inline void IndexToPhysicalCoordinates(IndexType index, PointType &point) {
		for (int i=0 ; i<m_Dimension ; ++i) { point(i) = m_origin(i) + index(i) * m_spacing(i); }
	}

	inline vnl_vector<double> GetImageBoundingBox() { return m_ImageBoundingBox; }

	/** use with caution... only intended to be called by BinaryConvexModel */
	inline void AddPixel(unsigned long add) { m_pixelSet.push_back(add); }

	/** Asks whether image information has been set */
	inline bool GetImageInfoIsSet() {return m_imageInfoIsSet;}

	/** get the multiplier vector used for convertion - not expected for public use... */ 
	vnl_vector<unsigned long> GetMultVect() { return m_tmpMult; }

	/** print contents ... for debug purposes only... */
	void PrintData() {
		std::cout<<"image origin: "<<m_origin<<", spacing: "<<m_spacing<<", size: "<<m_size<<std::endl;
		for (unsigned i=0 ; i<m_pixelSet.size() ; i++) std::cout<<" "<<m_pixelSet[i];
		std::cout<<std::endl;
	}

protected:
	ObjectPixelSet() : m_Dimension(0), m_imageInfoIsSet(false) {};
	virtual ~ObjectPixelSet() {};

	SetType                  m_pixelSet; //OPTIMIZATION: it would probably be beneficial if the memory for the set was allocated before starting the insertion...
	unsigned int             m_Dimension;
	vnl_vector<double>       m_origin, m_spacing;
	vnl_vector<unsigned int> m_size;

	vnl_vector<int> m_tmpSignedIndex;
	IndexType m_tmpIndex;	//here to save allocation time when calling FlatIndexToIndex
	unsigned long m_tmpFlatIndex;
	vnl_vector<unsigned long> m_tmpMult;

	vnl_vector<double> m_ImageBoundingBox;

	bool m_imageInfoIsSet;

private:
	ObjectPixelSet(const Self&);			//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented

};

} // namespace psciob

#endif /* OBJECTPIXELSET_H_ */
