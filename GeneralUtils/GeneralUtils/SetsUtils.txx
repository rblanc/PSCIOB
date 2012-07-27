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
* \file SetsUtils.txx
* \author Rémi Blanc 
* \date 14. October 2011
*/


#include "SetsUtils.h"

namespace psciob {


template <class InputIterator1, class InputIterator2>
bool
TestSetIntersection( InputIterator1 first1, InputIterator1 last1, InputIterator2 first2, InputIterator2 last2) {
  while (first1!=last1 && first2!=last2)
  {
    if (*first1<*first2) ++first1;
    else if (*first2<*first1) ++first2;
    else { return true; }
  }
  return false;
}


//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
////////////////									//////////////
//					SetOfMutuallyRelatedLabels					//
////////////////									//////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////

template <typename ObjectIDType>
SetOfMutuallyRelatedLabels<ObjectIDType>::SetOfMutuallyRelatedLabels() : m_maxIDReached(0), m_nbObjects(0) { }

template <typename ObjectIDType>
SetOfMutuallyRelatedLabels<ObjectIDType>::~SetOfMutuallyRelatedLabels() { }


template <typename ObjectIDType>
ObjectIDType SetOfMutuallyRelatedLabels<ObjectIDType>::AddLabel(long label) {
	ObjectIDType id;
	if (!m_stackFreedIDs.empty()) {
		//get an old, no longer used ID
		id = m_stackFreedIDs.top();m_stackFreedIDs.pop();
		//update the corresponding object in the container
		m_arrayLabels[id-1].id = id;
	}
	else {
		//create a new ID
		id = ++m_maxIDReached; //this increments m_maxIDReached, and return the new value for assignement
		//create a new entry for this object in the container
		ObjectEntryType newEntry; newEntry.id = id;
		m_arrayLabels.push_back( newEntry );	
	}

	m_nbObjects++;
	return id;
}


template <typename ObjectIDType>	
bool SetOfMutuallyRelatedLabels<ObjectIDType>::RemoveLabel(ObjectIDType id) {
	if ( (id <=0) || (id >= m_arrayLabels.size()) ) return false;
	if ( m_arrayLabels[id-1].id == 0 ) return false;

	//look at all objects this one was interacting with, and tell them they no longer interact with it!
	for ( RelatedIDSetType::iterator it = m_arrayLabels[id-1].relatedIDs.begin() ; it!=m_arrayLabels[id-1].relatedIDs.end() ; ++it ) {
		m_arrayLabels[*it-1].relatedIDs.erase(id);
	}

	//delete this object from the container
	m_arrayLabels[id-1].id = 0;

	//push the id in the list of freed IDs
	m_stackFreedIDs.push(id);

	m_nbObjects--;
	return true;
}



template <typename ObjectIDType>	
int SetOfMutuallyRelatedLabels<ObjectIDType>::RelateLabels(ObjectIDType id1, ObjectIDType id2) {
	if ( (id1<=0) || (id2<=0) || (id1>=m_arrayLabels.size()) || (id2>=m_arrayLabels.size()) ) return -1;
	if ( m_arrayLabels[id1-1].id == 0 ) return -1;
	if ( m_arrayLabels[id2-1].id == 0 ) return -1;

	//check if the relation was already known
	std::pair<RelatedIDSetType::iterator, bool> pair = m_arrayLabels[id1-1].relatedIDs.insert( id2 );
	if ( !pair.second ) return 0;		
	m_arrayLabels[id2-1].relatedIDs.insert( id1 );

	return 1;		
}


template <typename ObjectIDType>
bool SetOfMutuallyRelatedLabels<ObjectIDType>::UnrelateLabels(ObjectIDType id1, ObjectIDType id2) {
	if ( (id1<=0) || (id2<=0) || (id1>=m_arrayLabels.size()) || (id2>=m_arrayLabels.size()) ) return false;
	if ( m_arrayLabels[id1-1].id == 0 ) return false;
	if ( m_arrayLabels[id2-1].id == 0 ) return false;

	RelatedIDSetType::iterator it = m_arrayLabels[id1-1].relatedIDs.find(id2);
	if ( it == m_arrayLabels[id1-1].relatedIDs.end() ) return false;
	m_arrayLabels[id1-1].relatedIDs.erase( it );
	if (!m_arrayLabels[id2-1].relatedIDs.erase( id1 )) return false; //TODO: throw an exception - THIS SHOULD NEVER HAPPEN!!
	return true;
}

template <typename ObjectIDType>
unsigned SetOfMutuallyRelatedLabels<ObjectIDType>::GetNumberOfObjects() {return m_nbObjects;}


template <typename ObjectIDType>
typename SetOfMutuallyRelatedLabels<ObjectIDType>::ObjectEntryType* 
SetOfMutuallyRelatedLabels<ObjectIDType>::GetEntry(ObjectIDType id) const { 
	//throw an exception if id is invalid (out of bounds)
	return const_cast<ObjectEntryType*>(&(m_arrayLabels[id-1]));
}


//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
////////////////									//////////////
//					SetMutuallyRelatedObjects					//
////////////////									//////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////


template <typename ObjectIDType, typename ObjectDataType, typename RelationDataType>
SetMutuallyRelatedObjects<ObjectIDType, ObjectDataType, RelationDataType>::SetMutuallyRelatedObjects() : m_maxIDReached(0), m_nbObjects(0) {}


template <typename ObjectIDType, typename ObjectDataType, typename RelationDataType>
SetMutuallyRelatedObjects<ObjectIDType, ObjectDataType, RelationDataType>::~SetMutuallyRelatedObjects() {}


template <typename ObjectIDType, typename ObjectDataType, typename RelationDataType>
void SetMutuallyRelatedObjects<ObjectIDType, ObjectDataType, RelationDataType>::SetSize(unsigned size) { 
	m_arrayObjects.clear(); 
	m_maxIDReached = 0;
	m_nbObjects = 0;
	ObjectDataType data;
	for (unsigned i=0 ; i<size ; i++) { AddObject(data); }
}

template <typename ObjectIDType, typename ObjectDataType, typename RelationDataType>
ObjectIDType 
SetMutuallyRelatedObjects<ObjectIDType, ObjectDataType, RelationDataType>::AddObject(ObjectDataType data) {
	ObjectIDType id=0;
	//get an old, no longer used ID if this is possible
	while ( id==0 && !m_stackFreedIDs.empty() ) {
		id = m_stackFreedIDs.top(); m_stackFreedIDs.pop();
	}
	if (id==0) {
		//create a new ID
		id = ++m_maxIDReached; //this increments m_maxIDReached, and return the new value for assignement
		//create a new entry for this object in the container
		ObjectEntryType newEntry; newEntry.data = data; newEntry.id = id;
		m_arrayObjects.push_back( newEntry );	
	}
	else {
		//update the corresponding object in the container
		m_arrayObjects[id-1].data = data; m_arrayObjects[id-1].id = id;
	}

	m_nbObjects++;
	return id;
}


template <typename ObjectIDType, typename ObjectDataType, typename RelationDataType>
ObjectIDType 
SetMutuallyRelatedObjects<ObjectIDType, ObjectDataType, RelationDataType>::AddObject(ObjectDataType data, ObjectIDType preferedId) {
	ObjectIDType id;
	if (preferedId>m_maxIDReached) {
		//add empty objects before the object of interest
		id = m_maxIDReached+1;
		ObjectDataType emptyData;
		while (id!=preferedId) { id = AddObject(emptyData); }

		//indicate these locations for prefered insertion location
		id = preferedId-1;
		while (id!=m_maxIDReached) { m_stackFreedIDs.push(id); id--; }

		//effectively add the new object
		id = preferedId;
		ObjectEntryType newEntry; newEntry.data = data; newEntry.id = id;
		m_arrayObjects.push_back( newEntry );	

		//update max reached id and number of objects
		m_maxIDReached = preferedId;
		m_nbObjects++;
	}
	else {			
		id = AddObject(data);
		////if the requested ID is already being used, find an available one the standard way
		//if ( m_arrayObjects[preferedId-1].id != 0 ) { id = AddObject(data); }
		//else { //if not, insert it here
		//	ObjectEntryType newEntry; newEntry.data = data; newEntry.id = id;
		//	m_arrayObjects[preferedId-1]
		//}
	}

	return id;
}


template <typename ObjectIDType, typename ObjectDataType, typename RelationDataType>
bool SetMutuallyRelatedObjects<ObjectIDType, ObjectDataType, RelationDataType>::RemoveObject(ObjectIDType id) {
	if ( (id <=0) || (id >= m_arrayObjects.size()) ) return false;
	if ( m_arrayObjects[id-1].id == 0 ) return false;

	//look at all objects this one was interacting with, and tell them they no longer interact with it!
	for ( RelatedIDMapType::iterator it = m_arrayObjects[id-1].relatedIDs.begin() ; it!=m_arrayObjects[id-1].relatedIDs.end() ; ++it ) {
		m_arrayObjects[it->first-1].relatedIDs.erase(id);
	}

	//delete this object from the container
	m_arrayObjects[id-1].id = 0;

	//push the id in the list of freed IDs
	m_stackFreedIDs.push(id);

	m_nbObjects--;
	return true;
}



template <typename ObjectIDType, typename ObjectDataType, typename RelationDataType>
int SetMutuallyRelatedObjects<ObjectIDType, ObjectDataType, RelationDataType>::RelateObjects(ObjectIDType id1, ObjectIDType id2, RelationDataType rData) {
	if (id1==id2) return -1;
	if ( (id1<=0) || (id2<=0) || (id1>=m_arrayObjects.size()) || (id2>=m_arrayObjects.size()) ) return -1;
	if ( m_arrayObjects[id1-1].id == 0 ) return -1;
	if ( m_arrayObjects[id2-1].id == 0 ) return -1;

	//check if the relation was already known
	std::pair<RelatedIDMapType::iterator, bool> pair = m_arrayObjects[id1-1].relatedIDs.insert( RelationPair(id2, rData)  );
	if ( !pair.second ) return 0;		
	m_arrayObjects[id2-1].relatedIDs.insert( RelationPair(id1, rData) );

	return 1;		
}


template <typename ObjectIDType, typename ObjectDataType, typename RelationDataType>
bool SetMutuallyRelatedObjects<ObjectIDType, ObjectDataType, RelationDataType>::UnrelateObjects (ObjectIDType id1, ObjectIDType id2) {
	if ( (id1<=0) || (id2<=0) || (id1>=m_arrayObjects.size()) || (id2>=m_arrayObjects.size()) ) return false;
	if ( m_arrayObjects[id1-1].id == 0 ) return false;
	if ( m_arrayObjects[id2-1].id == 0 ) return false;

	RelatedIDSetType::iterator it = m_arrayObjects[id1-1].relatedIDs.find(id2);
	if ( it == m_arrayObjects[id1-1].relatedIDs.end() ) return false;
	m_arrayObjects[id1-1].relatedIDs.erase( it );
	if (!m_arrayObjects[id2-1].relatedIDs.erase( id1 )) return false; //TODO: throw an exception - THIS SHOULD NEVER HAPPEN!!
	return true;
}

template <typename ObjectIDType, typename ObjectDataType, typename RelationDataType>
unsigned SetMutuallyRelatedObjects<ObjectIDType, ObjectDataType, RelationDataType>::GetNumberOfObjects() {return m_nbObjects;}

template <typename ObjectIDType, typename ObjectDataType, typename RelationDataType>
typename SetMutuallyRelatedObjects<ObjectIDType, ObjectDataType, RelationDataType>::ObjectEntryType* 
SetMutuallyRelatedObjects<ObjectIDType, ObjectDataType, RelationDataType>::GetEntry(ObjectIDType id) { 
	//throw an exception if id is invalid (out of bounds)
	return (&(m_arrayObjects[id-1]));
}

} // namespace psciob
