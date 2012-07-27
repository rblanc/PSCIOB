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
 * \file SetsUtils.hpp
 * \author Rémi Blanc 
 * \date 29. August 2011
*/

#ifndef SETSUTILS_H_
#define SETSUTILS_H_


#include <set>
#include <map>
#include <stack>
#include <vector>

namespace psciob {

/** Test if two std::sets share at least one sample 
 *  inspired from the description of std::set_intersection at http://www.cplusplus.com/reference/algorithm/set_intersection/
 */
template <class InputIterator1, class InputIterator2>
bool
TestSetIntersection( InputIterator1 first1, InputIterator1 last1, InputIterator2 first2, InputIterator2 last2);



/**\class SetOfMutuallyRelatedLabels
* \class SetOfMutuallyRelatedLabels: manages a set of mutually related objects
* Each object is uniquely identified with an ID (actually a non-zero integer). The ID is automatically provided by the AddObject method.
* Each object is associated with a list (std::set) of other objects with which it is in relations
* In this container, the relations are always bi-lateral. When an object is removed from the container, the interactions of all other objects are updated to reflect this modification.
*
* The container is optimized for speed rather than for memory ... 
*/
template <typename ObjectIDType>
class SetOfMutuallyRelatedLabels {
public:

	typedef std::set<ObjectIDType> RelatedIDSetType;

	struct ObjectEntryType {
		ObjectIDType id;
		typename RelatedIDSetType relatedIDs;
	};

	SetOfMutuallyRelatedLabels();
	~SetOfMutuallyRelatedLabels();

	/** Adds an object to the container, returns the unique identifier that has been attributed */
	ObjectIDType AddLabel(long label);
	
	/** Removes the specified object from the container, updating the other objects if they were in relation. 
	 * Returns false if the given id is unused 
	*/
	bool RemoveLabel(ObjectIDType id);


	/** Indicate Pairwise relation - returns -1 if the ids are incorrect, 0 if the relation was already known, and 1 if the relation is newly registered */
	int RelateLabels(ObjectIDType id1, ObjectIDType id2);
	
	/** Unrelate two objects - return true if successful, and false if the relations did not exist */
	bool UnrelateLabels(ObjectIDType id1, ObjectIDType id2);

	/** Get number of objects in the container */
	unsigned GetNumberOfObjects();

	/** Get the entry for the requested object */
	ObjectEntryType* GetEntry(ObjectIDType id) const;

private:	
	unsigned int m_maxIDReached, m_nbObjects;
	std::vector<ObjectEntryType> m_arrayLabels;
	std::stack<ObjectIDType> m_stackFreedIDs;

};

/**\class SetMutuallyRelatedObjects
* \class SetMutuallyRelatedObjects: manages a set of mutually related objects
*
* Each object is:
* 1/ uniquely identified with an ID (actually a non-zero integer). The ID is automatically provided by the AddObject method.
* 2/ a data container describing this object
* 3/ a list (std::map) of other objects with which it is in relations and the associated relation data
* these container should have default copy constructors.
* \todo check whether a shared pointer can be used to avoid duplicating this data...
* In this container, the relations are always bi-lateral. When an object is removed from the container, the interactions of all other objects are updated to reflect this modification.
*
* The container is optimized for speed rather than for memory ... 
*
*/
template <typename ObjectIDType, typename ObjectDataType, typename RelationDataType>
class SetMutuallyRelatedObjects {
public:

	typedef std::map<ObjectIDType, RelationDataType> RelatedIDMapType;
	typedef std::pair<ObjectIDType, RelationDataType> RelationPair;

	struct ObjectEntryType {
		typename ObjectDataType data;
		ObjectIDType id;
		typename RelatedIDMapType relatedIDs;
	};

	/** Default Constructor */
	SetMutuallyRelatedObjects();
	/** Default Destructor */
	virtual ~SetMutuallyRelatedObjects();

	/** Clears all the contents, and Add the requested number of empty entries */
	void SetSize(unsigned size);

	/** Adds an object to the container, returns the unique identifier that has been attributed */
	ObjectIDType AddObject(ObjectDataType data);

	/** Adds an object to the container, with a prefered id value
	* if this value already exists, one is automatically attributed
	*/
	ObjectIDType AddObject(ObjectDataType data, ObjectIDType preferedId);

	/** Removes the specified object from the container, updating the other objects if they were in relation. 
	 * Returns false if the given id is unused 
	*/
	bool RemoveObject(ObjectIDType id);
	
	/** Indicate Pairwise relation - returns -1 if the ids are incorrect (or if they are identical), 0 if the relation was already known, and 1 if the relation is newly registered */
	int RelateObjects(ObjectIDType id1, ObjectIDType id2, RelationDataType rData = RelationDataType());
	
	/** Unrelate two objects - return true if successful, and false if the relations did not exist */
	bool UnrelateObjects (ObjectIDType id1, ObjectIDType id2);

	/** Get number of objects in the container */
	unsigned GetNumberOfObjects();

	/** Get the entry for the requested object */
	inline ObjectEntryType* GetEntry(ObjectIDType id);

	///** Get the entry for the requested object */
	//inline bool GetDataOfEntry(ObjectIDType id, ObjectDataType &data) { 
	//	if ( m_arrayObjects[id-1].id == 0 ) return false;
	//	data = 
	//	return (&(m_arrayObjects[id-1].data));
	//}

	///** Get the entry for the requested object */
	//inline RelationDataType* GetRelationDataOfEntries(ObjectIDType id1, ObjectIDType id2) { 
	//	//throw an exception if id is invalid (out of bounds)
	//	return (&(m_arrayObjects[id1-1].relatedIDs.find(id2)->second));
	//}

protected:
	
	unsigned int m_maxIDReached, m_nbObjects;
	std::vector<ObjectEntryType> m_arrayObjects;
	std::stack<ObjectIDType> m_stackFreedIDs;

};

} // namespace psciob

#include "SetsUtils.txx"

#endif //SETSUTILS_H_