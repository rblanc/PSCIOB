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

//seems to enable IntelliSence for VisualStudio...
#pragma once

#include <iostream>
#include <typeinfo>

#include "GeneralUtils.h"
#include "SetsUtils.h"
#include "ITKUtils.h"
#include "time.h"

#include <typeinfo>

using namespace psciob;


class Base : public itk::LightObject {
public:
	/** Standard class typedefs. */
	typedef Base							Self;
	typedef itk::LightObject				Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	typedef Self							BaseClass;
	typedef Pointer							BasePointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(Base,itk::LightObject);

	virtual void PrintData() = 0;
	virtual void PrintData2() {std::cout<<"Base::PrintData2"<<std::endl;}
	//virtual BasePointer CreateAnother() = 0;
	/*BasePointer CreateAnother(BaseClass *in) {
		
		return obj;
	}*/
	double m_data;

protected:
	Base()  { m_data = 1; std::cout<<"Base constructor - m_data = "<<m_data<<std::endl;}
	virtual ~Base() {std::cout<<"Base destructor"<<std::endl;}

private:
	Base(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};

class ChildA : public Base {
public:
	/** Standard class typedefs. */
	typedef ChildA			Self;
	typedef itk::LightObject				Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(ChildA,Base);
	itkNewMacro(Self);

	void PrintData() { std::cout<<"ChildA::Print -> m_data = "<<m_data<<std::endl; }
	/*BasePointer CreateAnother() { 
		std::cout<<"ChildA::CreateAnother"<<std::endl; 
		BasePointer obj = New();
		return obj;
	}*/
protected:
	ChildA() : Base() {
		m_data += 1;
		std::cout<<"ChildA constructor - m_data = "<<m_data<<std::endl;
	};
	~ChildA() { std::cout<<"ChildA destructor"<<std::endl; }

private:
	ChildA(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};

class ChildB : public Base {
public:
	/** Standard class typedefs. */
	typedef ChildB			Self;
	typedef itk::LightObject				Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(ChildB,Base);
	itkNewMacro(Self);

	void PrintData() { std::cout<<"ChildB::Print -> m_data = "<<m_data<<std::endl; }
	virtual void PrintData2() {std::cout<<"ChildB::PrintData2"<<std::endl;}

	/*BasePointer CreateAnother() { 
		std::cout<<"ChildB::CreateAnother"<<std::endl; 
		BasePointer obj = New();
		return obj;
	}*/

protected:
	ChildB() : Base() {
		m_data += 5;
		std::cout<<"ChildB constructor - m_data = "<<m_data<<std::endl;
	};
	~ChildB() { std::cout<<"ChildB destructor"<<std::endl; }

private:
	ChildB(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );	//purposely not implemented
};


class TOTO : public itk::LightObject {
public:
	static const unsigned int toto_i=0;

	enum TOTOCODE {	//different codes indicating how went the drawing of the object.
		C1,			// 0 
		C2,			// 1: complete success, no overlap, object fully inside the scene...
	};
	/** Standard class typedefs. */
	typedef TOTO			Self;
	typedef itk::LightObject				Superclass;
	typedef itk::SmartPointer<Self>			Pointer;
	typedef itk::SmartPointer<const Self>	ConstPointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(TOTO,itk::LightObject);
	itkSimpleNewMacro(Self);	//This is necessary to overload CreateAnother

	void SetTutu(Base *ptr) {tutu = ptr;}
	Base * GetTutu() {
		if (!tutu) { std::cout<<"tutu is not defined..."<<std::endl; }
		return tutu.GetPointer();
	}

	virtual ::itk::LightObject::Pointer CreateAnother(void) const {
		Pointer copyPtr = Self::New().GetPointer();	
		copyPtr->tutu = static_cast<Base*>(tutu->CreateAnother().GetPointer());
		return static_cast<Self*>( copyPtr );
	}

protected:
	TOTO()  { tutu=NULL; std::cout<<"TOTO constructor "<<std::endl;}
	virtual ~TOTO() {std::cout<<"TOTO destructor"<<std::endl;}
	Base::Pointer tutu;
};

void Test1() {
	std::cout<<"\n\nTEST 1\n\n"<<std::endl;
	clock_t t0 = clock();

	std::cout<<"\ncreating child A:"<<std::endl;
	ChildA::Pointer a = ChildA::New();
	a->PrintData();
	a->m_data = 15;
	a->PrintData();


	std::cout<<"\ncreating child B:"<<std::endl;
	ChildB::Pointer b = ChildB::New();
	b->PrintData();
	b->m_data = 18;
	b->PrintData();

	std::cout<<"\ncreating an instance of TOTO"<<std::endl;
	TOTO::Pointer toto = TOTO::New();
	toto->SetTutu(a);
	std::cout<<"print tutu of toto: ";toto->GetTutu()->PrintData();

	std::cout<<"toto_i = "<<TOTO::toto_i<<std::endl;

	std::cout<<"\ncreating another TOTO"<<std::endl;
	TOTO::Pointer toto2 = static_cast<TOTO*>(toto->CreateAnother().GetPointer());
	//TOTO::Pointer toto2 = TOTO::New();
	std::cout<<"print tutu of toto: ";
	if (!toto2->GetTutu()) std::cout<<" UNDEFINED"<<std::endl;
	else toto2->GetTutu()->PrintData();

	std::cout<<"modifying a..., then checking a, toto and toto2 ..."<<std::endl;
	a->m_data += 5;
	a->PrintData();
	toto->GetTutu()->PrintData();	//OK: toto remains tied with a (changing a modifies toto::tutu)
	toto2->GetTutu()->PrintData();	//OK: toto2 is an independant copy, not tied to a in any way (doesn't even copy the values in this implementation.)
	
	std::cout<<"typeid name of toto: "<<typeid(*toto).name()<<std::endl;

	//std::cout<<"\ncasting childA to BaseClass:"<<std::endl;
	//Base::Pointer ptra = a;
	//ptra->PrintData();

	//std::cout<<"\ncasting childB to BaseClass:"<<std::endl;
	//Base::Pointer ptrb = b;
	//ptrb->PrintData();

	//std::cout<<"\ncreate another from ptra"<<std::endl;
	////Base::Pointer ptra2 = (Base*)(ptra->CreateAnother().GetPointer());
	//Base::Pointer ptra2 = static_cast<Base*>(ptra->CreateAnother().GetPointer());
	//ptra2->PrintData();

	//std::cout<<"\ncreate another from ptrb"<<std::endl;
	//Base::Pointer ptrb2 = static_cast<Base*>(ptrb->CreateAnother().GetPointer());
	//ptrb2->PrintData();

	//std::cout<<"typeid(*a) == typeid(*b)? "<< (typeid(*a)==typeid(*b)) <<std::endl;
	//std::cout<<"typeid(*a) == typeid(*ptra)? "<< (typeid(*a)==typeid(*ptra)) <<std::endl;
	//std::cout<<"typeid(*b) == typeid(*ptrb)? "<< (typeid(*b)==typeid(*ptrb)) <<std::endl;
	//std::cout<<"typeid(*ptra) == typeid(*ptrb)? "<< (typeid(*ptra)==typeid(*ptrb)) <<std::endl;
	//std::cout<<"typeid(*ptra) == typeid(*ptra2)? "<< (typeid(*ptra)==typeid(*ptra2)) <<std::endl;

}

void Test2() {
	std::cout<<"\n\nTEST 2\n\n"<<std::endl;

	std::vector<Base::Pointer> vect;
	ChildA::Pointer a = ChildA::New();
	ChildB::Pointer b = ChildB::New();
	ChildA::Pointer c = ChildA::New();
	ChildA::Pointer empty;

	std::cout<<"\nnb of references of a: "<<a->GetReferenceCount()<<std::endl;
	std::cout<<"nb of references of b: "<<b->GetReferenceCount()<<std::endl;
	std::cout<<"nb of references of c: "<<c->GetReferenceCount()<<std::endl;

	vect.push_back( a.GetPointer() );
	vect.push_back( b.GetPointer() );

	std::cout<<"\nnb of references of a: "<<a->GetReferenceCount()<<std::endl;
	std::cout<<"nb of references of b: "<<b->GetReferenceCount()<<std::endl;
	std::cout<<"nb of references of c: "<<c->GetReferenceCount()<<std::endl;

	vect[1] = c.GetPointer();

	std::cout<<"\nnb of references of a: "<<a->GetReferenceCount()<<std::endl;
	std::cout<<"nb of references of b: "<<b->GetReferenceCount()<<std::endl;
	std::cout<<"nb of references of c: "<<c->GetReferenceCount()<<std::endl;

	vect.push_back( empty.GetPointer() );
	vect.push_back( empty.GetPointer() );
	vect.push_back( empty.GetPointer() );
	vect.push_back( empty.GetPointer() );

	vect[2] = c.GetPointer();

	std::cout<<"\nnb of references of a: "<<a->GetReferenceCount()<<std::endl;
	std::cout<<"nb of references of b: "<<b->GetReferenceCount()<<std::endl;
	std::cout<<"nb of references of c: "<<c->GetReferenceCount()<<std::endl;

}

void Test3() {
	std::cout<<"\n\nTEST 3\n\n"<<std::endl;


	std::vector<Base::Pointer> vect;
	ChildA::Pointer a = ChildA::New();
	ChildB::Pointer b = ChildB::New();
	ChildA::Pointer c = ChildA::New();
	ChildA::Pointer empty;

	vect.push_back( a.GetPointer() );
	vect.push_back( b.GetPointer() );

	std::cout<<"Printing from a: "; vect[0]->PrintData2(); // -> uses the method from the base class
	std::cout<<"Printing from b: "; vect[1]->PrintData2(); // -> uses the method from the Child class ! yes!
}

int main(int argc, char** argv) {

	Test1();

	Test2();

	Test3();

	return 1;
}
