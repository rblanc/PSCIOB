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
 * \file SensorOptimizationManager.h
 * \author Rémi Blanc 
 * \date 29. September 2011
 */


#ifndef SENSOROPTIMIZATIONMANAGER_H_
#define SENSOROPTIMIZATIONMANAGER_H_


#include "OptimizationManager_Base.h"

namespace psciob {

/** \brief SensorAppearanceOptimizationManager
 * consider the optimization of the parameters of the sensor
 * -> brightness, contrast ; camera position / angle , etc...
 * -> necessitate to refine the interface of the Sensor, so that these different parameters can be get/set easily...
 * or start with the brightness & contrast stuff ; and treat camera position in a inherited / different class ...
*/

//FUTURE: Implement optimizationManager for various aspects of the sensor (noise level, etc...)

template<class TSensor>
class SensorAppearanceOptimizationManager : public OptimizationManager_Base {
public:
	/** Standard class typedefs. */
	typedef SensorAppearanceOptimizationManager	Self;
	typedef OptimizationManager_Base			Superclass;
	typedef itk::SmartPointer<Self>				Pointer;
	/** Run-time type information (and related methods). */
	itkTypeMacro(SensorAppearanceOptimizationManager,OptimizationManager_Base);
	itkNewMacro(Self);

	typedef TSensor								SensorType;

	void SetSensor(SensorType* sensor)			{ m_sensor = sensor; }

	inline unsigned int GetNumberOfParameters() { return m_sensor->GetNumberOfAppearanceParameters(); }

	inline vnl_vector<double> GetParameters()	{ return m_sensor->GetAppearanceParameters(); }
	bool SetParameters(vnl_vector<double> p)	{ return m_sensor->SetAppearanceParameters(p); }

	//CLEAN: access should be restricted to some optimizers ... use friendship?
	void SaveCurrentStateAsBest()		{ m_bestCost = this->GetValue(); m_bestKnownParameters = this->GetParameters(); }
protected:
	SensorAppearanceOptimizationManager() : OptimizationManager_Base() {};
	~SensorAppearanceOptimizationManager() {};

	typename SensorType::Pointer m_sensor;

	vnl_vector<double> m_bestKnownParameters;
	double ReturnToBestKnownState()		{ this->SetParameters(m_bestKnownParameters); return this->GetValue(); }

private:
	SensorAppearanceOptimizationManager(const Self&);		//purposely not implemented
	const Self & operator=( const Self & );		//purposely not implemented
};



} // namespace psciob

#endif /* SENSOROPTIMIZATIONMANAGER_H_ */
