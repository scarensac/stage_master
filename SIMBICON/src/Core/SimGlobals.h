/*
	Simbicon 1.5 Controller Editor Framework, 
	Copyright 2009 Stelian Coros, Philippe Beaudoin and Michiel van de Panne.
	All rights reserved. Web: www.cs.ubc.ca/~van/simbicon_cef

	This file is part of the Simbicon 1.5 Controller Editor Framework.

	Simbicon 1.5 Controller Editor Framework is free software: you can 
	redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Simbicon 1.5 Controller Editor Framework is distributed in the hope 
	that it will be useful, but WITHOUT ANY WARRANTY; without even the 
	implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
	See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Simbicon 1.5 Controller Editor Framework. 
	If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <MathLib/Vector3d.h>
#include <Physics/ODEWorld.h>


#define LEFT_STANCE 0
#define RIGHT_STANCE 1

/**
	This class is used as a container for all the constants that are pertinent for the physical simulations, the controllers, etc.
*/

class SimGlobals {
public:
	//We will assume that the gravity is in the y-direction (this can easily be changed if need be), and this value gives its magnitude. 
	static double gravity;
	//this is the direction of the up-vector
	static Vector3d up;
	//if this is set to true, then the heading of the character is controlled, otherwise it is free to do whatever it wants
	static int forceHeadingControl;
	//this variable is used to specify the desired heading of the character
	static double desiredHeading;
	//and this is the desired time interval for each simulation timestep (does not apply to animations that are played back).
	static double dt;

	static AbstractRBEngine* activeRbEngine;

	//temp..
	static double targetPos;
	static double targetPosX;
	static double targetPosZ;

	static double conInterpolationValue;
	static double bipDesiredVelocity;

	static int constraintSoftness;

	static int CGIterCount;
	static int linearizationCount;

	static double rootSagittal;
	static double rootLateral;
	static double swingHipSagittal;
	static double swingHipLateral;
	static double stanceAngleSagittal;
	static double stanceAngleLateral;
	static double stanceKnee;


	static double COMOffsetX;
	static double COMOffsetZ;

	static double time_factor;

	//memebers for the water force
	static double force_alpha;
	static double water_level;
	static double liquid_density;
	static double left_stance_factor;//0 or 1 

	//I'll use the contact point structure but all I want is the position and the force
	static std::vector<ForceStruct> vect_forces;

	//for the deplacement direction control
	static double velDSagittal;
	static double velDCoronal;

	static double step_width;

	//those variables are here for the evaluation mode (done in the case of an evolution strategy)
	static bool is_evaluation;
	static int steps_before_evaluation;
	static int nbr_evaluation_steps;

	//those variablex are here to give us a way to prioritise one strategy of speedcontrol over the others
	static double ipm_alteration_effectiveness; 
	static double virtual_force_effectiveness;


	SimGlobals(void){
	}
	~SimGlobals(void){
	}

	inline static AbstractRBEngine* getRBEngine(){
		if (activeRbEngine == NULL)
			activeRbEngine = new ODEWorld();
		return activeRbEngine;
//		return new PhysEng();
	}

};
