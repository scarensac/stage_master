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
#include "basecontrolframework.h"
#include "Character.h"
#include <Core/SimBiController.h>

/**
	This structure is used to hold the state of the simbicon framework. This includes the world configuration (i.e. state of the rigid bodies), the
	state of the Simbicon controller that is used and also the contact forces that are acting on the character. The forces are necessary because
	the control will be using them - the simulation will actually be ok without them, since they are recomputed before integration anyway.
*/
typedef struct {
	//hold the world state here
	DynamicArray<double> worldState;
	//hold the state of the controller here:
	SimBiControllerState conState;
	//position of the last stance foot - used to compute step lengths
	Point3d lastFootPos;
	//and this is used to hold the contact force information
	DynamicArray<ContactPoint> cfi;



} SimBiConFrameworkState;

/**
	This class is used for Simbicon control.
*/

class SimBiConFramework : public BaseControlFramework{
	friend class ControllerEditor;
private:
	//this is the controller that we will use.
	SimBiController* con;

	//we'll keep track of the vector that represents each step taken. The vector will be represented in the 'relative' world coordinate frame
	Vector3d lastStepTaken;
	//this is the position of the foot at the previous state
	Point3d lastFootPos;

	//I'll store the COM displacement on the previouses steps so I can estimate the deviation from
	//the correct movement direction
	Vector3d com_displacement_last_step;
	Vector3d com_displacement_previous_to_last_step;

	//some members for IPM
	double legLength;
	double ankleBaseHeight;
	double stepHeight;
	double coronalStepWidth;

	//we need to keep track of the position of the swing foot at the beginning of the step
	Point3d swingFootStartPos;

	//alternate planned foot trajectory, for cases where we need to go around the stance foot...
	Trajectory3D alternateFootTraj;

public:
	double step_delta;

public:
	SimBiConFramework(char* input, char* conFile = NULL);
	virtual ~SimBiConFramework(void);
	
	Vector3d get_step_size(){
		return con->get_step_size();
	}

	/**
		this method is used to advance the simulation. Typically, we will first compute the control, and then we will take one
		simulation step. If we are to apply control at this point in the simulation, we can either use a controller to recompute it,
		or we can use the values that were set before. This method returns true if the controller transitions to a new state, false
		otherwise.
	*/
	virtual bool advanceInTime(double dt, bool applyControl = true, bool recomputeTorques = true, bool advanceWorldInTime = true);

	/**
	this method gets called at every simulation time step
	*/
	virtual void simStepPlan(double dt);

	virtual void adjustStepHeight();

	/**
	this method determines the degree to which the character should be panicking
	*/
	virtual double getPanicLevel();

	/**
	determines the desired swing foot location
	*/
	virtual void setDesiredSwingFootLocation();

	/**
	determine the estimate desired location of the swing foot, given the etimated position of the COM, and the phase
	*/
	virtual Vector3d computeSwingFootLocationEstimate(const Point3d& comPos, double phase);


	/**
	modify the coronal location of the step so that the desired step width results.
	*/
	double adjustCoronalStepLocation(double IPPrediction);

	/**
	returns a panic level which is 0 if val is between minG and maxG, 1 if it's
	smaller than minB or larger than maxB, and linearly interpolated
	*/
	double getValueInFuzzyRange(double val, double minB, double minG, double maxG, double maxB);

	/**
		this method is used to load the conroller settings/states from a file.
	*/
	void loadFromFile(char* fName);

	/**
		this method is used to return the quaternion that represents the to
		'rel world frame' transformation. This is the coordinate frame that the desired pose
		is computed relative to.
	*/
	inline Quaternion getCharacterFrame(){
		return con->getCharacterFrame();
	}

	/**
		This method returns the controller that is being used.
	*/
	SimBiController* getController(){
		return con;
	}

	/**
		populates the structure that is passed in with the state of the framework
	*/
	void getState(SimBiConFrameworkState* conFState);

	/**
		populates the state of the framework with information passed in with the state of the framework
	*/
	void setState(SimBiConFrameworkState& conFState);

	/**
		this method returns the vector that corresponds to the last step taken
	*/
	inline Vector3d getLastStepTaken(){
		return lastStepTaken;
	}

};
