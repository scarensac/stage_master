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

#include "InteractiveWorld.h"
#include "CurveEditor.h"

/**
  * This class is used to build ControllerFramework and use it to control articulated figures.
  */
class ControllerEditor : public InteractiveWorld {
protected:
	//this is the physical world that contains all the objects inside
	SimBiConFramework* conF;

	//this array will be used to save/load the state of the dynamic world
	DynamicArray<double> worldState;

	//this is the name of the input file
	char inputFile[100];

	//this is the initial state of the framework...
	SimBiConFrameworkState conState;

	// This indicates the number of the future control shot and the maximal number of shots
	int nextControlShot;
	int maxControlShot;



	// These trajectories are reset after every cycle
	// dTrajX and vTrajX are sign-reversed on right stance cycles
	Trajectory1D dTrajX;
	Trajectory1D dTrajZ;
	Trajectory1D vTrajX;
	Trajectory1D vTrajZ;

	// Contains the FSM state index of the last simulation step
	int lastFSMState;

	/**
		This method is called whenever the controller has just taken a new step
	*/
	virtual void stepTaken();

public:
	/**
	 * Constructor.
	 */
	ControllerEditor(void);

	/**
	 * Destructor.
	 */
	virtual ~ControllerEditor(void);

	/**
		This method draws the desired target that is to be tracked.
	*/
	void drawDesiredTarget();

	// This is our associated curve editor
	DynamicArray<CurveEditor*> curveEditors;

	/**
	 * This method is used to create a physical world and load the objects in it.
	 */
	virtual void loadFramework();

	/**
	 * This method is used to create a physical world and load the objects in it.
	 */
	void loadFramework( int controlShot );

	/**
	 * This method is called whenever the window gets redrawn.
	 */
	virtual void draw(bool shadowMode = false);

	/**
	 * This method is used to draw extra stuff on the screen (such as items that need to be on the screen at all times)
	 */
	virtual void drawExtras();

	/**
	 * This method is used to restart the application.
	 */
	virtual void restart();

	/**
	 * This method is used to reload the application.
	 */
	virtual void reload();

	/**
		This method is used to undo the last operation
	*/
	virtual void undo();

	/**
		This method is used to redo the last operation
	*/
	virtual void redo();

	/**
     *	This method is used when a mouse event gets generated. This method returns true if the message gets processed, false otherwise.
	 */
	virtual bool onMouseEvent(int eventType, int button, int mouseX, int mouseY);


	/**
	 * Delete all curve editors
	 */
	inline void clearEditedCurves() {
		for( uint i = 0; i < curveEditors.size(); ++i )
			delete curveEditors[i];
		curveEditors.clear();
	}

	/**
	 * Selects the trajectory to edit
	 */
	inline void addEditedCurve( Trajectory1D* editedCurve ) {
		if (curveEditors.size() < 2){
			curveEditors.push_back(new CurveEditor(225 * curveEditors.size(), 0, 225, 150));
		}
		else{
			curveEditors.push_back(new CurveEditor(100+225 * curveEditors.size(), 0, 225, 150));
		}
		curveEditors.back()->setTrajectory( editedCurve );
	}

	/**
	 * Registers TCL functions specific to this application
	 */
	void registerTclFunctions();


	inline SimBiConFramework* getFramework(){
		return conF;
	}

	/**
	 * This method gets called when the application gets initialized. 
	 */
	virtual void init();

	/**
	 * This method returns the target that the camera should be looking at
	 */
	Point3d getCameraTarget();

	/**
	* This method will get called on idle. This method should be responsible with doing the work that the application needs to do 
	* (i.e. run simulations, and so on).
	*/
	virtual void processTask();


	/**
     * This method is to be implemented by classes extending this one. The output of this function is a point that
	 * represents the world-coordinate position of the dodge ball, when the position in the throw interface is (x, y).
	 */
	virtual void getDodgeBallPosAndVel(double x, double y, double strength, Point3d* pos, Vector3d* vel);


	/*
	this function is a quick and easy way to save the current controler and the current position
	the to boolean are here to help choose which one is saved
	*/
	void save(bool save_controller=true, bool save_position=true);
};




