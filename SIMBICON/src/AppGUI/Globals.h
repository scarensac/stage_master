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

#include <TCL/tcl.h>
#include <TCL/tk.h>
#include "AppGUI\Application.h"
#include "AppGUI\GLWindow.h"
#include <include/glut.h>

//disable all the 'deprecated function' warnings
#pragma warning( disable : 4996)

/**
	This class is used as a container for all the global variables that are needed in the system.
*/
class Globals{
public:
	//a reference to the tcl interpreter that we are using
	static Tcl_Interp *tclInterpreter;
	//a reference to the application that is running
	static Application* app;
	//a reference to the openGL window
	static GLWindow* window;
	//indicates whether or not the animation (i.e. simulation, play back, etc) is playing
	static int animationRunning;
	//gives the ratio of animation time to real time.
	static double animationTimeToRealTimeRatio;
	//this is the desired frame rate
	static double desiredFrameRate;
	//flag that controls the drawing of the FPS information
	static int drawFPS;
	//flag that controls the drawing of the cubeMap
	static int drawCubeMap;
	//flag that controls the drawing of the golbal axes
	static int drawGlobalAxes;
	//flag that controls the drawing of the shadows
	static int drawShadows;
	//flag that controls the drawing of the collision primitives
	static int drawCollisionPrimitives;
	//flag that controls the drawing of the ground
	static int drawGroundPlane;
	//flag that controls the drawing of the contact forces
	static int drawContactForces;
	//flag that controls the drawing of the desired pose that is being tracked
	static int drawDesiredPose;
	//controls the phase at which the target pose is drawn
	static double targetPosePhase;
	//flag that controls the drawing of the push interface
	static int drawPushInterface;
	//flag that controls the drawing of the curve editor
	static int drawCurveEditor;
	//flag that controls the drawing of the canvas
	static int drawCanvas;
	//flag that controls the camera tracking
	static int followCharacter;
	//flag that controls the joint position display
	static int drawJoints;
	//flag that controls the drawing of screenshots
	static int drawScreenShots;
	//flag that controls the capturing of the 3D world in OBJ files
	static int drawWorldShots;
	//flag that controls the capturing of the controller
	static int drawControlShots;
	//a text variable to display the current control shot displayed
	static char* currControlShotStr;
	//flat that indicates if the controller D and V trajectories should be updated on next step
	static int updateDVTraj;
	//indicates wether or not to use shaders
	static bool useShader;
	//indicates wether or not to use the console
	static bool useConsole;
	//this string will contain the path to the init folder
	static std::string init_folder_path;
	//this string will contain the path to the data folder
	static std::string data_folder_path;
	//this string will contain the path to the binaries folder
	static std::string binaries_folder_path;
	
	//those booleans are used to activate or disactivate the tk and gl interfaces (for evolution)
	static bool use_tk_interface;
	static bool use_gl_interface;
	
	//this bollean specify that we are in the evolution mode 
	static bool evolution_mode;
	//those variable contain the cost of the usage of each speed control strategy
	static double ipm_alteration_cost;
	static double virtual_force_cost;


	//and this one specify if we want a save at the end of the evolution
	//the name is here to indicate a second config to save the current simulation
	static bool save_mode; 
	static std::string primary_save_config;
	static std::string secondary_save_config;
	//this variable is here to let the user specifya controller that overide the input.conf
	static std::string save_mode_controller;
	//this bool is here to tell the system  to close the program after saving the state
	static bool close_after_saving;

	//those are just some var to do the communication to show the speed on the gl window
	static Vector3d avg_speed;

	//these params define the ground plane - for drawing purposes only
	static double a, b, c, d;


	//just some value to realise the paper (ignore them)
	static double ref_paper_eval, cur_paper_eval;

	Globals(void);
	~Globals(void);

	static void changeCurrControlShotStr( int currControlShot );
};


//print in an openGL window. The raster position needs to be already defined.
void gprintf(const char *format, ...);

//print in an openGL window with a large font. The raster position needs to be already defined.
void glargeprintf(const char *format, ...);


//and some function prototypes
int tprintf(const char *format, ...);

//declare some useful constants and data types here as well
#define STRLEN 200
#define MAX_LINE 255
typedef char STR[STRLEN];


#define MOUSE_LBUTTON 1
#define MOUSE_RBUTTON 2
#define MOUSE_MBUTTON 3
#define MOUSE_WHEEL_DOWN 4
#define MOUSE_WHEEL_UP 5

#define MOUSE_DOWN 1
#define MOUSE_DRAG 2
#define MOUSE_UP 3
#define MOUSE_MOVE 4


