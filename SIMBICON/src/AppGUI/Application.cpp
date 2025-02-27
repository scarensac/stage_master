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

#include "Application.h"
#include <include/GLheaders.h>
#include <GlUtils/GLUtils.h>
#include <AppGUI/Globals.h>
#include <Core/SimGlobals.h>
#include <sstream>
#include <iostream>



/**
 * Constructor.
 */
Application::Application(void){

	registerTclFunctions();

}

/**
 * Destructor.
 */
Application::~Application(void){

}

/**
 * This method is called whenever the window gets redrawn.
 */
void Application::draw(bool shadow){
	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	glColor3f(1.0f,1.0f,0.0f);
	glBegin(GL_QUADS);
		glVertex3f(-1,-1,0);
		glVertex3f(-1,1.0,0);
		glVertex3f(1,1,0);
		glVertex3f(1,-1,0);
	glEnd();
}

/**
 * This method is called whenever the system wants to draw the current frame to an obj file
 * The file is already opened and the header is written, the application should only output
 * vertices and faces
 *
 * vertexIdxOffset indicates the index of the first vertex for this object, this makes it possible to render
 * multiple different meshes to the same OBJ file
 *
 * This is a fallback-default method that should be overriden by all subclasses.
 * Subclasses shouldn't call this method.
 *
 * Returns the number of vertices written to the file
 */
uint Application::renderToObjFile(FILE* fp, uint vertexIdxOffset) {

	fprintf( fp, "# Render to OBJ file not supported!\n\n" );

	return 0;
}

/**
 * This method returns the target that the camera should be looking at
 */
Point3d Application::getCameraTarget(){
	return Point3d(0, 1, 0);
}


/**
 * This metod is used to draw the ground. We need to separate it from the main drawing method because of the shadows.
 */
void Application::drawGround(){
	glDisable(GL_LIGHTING);
	glColor3d(1, 1, 1);
	//GLUtils::drawCheckerboard(10, 1, 0);

	double size = 1500;

	double x, z;
	
	glEnable(GL_TEXTURE_2D);
	groundTexture->activate();
	glBegin(GL_QUADS);
		x = size; z = size;
		glTexCoord2d(x/2, z/2);
		glVertex3d(x, (-x*Globals::a-z*Globals::c-Globals::d)/Globals::b,z);
		z = -size;
		glTexCoord2d(x/2, z/2);
		glVertex3d(x, (-x*Globals::a-z*Globals::c-Globals::d)/Globals::b,z);
		x = -size; z = -size;
		glTexCoord2d(x/2, z/2);
		glVertex3d(x, (-x*Globals::a-z*Globals::c-Globals::d)/Globals::b,z);
		z = size;
		glTexCoord2d(x/2, z/2);
		glVertex3d(x, (-x*Globals::a-z*Globals::c-Globals::d)/Globals::b,z);
	glEnd();

	//I just duplicate it to show the water level (just for the time being)
	if (SimGlobals::water_level > 0){
		size = 15;
		glEnable(GL_TEXTURE_2D);
		waterTexture->activate();
		glBegin(GL_QUADS);
		x = size*100 ; z = size * 100;
		glTexCoord2d(x / 2, z / 2);
		glVertex3d(x, (-x*Globals::a - z*Globals::c - Globals::d) / Globals::b + SimGlobals::water_level + 0.001, z);
		z = -size * 100;
		glTexCoord2d(x / 2, z / 2);
		glVertex3d(x, (-x*Globals::a - z*Globals::c - Globals::d) / Globals::b + SimGlobals::water_level + 0.001, z);
		x = -size * 100; z = -size * 100;
		glTexCoord2d(x / 2, z / 2);
		glVertex3d(x, (-x*Globals::a - z*Globals::c - Globals::d) / Globals::b + SimGlobals::water_level + 0.001, z);
		z = size * 100;
		glTexCoord2d(x / 2, z / 2);
		glVertex3d(x, (-x*Globals::a - z*Globals::c - Globals::d) / Globals::b + SimGlobals::water_level + 0.001, z);
		glEnd();
	}

}

/**
 * This method is used to return the normal to the ground. I don't know what should happen if the ground isn't flat... 
 * only used for shadow drawing!
 */
Vector3d Application::getGroundNormal(){
	return Vector3d(Globals::a, Globals::b, Globals::c);
}



/**
 * This method gets called when the application gets initialized.
 */
void Application::init(){
	std::ostringstream oss;
	oss << Globals::data_folder_path << "textures/grid.bmp";
	char dest[230];
	strcpy(dest, oss.str().c_str());
	groundTexture = new GLTexture(dest);


	std::ostringstream oss2;
	oss2 << Globals::data_folder_path << "textures/waterText.transpa.bmp";
	//oss2 << Globals::data_folder_path << "textures/blue.transpa.bmp";
	char dest2[230];
	strcpy(dest2, oss2.str().c_str());
	waterTexture = new GLTexture(dest2);
}

/**
 * This method is used to restart the application.
 */
void Application::restart(){
}

/**
 * This method is used to reload the application.
 */
void Application::reload(){

}

/**
 *	This method is used when a mouse event gets generated. This method returns true if the message gets processed, false otherwise.
 */
bool Application::onMouseEvent(int eventType, int button, int mouseX, int mouseY){
	return false;
}

/**
 *  This method is used when a keyboard event gets generated. This method returns true if the message gets processed, false otherwise.
 */
bool Application::onKeyEvent(int key){
	if (key == 27){
		exit(0);
		return true;//useless but at least it follow the function logic 
	}
	else if (key == 122){
		if (SimGlobals::water_level < 0.5){
			SimGlobals::water_level += 0.0015;
		}else if (SimGlobals::water_level < 1.0){
			SimGlobals::water_level += 0.003;
		}
		return true;
	}

	return false;
}

/**
 * This method will get called on idle. This method should be responsible with doing the work that the application needs to do 
 * (i.e. run simulations, and so on).
 */
void Application::processTask(){
	//nothing to do here...
}

/**
 * Registers TCL functions specific to this application
 */
void Application::registerTclFunctions() {
}