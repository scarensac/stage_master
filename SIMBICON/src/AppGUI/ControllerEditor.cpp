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

#include "ControllerEditor.h"
#include "Globals.h"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <sstream>

/**
 * Constructor.
 */
ControllerEditor::ControllerEditor(void){

	//tprintf("Loading Controller Editor...\n");

	std::ostringstream oss;
	oss << Globals::init_folder_path;
	oss << "input.conF";

	strcpy(inputFile,  oss.str().c_str());

	loadFramework();

	Globals::changeCurrControlShotStr( -1 );
	conF->getState(&conState);

	nextControlShot = 0;
	maxControlShot = -1;
	
	if (Globals::use_tk_interface){
		registerTclFunctions();
	}
}


/**
 * Destructor.
 */
ControllerEditor::~ControllerEditor(void){

	nextControlShot = 0;
	maxControlShot = -1;

	clearEditedCurves();
}

/**
 * This method is used to create a physical world and load the objects in it.
 */
void ControllerEditor::loadFramework(){
	loadFramework( -1 );
}


/**
 * This method is used to create a physical world and load the objects in it.
 */
void ControllerEditor::loadFramework( int controlShot ){
	this->world = NULL;
	//create a new world, and load some bodies
	try{
		if( controlShot < 0 ) {
			conF = new SimBiConFramework(inputFile, NULL);
		}
		else {
			char conFile[256];
			sprintf(conFile, "..\\controlShots\\cs%05d.sbc", controlShot);
			conF = new SimBiConFramework(inputFile, conFile);
		}

		Globals::changeCurrControlShotStr( controlShot );
		conF->getState(&conState);
		this->world = conF->getWorld();
	}catch(const char* msg){
		conF = NULL;
		tprintf("Error: %s\n", msg);
		logPrint("Error: %s\n", msg);
		exit(-1);
	}

}


/**
	This method draws the desired target that is to be tracked.
*/
void ControllerEditor::drawDesiredTarget(){
	Character* ch = conF->getCharacter();
	//we need to get the desired pose, and set it to the character
	DynamicArray<double> pose;
	//conF->getController()->updateTrackingPose(pose, Globals::targetPosePhase);
	conF->getController()->getDesiredPose(pose);


	glPushMatrix();
	Point3d p = ch->getRoot()->getCMPosition();
	//this is where we will be drawing the target pose
	p.x += 1;
	p.y = 1;
	p.z += 0;

	Globals::window->setCameraTarget(p);

	worldState.clear();
	conF->getWorld()->getState(&worldState);
	
	pose[0] = 0;
	pose[1] = 0;
	pose[2] = 0;
	
	ch->setState(&pose);
	
	glTranslated(p.x, p.y, p.z);

	TransformationMatrix tmp;
	double val[16];
	conF->getCharacterFrame().getRotationMatrix(&tmp);
	tmp.getOGLValues(val);
	glMultMatrixd(val);

	float tempColor[] = {0.5, 0.5, 0.5, 1.0};
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, tempColor);

	conF->getWorld()->drawRBs(SHOW_MESH);
	p = ch->getRoot()->getCMPosition();
	glDisable(GL_LIGHTING);
	glTranslated(p.x,p.y,p.z);
	GLUtils::drawAxes(0.2);
	glPopMatrix();
	glEnable(GL_LIGHTING);
	//set the state back to the original...
	conF->getWorld()->setState(&worldState);
}


/**
 * This method is called whenever the window gets redrawn.
 */
void ControllerEditor::draw(bool shadowMode){
	int flags = SHOW_MESH;
	//if we are drawing shadows, we don't need to enable textures or lighting, since we only want the projection anyway
	if (shadowMode == false){
		flags |= SHOW_COLOURS;

		glEnable(GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);
		if (Globals::drawCollisionPrimitives)
			flags |= SHOW_CD_PRIMITIVES | SHOW_FRICTION_PARTICLES;
		if (Globals::drawJoints){
			flags |= SHOW_JOINTS | SHOW_BODY_FRAME;
		}

		if (Globals::drawContactForces){
			//figure out if we should draw the contacts, desired pose, etc.
			glColor3d(0, 0, 1);
			double factor = 0.001;
			//*
			DynamicArray<ContactPoint>* cfs = conF->getWorld()->getContactForces();
			for (uint i=0;i<cfs->size();i++){
				ContactPoint *c = &((*cfs)[i]);
				
				GLUtils::drawCylinder(0.005, c->f * 9 *factor, c->cp);
				GLUtils::drawCone(0.015, c->f * 1 *factor, c->cp+c->f*9*factor);
			}
			//*/
			//*
			std::vector<ForceStruct> vect = SimGlobals::vect_forces;
			
			for (uint i = 0; i < vect.size(); ++i){
				GLUtils::drawCylinder(0.005, vect[i].F * 9*factor, vect[i].pt);
				GLUtils::drawCone(0.015, vect[i].F * 1 * factor, vect[i].pt + vect[i].F * 9 * factor);
			}
			//*/

		}
	}
	else{
		glDisable(GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);
	}
	
	if (conF == NULL)
		return;

	AbstractRBEngine* rbc = conF->getWorld();

	if (rbc)
		rbc->drawRBs(flags);

	if (shadowMode == false){
		//draw the pose if desirable...
		if (Globals::drawDesiredPose && conF && conF->getWorld()){
			drawDesiredTarget();
		}
	}
}

/**
 * This method is used to draw extra stuff on the screen (such as items that need to be on the screen at all times)
 */
void ControllerEditor::drawExtras(){
	if (Globals::drawCurveEditor == 1) {
		for( uint i = 0; i < curveEditors.size(); ++i )
			curveEditors[i]->draw();
	}else
		InteractiveWorld::drawExtras();
}


/**
 * This method is used to restart the application.
 */
void ControllerEditor::restart(){
	conF->setState(conState);
}


/**
 * This method is used to reload the application.
 */
void ControllerEditor::reload(){
	nextControlShot = 0;
	clearEditedCurves();

	delete conF;
	loadFramework();
}


/**
	This method is used to undo the last operation
*/
void ControllerEditor::undo(){
	if( nextControlShot <= 0 ) return;
	nextControlShot--;
	clearEditedCurves();
	delete conF;
	loadFramework(nextControlShot-1);	
}

/**
	This method is used to redo the last operation
*/
void ControllerEditor::redo(){
	if( nextControlShot >= maxControlShot+1 ) return;
	nextControlShot++;
	clearEditedCurves();
	delete conF;
	loadFramework(nextControlShot-1);	
}


void ControllerEditor::stepTaken() {


	if( Globals::updateDVTraj ) {
		
		if (conF) {


			SimBiConState *state = conF->getController()->states[ lastFSMState ];

			dTrajX.simplify_catmull_rom( 0.05 );
			dTrajZ.simplify_catmull_rom( 0.05 );
			vTrajX.simplify_catmull_rom( 0.05 );
			vTrajZ.simplify_catmull_rom( 0.05 );

			state->updateDVTrajectories(NULL, NULL, dTrajX, dTrajZ, vTrajX, vTrajZ );

			clearEditedCurves();
			addEditedCurve( state->dTrajX );
			addEditedCurve( state->dTrajZ );
			addEditedCurve( state->vTrajX );
			addEditedCurve( state->vTrajZ );
		}

	}

	if( Globals::drawControlShots ) {
		/*
		char stateFileName[100], fName[100];
		sprintf(stateFileName, "..\\controlShots\\cs%05d.rs", nextControlShot);
		conF->getCharacter()->saveReducedStateToFile(stateFileName);
		sprintf(fName, "..\\controlShots\\cs%05d.sbc", nextControlShot);
		conF->getController()->writeToFile(fName,stateFileName);
		Globals::changeCurrControlShotStr( nextControlShot );
		maxControlShot = nextControlShot;
		nextControlShot++;
		Globals::drawControlShots = false;
		Tcl_UpdateLinkedVar( Globals::tclInterpreter, "toggleControlshots" );
		conF->getState(&conState);
		//*/

		//we just same the position and the controller
		
		conF->save();

		if (Globals::close_after_saving){
			exit(0);
		}

		Globals::drawControlShots = false;
		
		if (Globals::use_tk_interface){
			Tcl_UpdateLinkedVar(Globals::tclInterpreter, "toggleControlshots");
		}

		
	}
}


/**
 *	This method is used when a mouse event gets generated. This method returns true if the message gets processed, false otherwise.
 */
bool ControllerEditor::onMouseEvent(int eventType, int button, int mouseX, int mouseY){

	//need to figure out if the mouse is in the curve editor (and if we care)...
	if ( Globals::drawCurveEditor == 1 ) {
		for( uint i = 0; i < curveEditors.size(); ++i )
			if( curveEditors[i]->onMouseEvent( eventType, button, mouseX, mouseY ) ) {
				return true;
			}
	}

	return InteractiveWorld::onMouseEvent(eventType, button, mouseX, mouseY);

}

/**
 * This method gets called when the application gets initialized. 
 */
void ControllerEditor::init(){
	InteractiveWorld::init();
}

/**
 * This method returns the target that the camera should be looking at
 */
Point3d ControllerEditor::getCameraTarget(){
	if (conF == NULL)
		return Point3d(0,1,0);
	Character* ch = conF->getCharacter();
	if (ch == NULL)
		return Point3d(0,1,0);
	//we need to get the character's position. We'll hack that a little...
	return Point3d(ch->getRoot()->getCMPosition().x, 1, ch->getRoot()->getCMPosition().z);
}

/**
* This method will get called on idle. This method should be responsible with doing the work that the application needs to do 
* (i.e. run simulations, and so on).
*/
void ControllerEditor::processTask(){
	//if (Globals::animationRunning == 0){
	//	return;
	//}


	double simulationTime = 0;
	double maxRunningTime = 0.98/Globals::desiredFrameRate;

	static double step_time_end = 0;

	static double initial_phi = 0;
	static double last_phi = 0;
	static double ratio = 1;


	
	


	double cur_height = 0;

	static double phi_it = 0;
	std::vector<double> phi_vect;
	std::vector<Vector3d> speed_vect;



	//if we still have time during this frame, or if we need to finish the physics step, do this until the simulation time reaches the desired value
	while (simulationTime/maxRunningTime < Globals::animationTimeToRealTimeRatio){
		simulationTime += SimGlobals::dt;
		step_time_end += SimGlobals::dt;

		static uint count_step = 0;
		//we just make sure that we don't try anything before the initialisation of the physical world
		if (conF) { 
			
			//get the current phase, pose and state and update the GUI
			double phi = conF->getController()->getPhase();

			//init some variables
			conF->simStepPlan(SimGlobals::dt);



			lastFSMState = conF->getController()->getFSMState();
			double signChange = (conF->getController()->getStance() == RIGHT_STANCE)?-1:1;

			Globals::targetPosePhase = phi;

			
			cur_height = conF->getController()->getSwingFootPos().y;
			
			if (Globals::use_tk_interface){
				Tcl_UpdateLinkedVar( Globals::tclInterpreter, "targetPosePhase" );
			}
	
			
			//if phi is lower than the last position of our trajectory, it means we changed phase and so we need to 
			//reset the trajectory
			if( phi < dTrajX.getMaxPosition() ) {
				dTrajX.clear();
				dTrajZ.clear();
				vTrajX.clear();
				vTrajZ.clear();
							
			}

			//we add the current position to the trajectory
			Vector3d d = conF->getController()->d;
			Vector3d v = conF->getController()->v;
			dTrajX.addKnot( phi, d.x * signChange);
			dTrajZ.addKnot( phi, d.z  );
			vTrajX.addKnot( phi, v.x * signChange );
			vTrajZ.addKnot( phi, v.z  );					

			SimGlobals::left_stance_factor = 0;
			if (conF->getController()->getStance() == RIGHT_STANCE){
				SimGlobals::left_stance_factor = 1;
			}

			//we can now advance the simulation
			bool newStep = conF->advanceInTime(SimGlobals::dt);

			//if we fall we just stop
			if (Globals::evolution_mode){

				if ((conF->getCharacter()->getRoot()->getCMPosition().y<0.3)){
					Globals::animationRunning = 0;
					std::ofstream myfile("eval_result.txt");
					if (myfile.is_open()){
						myfile << std::fixed << std::scientific << std::setprecision(8) << (double)10E20;
						myfile.close();
					}
					else{
						std::cout << "damn dat fail" << std::endl;
					}

					exit(0);
				}

				if (count_step>(uint)SimGlobals::steps_before_evaluation){
					static double eval_result=0;
					static double cumul_time = 0;
					cumul_time += SimGlobals::dt;

					
					//here I'll do the evaluation
					//first some var used by multiples eval
					std::vector<Joint*> vect_lower_body;
					conF->getController()->character->getCharacterBottom(vect_lower_body);

					//*
					//this version just sum the torques on the lower body
					double eval_buff_torque = 0;
					for (int i = 0; i < (int)vect_lower_body.size(); ++i){
						eval_buff_torque += conF->getController()->torques[vect_lower_body[i]->get_idx()].length();
					}
					eval_result += 3 * eval_buff_torque / (1.2*1E6);
					//*/
					//*
					//this version just sum the drag torque
					double eval_buff_drag = 0;
					for (auto it = conF->resulting_impact.begin(); it != conF->resulting_impact.end(); ++it){
						WaterImpact impact = it->second;
						eval_buff_drag += impact.drag_torque.length();
					}
					eval_result += 6 * eval_buff_drag / (6 * 1E4);
					//*/
					/*
					//this version minimise the maximum necessary trque and we do it for each step
					static double cur_step_eval = 0;
					static bool first_pass = true;
					std::vector<Joint*> vect_lower_body;
					static std::vector<double> vect_max_torques;

					if (last_phi > phi){
					eval_result += cur_step_eval;
					vect_max_torques.clear();
					first_pass = true;
					}

					if (first_pass){
					conF->getController()->character->getCharacterBottom(vect_lower_body);
					for (int i = 0; i < (int)vect_lower_body.size(); ++i){
					vect_max_torques.push_back( conF->getController()->torques[vect_lower_body[i]->get_idx()].length());
					}
					}
					else{
					for (int i = 0; i < (int)vect_lower_body.size(); ++i){
					double new_val = conF->getController()->torques[vect_lower_body[i]->get_idx()].length();
					if (new_val>vect_max_torques[i]){
					vect_max_torques[i] = new_val;
					}
					}
					}
					cur_step_eval = 0;
					for (int i = 0; i < (int)vect_max_torques.size(); ++i){
					cur_step_eval += vect_max_torques[i];
					}

					first_pass = false;
					//*/
					//*
					//this version will minimize the weighted acc on both the target positions and the effective result
					//std::vector<Joint*> vect_lower_body;
					//conF->getController()->character->getCharacterBottom(vect_lower_body);
					static double cur_step_eval = 0;
					static bool first_pass = true;
					static std::vector<Vector3d> vect_ang_speed;
					static std::vector<Vector3d> vect_ang_speed_desired_pose;

					if (last_phi > phi){
						eval_result += cur_step_eval/ (4*1E1);
						cur_step_eval = 0;
						vect_ang_speed.clear();
						vect_ang_speed_desired_pose.clear();
						first_pass = true;
					}

					if (first_pass){
						conF->getController()->character->getCharacterBottom(vect_lower_body);

						//we init te vector for the desired pos speed
						//we create the interface to modify the target pose
						ReducedCharacterState poseRS(&conF->getController()->desiredPose);

						for (int i = 0; i < (int)vect_lower_body.size(); ++i){
							//we init the vector for the actual speeds
							vect_ang_speed.push_back(vect_lower_body[i]->getChild()->getAngularVelocity());
							vect_ang_speed_desired_pose.push_back(poseRS.getJointRelativeAngVelocity(vect_lower_body[i]->get_idx()));
						}

					}
					else{
						ReducedCharacterState poseRS(&conF->getController()->desiredPose);

						for (int i = 0; i < (int)vect_lower_body.size(); ++i){


							//we calc the variation and ponderate by the child mass
							Vector3d new_val = vect_lower_body[i]->getChild()->getAngularVelocity();
							double d_acc = (new_val - vect_ang_speed[i]).length() / SimGlobals::dt;
							cur_step_eval += d_acc*d_acc*vect_lower_body[i]->getChild()->getMass() / 10E10 *0.8;
							vect_ang_speed[i] = new_val;

							//and do the same for the desired pos
							new_val = poseRS.getJointRelativeAngVelocity(vect_lower_body[i]->get_idx());
							d_acc = (new_val - vect_ang_speed_desired_pose[i]).length() / SimGlobals::dt;
							cur_step_eval += d_acc*d_acc*vect_lower_body[i]->getChild()->getMass() / 10E10 *0.2;
							vect_ang_speed[i] = new_val;
						}
					}

					first_pass = false;

					//*/

					
					
					//we stop if we have enougth
					if (cumul_time >SimGlobals::nbr_evaluation_steps ){
						try{
							//just add the result of the eval that only add at the end of the steps
							eval_result += cur_step_eval / (4*1E1);



							Globals::animationRunning = 0;
							
							//we penalise this simulation if the speed ain't correct
							//the accepted error is 5%
							double z_speed = Globals::avg_speed.z;
							double accepted_error = std::fmax(std::abs(SimGlobals::velDSagittal / 100), 0.005);
							if (std::abs(z_speed - SimGlobals::velDSagittal) > accepted_error){
								eval_result += (double)10E15;
							}

							//we do the same for the x axis
							double x_speed = Globals::avg_speed.x;
							accepted_error = std::fmax(std::abs(SimGlobals::velDCoronal / 100), 0.005);
							if (std::abs(x_speed - SimGlobals::velDCoronal) > accepted_error){
								eval_result += (double)10E15;
							}

							//I penalize the simulation that use phi>0.90 and <0.70 to have a movement that will be able to handle
							//interferences without having to phi motion that are on the limit of what we can control
							if (conF->getController()->phi_last_step > 0.9 || conF->getController()->phi_last_step < 0.7){
								eval_result += (double)10E10;
							}


							//*
							//this passage penalise the usage of the speed strategies
							eval_result += eval_result*(1+0.1*std::abs(conF->ipm_alt_sagittal/0.09));
							//eval_result += eval_result*Globals::virtual_force_cost*SimGlobals::virtual_force_effectiveness;
							//*/


							std::ofstream myfile("eval_result.txt");
							if (myfile.is_open()){
								myfile << std::fixed << std::scientific << std::setprecision(12) << (double)eval_result;
								myfile.close();
							}
							else{
								std::cout << "damn dat fail" << std::endl;
								exit(0);
							}

							if (Globals::save_mode){
								Globals::drawControlShots = true;
								Globals::close_after_saving = true;
								Globals::evolution_mode = false;
								Globals::animationRunning = 1;
							}

							if (!Globals::save_mode&&!Globals::close_after_saving){
								exit(0);
							}
						}
						catch (...){
							exit(0);
						}
					}
				
					//we stop if need a recovery step or if we fall
					if (conF->getController()->recovery_step||(conF->getCharacter()->getRoot()->getCMPosition().y<0.3)){
						Globals::animationRunning = 0;
						std::ofstream myfile("eval_result.txt");
						if (myfile.is_open()){
							myfile << std::fixed << std::scientific << std::setprecision(8) << (double)10E20;
							myfile.close();
						}
						else{
							std::cout << "damn dat fail" << std::endl;
							exit(0);
						}

						exit(0);

					}
					
					
				}
			}

			
			last_phi = phi;

			//we now check if we finished our current step and act accordingly
			if( newStep ) {
				phi_it = 0;
				count_step++;
				Vector3d v = conF->getLastStepTaken();


				if (!Globals::evolution_mode&&!Globals::close_after_saving){
					/*
					if (conF->getController()->recovery_step){
						tprintf("recovery: %lf %lf %lf (phi = %lf, avg_speed_x = %lf, avg_speed_z = %lf, TIME = %lf, ipm_alt_sagittal = %lf)\n",
							v.x, v.y, v.z, phi, Globals::avg_speed.x, Globals::avg_speed.z, step_time_end, conF->ipm_alt_sagittal*SimGlobals::ipm_alteration_effectiveness);
	
					}
					else{
						tprintf("step: %lf %lf %lf (phi = %lf, avg_speed_x = %lf, avg_speed_z = %lf, TIME = %lf, ipm_alt_sagittal = %lf)\n",
							v.x, v.y, v.z, phi, Globals::avg_speed.x, Globals::avg_speed.z,  step_time_end, conF->ipm_alt_sagittal*SimGlobals::ipm_alteration_effectiveness);
					}
					//*/
					tprintf("%lf, %lf, %lf // %lf, %lf // %lf, %lf // %lf, %lf, %lf\n",
						conF->getController()->velD_sagital_factor, conF->getController()->velD_coronal_factor_right, conF->getController()->velD_coronal_factor_left,
						conF->getController()->last_virt_force.z, conF->getController()->last_virt_force.x,
						conF->getController()->last_virt_force_signed.z, conF->getController()->last_virt_force_signed.x,
						conF->ipm_alt_sagittal, conF->ipm_alt_coronal_left, conF->ipm_alt_coronal_right);
					
				}
				//this print is to draw the curve of evo of the speeds
				//tprintf("%lf, %lf\n",Globals::avg_speed.x, Globals::avg_speed.z);


				step_time_end = 0;

				stepTaken();

				//get the reversed new state...
				/*
				DynamicArray<double> newState;
				if (conF->getController()->getStance() == RIGHT_STANCE)
					conF->getCharacter()->getReverseStanceState(&newState);
				else
					conF->getCharacter()->getState(&newState);

				ReducedCharacterState rNew(&newState);

				conF->getCharacter()->saveReducedStateToFile("out\\reducedCharacterState.rs", newState);
				//*/
			}

//			if (conF->getController()->isBodyInContactWithTheGround()){
//				tprintf("Lost control of the biped...\n");
//				Globals::animationRunning = false;
//				break;
//			}
//		break;
		}
	}
}




int trajectoryToEdit(ClientData clientData, Tcl_Interp *interp, int argc, CONST84 char **argv){

	// selectCurveToEdit stateIdx trajectoryIdx
	if( argc != 3 ) return TCL_ERROR;

	ControllerEditor* obj = (ControllerEditor*)clientData;

	int idx = atoi( argv[1] );
	SimBiConState* state = obj->getFramework()->getController()->getState( idx );
	if( !state ) return  TCL_ERROR;
	idx = atoi( argv[2] );
	Trajectory* trajectory = state->getTrajectory( idx );
	if( !trajectory ) return  TCL_ERROR;

	obj->clearEditedCurves();
	for( uint i = 0; i < trajectory->components.size(); ++i ) {
		obj->addEditedCurve( &trajectory->components[i]->baseTraj );
	}
	if( trajectory->strengthTraj != NULL ) {
		obj->addEditedCurve( trajectory->strengthTraj );
	}

	return TCL_OK;

}



int updateTargetPose(ClientData clientData, Tcl_Interp *interp, int argc, CONST84 char **argv){
	ControllerEditor* obj = (ControllerEditor*)clientData;
	return TCL_OK;
}

// Following are wrappers for TCL functions that can access the object
int controllerUndo (ClientData clientData, Tcl_Interp *interp, int argc, CONST84 char **argv){

	ControllerEditor* obj = (ControllerEditor*)clientData;
	obj->undo();
	return TCL_OK;
}

// Following are wrappers for TCL functions that can access the object
int controllerRedo (ClientData clientData, Tcl_Interp *interp, int argc, CONST84 char **argv){

	ControllerEditor* obj = (ControllerEditor*)clientData;
	obj->redo();
	return TCL_OK;
}


// Following are wrappers for TCL functions that can access the object
int getStateNames (ClientData clientData, Tcl_Interp *interp, int argc, CONST84 char **argv){

	ControllerEditor* obj = (ControllerEditor*)clientData;

	SimBiController* controller = obj->getFramework()->getController();

	DynamicArray<const char*> stateNames;
	uint i = 0;
	while( true ) {
		SimBiConState* state = controller->getState( i++ );
		if( !state ) break;
		stateNames.push_back( state->getDescription() );
	}	

	char* result = stringListToTclList( stateNames );
	Tcl_SetResult(interp, result, TCL_DYNAMIC);

	return TCL_OK;
}


int getTrajectoryNames (ClientData clientData, Tcl_Interp *interp, int argc, CONST84 char **argv){

	// getComponentNames stateIdx 
	if( argc != 2 ) return TCL_ERROR;

	ControllerEditor* obj = (ControllerEditor*)clientData;

	int idx = atoi( argv[1] );
	SimBiConState* state = obj->getFramework()->getController()->getState( idx );

	if( !state ) return  TCL_ERROR;

	DynamicArray<const char*> trajectoryNames;
	for( int i = 0; i < state->getTrajectoryCount(); ++i ) {
		Trajectory* trajectory = state->getTrajectory( i );
		trajectoryNames.push_back( trajectory->jName );
	}	

	char* result = stringListToTclList( trajectoryNames );
	Tcl_SetResult(interp, result, TCL_DYNAMIC);

	return TCL_OK;

}

int getComponentNames (ClientData clientData, Tcl_Interp *interp, int argc, CONST84 char **argv){

	// getComponentNames stateIdx trajectoryIdx
	if( argc != 3 ) return TCL_ERROR;

	ControllerEditor* obj = (ControllerEditor*)clientData;

	int idx = atoi( argv[1] );
	SimBiConState* state = obj->getFramework()->getController()->getState( idx );
	if( !state ) return  TCL_ERROR;
	idx = atoi( argv[2] );
	Trajectory* trajectory = state->getTrajectory( idx );
	if( !trajectory ) return  TCL_ERROR;

	DynamicArray<const char*> componentNames;
	for( uint i = 0; i < trajectory->components.size(); ++i ) {
		char* componentName = new char[ 32 ];
		sprintf( componentName, "Component %d", i );
		componentNames.push_back( componentName );
	}	

	char* result = stringListToTclList( componentNames );

	for( uint i = 0; i < componentNames.size(); ++i )
		delete[] componentNames[i];

	Tcl_SetResult(interp, result, TCL_DYNAMIC);
	return TCL_OK;

}


/**
 * Registers TCL functions specific to this application
 */
void ControllerEditor::registerTclFunctions() {	

	Application::registerTclFunctions();

	Tcl_CreateCommand(Globals::tclInterpreter, "getStateNames", getStateNames, (ClientData)this, (Tcl_CmdDeleteProc *) NULL);

	Tcl_CreateCommand(Globals::tclInterpreter, "getTrajectoryNames", getTrajectoryNames, (ClientData)this, (Tcl_CmdDeleteProc *) NULL);

	Tcl_CreateCommand(Globals::tclInterpreter, "getComponentNames", getComponentNames, (ClientData)this, (Tcl_CmdDeleteProc *) NULL);

	Tcl_CreateCommand(Globals::tclInterpreter, "trajectoryToEdit", trajectoryToEdit, (ClientData)this, (Tcl_CmdDeleteProc *) NULL);

	Tcl_CreateCommand(Globals::tclInterpreter, "updateTargetPose", updateTargetPose, (ClientData)this, (Tcl_CmdDeleteProc *) NULL);

	Tcl_CreateCommand(Globals::tclInterpreter, "controllerUndo", controllerUndo, (ClientData)this, (Tcl_CmdDeleteProc *) NULL);

	Tcl_CreateCommand(Globals::tclInterpreter, "controllerRedo", controllerRedo, (ClientData)this, (Tcl_CmdDeleteProc *) NULL);

}

/**
 * This method is to be implemented by classes extending this one. The output of this function is a point that
 * represents the world-coordinate position of the dodge ball, when the position in the throw interface is (x, y).
 */
void ControllerEditor::getDodgeBallPosAndVel(double x, double y, double strength, Point3d* pos, Vector3d* vel){
	vel->x = x;
	vel->y = 0;
	vel->z = y;

	*vel = conF->getCharacter()->getHeading().rotate(*vel) * 20;
	*pos = conF->getCharacter()->getRoot()->getCMPosition();
	*pos = *pos + conF->getCharacter()->getRoot()->getCMVelocity() * 0.5;
	pos->y +=1;
	*pos = *pos + vel->unit() * (-2);
}
