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

#include "SimBiConFramework.h"
#include <Utils/Utils.h>
#include <Core/PoseController.h>
#include <Core/SimBiController.h>
#include "SimGlobals.h"
#include <Physics/ODEWorld.h>
#include "Core\ForcesUtilitary.h"
#include <fstream>
#include <sstream>
#include "AppGUI\Globals.h"

SimBiConFramework::SimBiConFramework(char* input, char* conFile){
	//we should estimate these from the character info...
	legLength = 0.90;//the leg does 0.94 but since the leg ain't centered on the pelvis the effective length is a bit lower
	ankleBaseHeight = 0.05;
	stepHeight = 0.15;
	coronalStepWidth=0.1;
	step_delta = 0;


	//create the physical world...
	pw = SimGlobals::getRBEngine();
	con = NULL;
	bip = NULL;
	bool conLoaded = false;

	com_displacement_last_step = Vector3d(0, 0, 0);
	com_displacement_previous_to_last_step = Vector3d(0, 0, 0);


	//now we'll interpret the input file...
	if (input == NULL)
		throwError("NULL file name provided.");
	FILE *f = fopen(input, "r");
	if (f == NULL)
		throwError("Could not open file: %s", input);


	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	//this is where it happens.
	while (!feof(f)){
		//get a line from the file...
		fgets(buffer, 200, f);
		if (feof(f))
			break;
		if (strlen(buffer)>195)
			throwError("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = lTrim(buffer);
		int lineType = getConLineType(line);

		std::string path;
		char effective_path[256];
		switch (lineType) {
			case LOAD_RB_FILE:
				//first i need to add the part of the path to go to the configutation_data folder
				path=std::string((trim(line)));
				path = interpret_path(path);
				
				//and now we cna use it
				strcpy(effective_path, path.c_str());
				pw->loadRBsFromFile(effective_path);
				if (bip == NULL && pw->getAFCount()>0) {
					bip = new Character(pw->getAF(0));
					con = new SimBiController(bip);
				}
				break;
			case LOAD_CON_FILE:
				if( conFile != NULL ) break; // Controller file
				if (con == NULL){
					throwError("The physical world must contain at least one articulated figure that can be controlled!");
				}
				
				//if I am in save mode I'll simply
				if (Globals::save_mode&&(!Globals::save_mode_controller.empty())){
					std::ostringstream oss;
					oss << Globals::data_folder_path;
					oss << "controllers/";
					oss << Globals::save_mode_controller;
					path = oss.str();
				}
				else{
					//first i need to add the part of the path to go to the configutation_data folder
					path = std::string((trim(line)));
					path = interpret_path(path);
				}

				//std::cout << "loading controller:" << path << std::endl;
				//and now we cna use it
				strcpy(effective_path, path.c_str()); 
				con->loadFromFile(effective_path);
				conLoaded = true;
				break;
			case CON_NOT_IMPORTANT:
				tprintf("Ignoring input line: \'%s\'\n", line);
				break;
			case CON_COMMENT:
				break;
			default:
				throwError("Incorrect SIMBICON input file: \'%s\' - unexpected line.", buffer);
		}
	}
	fclose(f);

	if( conFile != NULL ) {
		if( con == NULL )
				throwError("The physical world must contain at least one articulated figure that can be controlled!");
		con->loadFromFile(conFile);
		conLoaded = true;
	}

	if (!conLoaded)
		throwError("A controller must be loaded!");

	//in case the state has changed while the controller was loaded, we will update the world again...
//	pw->updateTransformations();

	//initialize the last foot position to the position of the stance foot - this may not always be the best decision, but whatever...
	lastFootPos = con->getStanceFootPos();
}


SimBiConFramework::~SimBiConFramework(void){
	delete con;
}

/**
	this method is used to advance the simulation. Typically, we will first compute the control, and then we will take one
	simulation step. If we are to apply control at this point in the simulation, we can either use a controller to recompute it,
	or we can use the values that were set before. This method returns true if the controller transitions to a new state, false
	otherwise.
*/

bool SimBiConFramework::advanceInTime(double dt, bool applyControl, bool recomputeTorques, bool advanceWorldInTime){
	static double avg_speed = 0;
	static int times_vel_sampled = 0;

	//some static var for later use
	static Vector3d cur_com = Vector3d(0, 0, 0);

	ODEWorld* world = dynamic_cast<ODEWorld*>(pw);

	//we simulate the effect of the liquid
	SimGlobals::vect_forces.clear();
	resulting_impact.clear();
	world->compute_water_impact(con->get_character(),SimGlobals::water_level, resulting_impact);

	//I'll add a force for the control of the speed (only for now, I will have to convert it to virtual torques)
	/*Joint* torso_joint = con->get_character()->getJointByNameOfChild("torso");
	RigidBody* body = torso_joint->getChild();

	double factor;
	factor = (SimGlobals::left_stance_factor*SimGlobals::balance_force_factor_left +
		(1 - SimGlobals::left_stance_factor)*SimGlobals::balance_force_factor_right);
	//factor = (SimGlobals::balance_force_factor_right + SimGlobals::balance_force_factor_left) / 2;
	Vector3d F = -Vector3d(0, 0, 1)*SimGlobals::liquid_density / 3000.0* 5.0 * factor;
	world->applyForceTo(body, F, body->getLocalCoordinates(body->getCMPosition()));*/

	//I'll also add a force to help the caracter follow the heading 
	//don't work
	/*if (!com_displacement_previous_to_last_step.isZeroVector()){
		Vector3d displacement = com_displacement_last_step + com_displacement_previous_to_last_step;
		Vector3d F2 = Vector3d(displacement.x / displacement.z, 0, 0);
		world->applyForceTo(body, F2, body->getLocalCoordinates(body->getCMPosition()));
	}*/


	
	//compute te new torques and apply them 
	if (applyControl == false){
		con->resetTorques();
	}
	else{
		if (recomputeTorques == true){
			con->computeTorques(pw->getContactForces(),resulting_impact);
		}
	}
	con->applyTorques();
	
	//store the current speed to be able to know the avg speed at the end
	avg_speed += getCharacter()->getHeading().getComplexConjugate().rotate(getCharacter()->getRoot()->getCMVelocity()).z;
	times_vel_sampled++;

	if (advanceWorldInTime)
		pw->advanceInTime(dt);

	int new_state_idx = con->advanceInTime(dt, pw->getContactForces());
	bool newFSMState = (new_state_idx != -1);
	//con->updateDAndV();

	//here we are assuming that each FSM state represents a new step. 
	if (newFSMState){
		con->getState(con->getFSMState())->update_joints_trajectory();

		//adapt the velD trajectories
		con->velD_adapter(false);

		//read the step width
		coronalStepWidth = SimGlobals::step_width;

		//read the speed from the gui
		con->calc_desired_velocities();

		//we save the position of the swing foot (which is the old stance foot) and of the stance foot
		swingFootStartPos=lastFootPos;

		lastStepTaken = Vector3d(lastFootPos, con->getStanceFootPos());
		//now express this in the 'relative' character coordinate frame instead of the world frame
		lastStepTaken = con->getCharacterFrame().getComplexConjugate().rotate(lastStepTaken);
		lastFootPos = con->getStanceFootPos();

		//we save the com displacement (to know if we deviate^^)
		Vector3d old_com = cur_com;
		cur_com = getCharacter()->getCOM();

		com_displacement_previous_to_last_step = com_displacement_last_step;
		com_displacement_last_step = cur_com - old_com;
		
		//now prepare the step information for the following step:
		con->swingFootTrajectoryCoronal.clear();
		con->swingFootTrajectorySagittal.clear();


		con->swingFootTrajectoryCoronal.addKnot(0, 0);
		con->swingFootTrajectoryCoronal.addKnot(1, 0);

		con->swingFootTrajectorySagittal.addKnot(0, 0);
		con->swingFootTrajectorySagittal.addKnot(1, 0);

		//addapt the variation on the IPM result depending on our speed
		avg_speed/=times_vel_sampled;
		static double previous_speed = avg_speed;

		double d_v = con->velDSagittal - (avg_speed*.75+previous_speed*0.25);
		if (d_v < 0.3){
			step_delta -= (d_v)*0.1;
		}
		if (step_delta > 0){
			step_delta = 0;
		}
		else if (step_delta < -0.09){
			step_delta = -0.09;
		}
		

		previous_speed= avg_speed;

		times_vel_sampled = 0;
		avg_speed = 0;
	}

	return newFSMState;
}


/**
this method gets called at every simulation time step
*/
void SimBiConFramework::simStepPlan(double dt){

	//update the pointers for the controler (normaly should only be done once at the start of each step)
	con->updateSwingAndStanceReferences();

	//update D and V
	con->updateDAndV();

	//let the adapter learn for the new phi
	con->velD_adapter();

	//set the foot start pos
	static bool ipm_active = false;
	if (con->getPhase() <= 0.01){
		//swingFootStartPos = con->swingFoot->getWorldCoordinates(bip->getJoint(con->swingAnkleIndex)->getChildJointPosition());
		ipm_active = false;
	}
		
	//compute desired swing foot location with the IPM if needed
	if (con->ipm_used()){
		if (!ipm_active){
			swingFootStartPos = con->swingFoot->getWorldCoordinates(bip->getJoint(con->swingAnkleIndex)->getChildJointPosition());
			ipm_active = true;
		}


		setDesiredSwingFootLocation();
	}
	else{
		ipm_active = false;
	}
	
	//set some of these settings
	//the commented ones are used to modify the trajectories (but srly i don't care right now)
	//setUpperBodyPose(ubSagittalLean, ubCoronalLean, ubTwist);
	//setKneeBend(kneeBend);
	//setDuckWalkDegree((lowLCon->stance == LEFT_STANCE) ? (-duckWalk) : (duckWalk));
	//setDesiredHeading(desiredHeading);
	//this one may be usefull but I'll use constant speed for now
	//setVelocities(velDSagittal, velDCoronal);

	//adjust for panic mode or unplanned terrain...
	adjustStepHeight();

	//and see if we're really in trouble...
	//	if (shouldAbort()) onAbort();
}

void SimBiConFramework::adjustStepHeight(){
	con->unplannedForHeight = 0;
	//srly fu the oracle for now
	/*
	if (wo != NULL){
		//the trajectory of the foot was generated without taking the environment into account, so check to see if there are any un-planned bumps (at somepoint in the near future)
		con->unplannedForHeight = wo->getWorldHeightAt(con->getSwingFootPos() + con->swingFoot->getCMVelocity() * 0.1) * 1.5;
	}//*/

	//if the foot is high enough, we shouldn't do much about it... also, if we're close to the start or end of the
	//walk cycle, we don't need to do anything... the thing below is a quadratic that is 1 at 0.5, 0 at 0 and 1...
	double panicIntensity = -4 * con->getPhase() * con->getPhase() + 4 * con->getPhase();
	panicIntensity *= getPanicLevel();
	con->panicHeight = panicIntensity * 0.05;
}



/**
this method determines the degree to which the character should be panicking
*/
double SimBiConFramework::getPanicLevel(){
	//the estimate of the panic is given, roughly speaking by the difference between the desired and actual velocities
	double panicEstimate = 0;
	panicEstimate += getValueInFuzzyRange(con->get_v().z, con->velDSagittal - 0.4, con->velDSagittal - 0.3, con->velDSagittal + 0.3, con->velDSagittal + 0.4);
	panicEstimate += getValueInFuzzyRange(con->get_v().x, con->velDCoronal - 0.3, con->velDCoronal - 0.2, con->velDCoronal + 0.2, con->velDCoronal + 0.3);
	//	boundToRange(&panicEstimate, 0, 1);
	return panicEstimate / 2;
}

/**
determines the desired swing foot location
*/
void SimBiConFramework::setDesiredSwingFootLocation(){
	Point3d com_pos = con->get_character()->getCOM();
	Point3d com_vel = con->get_character()->getCOMVelocity();
	double phi = con->getPhase();
	Vector3d step = computeSwingFootLocationEstimate(com_pos, phi);
	con->swingFootTrajectoryCoronal.setKnotValue(0, step.x);
	con->swingFootTrajectorySagittal.setKnotValue(0, step.z);

	double dt = SimGlobals::dt;
	double nphi= phi+ dt/con->get_cur_state_time();
	step = computeSwingFootLocationEstimate(com_pos + Vector3d(com_vel) * dt, nphi);
	con->swingFootTrajectoryCoronal.setKnotValue(1, step.x);
	con->swingFootTrajectorySagittal.setKnotValue(1, step.z);
	//to give some gradient information, here's what the position will be a short time later...


	con->swingFootTrajectorySagittal.setKnotPosition(0, phi);
	con->swingFootTrajectorySagittal.setKnotPosition(1, nphi);

	con->swingFootTrajectoryCoronal.setKnotPosition(0, phi);
	con->swingFootTrajectoryCoronal.setKnotPosition(1, nphi);
}


/**
determine the estimate desired location of the swing foot, given the etimated position of the COM, and the phase
*/
Vector3d SimBiConFramework::computeSwingFootLocationEstimate(const Point3d& comPos, double phase){
	Vector3d step = con->computeIPStepLocation();

	//applying the IP prediction would make the character stop, so take a smaller step if you want it to walk faster, or larger
	//if you want it to go backwards
	//step.z += -con->velDSagittal / 20;
	Vector3d com_vel = con->get_v();
	if (com_vel.y < 0&& phase>0.2){
		step.z += -con->velDSagittal / 20;
		step.z += step_delta*SimGlobals::ipm_alteration_effectiveness;
	}
	//and adjust the stepping in the coronal plane in order to account for desired step width...
	step.x = adjustCoronalStepLocation(step.x);

	boundToRange(&step.z, -0.4 * legLength, 0.4 * legLength);
	boundToRange(&step.x, -0.4 * legLength, 0.4 * legLength);

	Vector3d result;
	Vector3d initialStep(comPos, swingFootStartPos);
	initialStep = con->getCharacterFrame().inverseRotate(initialStep);
	//when phi is small, we want to be much closer to where the initial step is - so compute this quantity in character-relative coordinates
	//now interpolate between this position and initial foot position - but provide two estimates in order to provide some gradient information
	double t = (1 - phase);
	
	//I comment this line cose I want the linear model back
	t = t * t;
	boundToRange(&t, 0, 1);

	Vector3d suggestedViaPoint;
	alternateFootTraj.clear();
	bool needToStepAroundStanceAnkle = false;

	//TODO integrate the leg intersection.
	//FOr now let's forget the legs intersections
	//if (phase < 0.8 && shouldPreventLegIntersections && getPanicLevel() < 0.5)
	//	needToStepAroundStanceAnkle = detectPossibleLegCrossing(step, &suggestedViaPoint);
	
	
	if (needToStepAroundStanceAnkle){
		//use the via point...
		Vector3d currentSwingStepPos(comPos, con->getSwingFootPos());
		currentSwingStepPos = con->getCharacterFrame().inverseRotate(initialStep); currentSwingStepPos.y = 0;
		//compute the phase for the via point based on: d1/d2 = 1-x / x-phase, where d1 is the length of the vector from
		//the via point to the final location, and d2 is the length of the vector from the swing foot pos to the via point...
		double d1 = (step - suggestedViaPoint).length(); double d2 = (suggestedViaPoint - currentSwingStepPos).length(); if (d2 < 0.0001) d2 = d1 + 0.001;
		double c = d1 / d2;
		double viaPointPhase = (1 + phase*c) / (1 + c);
		//now create the trajectory...
		alternateFootTraj.addKnot(0, initialStep);
		alternateFootTraj.addKnot(viaPointPhase, suggestedViaPoint);
		alternateFootTraj.addKnot(1, step);
		//and see what the interpolated position is...
		result = alternateFootTraj.evaluate_catmull_rom(1.0 - t);
		//		tprintf("t: %lf\n", 1-t);
	}
	else{
		result.addScaledVector(step, 1.0 - t);
		result.addScaledVector(initialStep, t);
	}

	result.y = 0;

	/*
	suggestedFootPosDebug = result;
	*/
	return result;
}


/**
modify the coronal location of the step so that the desired step width results.
*/
double SimBiConFramework::adjustCoronalStepLocation(double IPPrediction){


	//when the caracter ain't in a falling situation
	double speed_control= con->get_v().x;


	//this addjust to the specifed step width
	double stepWidth = coronalStepWidth / 2;
	stepWidth = (con->getStance() == LEFT_STANCE) ? (-stepWidth) : (stepWidth);

	//this help to diminish the fact that the caracter turn the leg when inside the water
	if (con->getPhase() > 0.8){
		 if(stepWidth*speed_control > 0){
			//IPPrediction += IPPrediction /5;
		 }
		 else{
			IPPrediction -= stepWidth;
		 }
	}

	//if (stepWidth*speed_control > 0){
		IPPrediction += stepWidth;
	//}
	//else{
	//	IPPrediction += speed_control;
	//}
	IPPrediction += -con->velDCoronal / 20;

	//I'll disable all the panic system for now ...
	/**

	//now for the step in the coronal direction - figure out if the character is still doing well - panic = 0 is good, panic = 1 is bad...
	double panicLevel = 1;
	
	if (con->getStance() == LEFT_STANCE){
		panicLevel = getValueInFuzzyRange(con->get_d().x, 1.15 * stepWidth, 0.5 * stepWidth, 0.25 * stepWidth, -0.25 * stepWidth);
		panicLevel += getValueInFuzzyRange(con->get_v().x, 2 * stepWidth, stepWidth, -stepWidth, -stepWidth*1.5);
	}
	else{
		panicLevel = getValueInFuzzyRange(con->get_d().x, -0.25 * stepWidth, 0.25 * stepWidth, 0.5 * stepWidth, 1.15 * stepWidth);
		panicLevel += getValueInFuzzyRange(con->get_v().x, -stepWidth*1.5, -stepWidth, stepWidth, 2 * stepWidth);
	}
	boundToRange(&panicLevel, 0, 1);
	Trajectory1D offsetMultiplier;
	offsetMultiplier.addKnot(0.05, 0); offsetMultiplier.addKnot(0.075, 1 / 2.0);
	double offset = stepWidth * offsetMultiplier.evaluate_linear(fabs(stepWidth));
	//	if (IPPrediction * stepWidth < 0) offset = 0;
	//if it's doing well, use the desired step width...
	IPPrediction = panicLevel * (IPPrediction + offset) + (1 - panicLevel) * stepWidth;
	con->comOffsetCoronal = (1 - panicLevel) * stepWidth;

	//	if (panicLevel >= 1)
	//		tprintf("panic level: %lf; d.x = %lf\n", panicLevel, lowLCon->d.x);
	//*/

	return IPPrediction;
}

/**
returns a panic level which is 0 if val is between minG and maxG, 1 if it's
smaller than minB or larger than maxB, and linearly interpolated
*/
double SimBiConFramework::getValueInFuzzyRange(double val, double minB, double minG, double maxG, double maxB){
	if (val <= minB || val >= maxB)
		return 1;
	if (val >= minG && val <= maxG)
		return 0;
	if (val > minB && val < minG)
		return (minG - val) / (minG - minB);
	if (val > maxG && val < maxB)
		return (val - maxG) / (maxB - maxG);
	//the input was probably wrong, so return panic...
	return 1;
}

/**
	populates the structure that is passed in with the state of the framework
*/
void SimBiConFramework::getState(SimBiConFrameworkState* conFState){
	conFState->worldState.clear();
	//read in the state of the world (we'll assume that the rigid bodies and the ode world are synchronized), and the controller
	pw->getState(&(conFState->worldState));
	con->getControllerState(&(conFState->conState));
	conFState->lastFootPos = lastFootPos;
	//now copy over the contact force information
	conFState->cfi.clear();
	DynamicArray<ContactPoint> *cfs = pw->getContactForces();
	for (uint i=0;i<cfs->size();i++){
		conFState->cfi.push_back(ContactPoint((*cfs)[i]));
	}
}

/**
	populates the state of the framework with information passed in with the state of the framework
*/
void SimBiConFramework::setState(SimBiConFrameworkState& conFState){
	//set the state of the world, and that of the controller
	pw->setState(&(conFState.worldState));
	con->setControllerState(conFState.conState);
	lastFootPos = conFState.lastFootPos;
	DynamicArray<ContactPoint> *cfs = pw->getContactForces();
	cfs->clear();
	//now copy over the contact force information
	for (uint i=0;i<conFState.cfi.size();i++)
		cfs->push_back(ContactPoint(conFState.cfi[i]));
}


/*
this function is a quick and easy way to save the current controler and the current position
the to boolean are here to help choose which one is saved
*/
void SimBiConFramework::save(bool save_controller, bool save_position){

	std::string line;
	std::ostringstream os;
	os << Globals::data_folder_path;
	os << Globals::primary_save_config;

	//the form of that function is to not have to do a lot of copy of the code (and because I'm too lazy to move it into another subfunction
	bool continues = false;
	int save_config_idx = 0;
	do{
		continues = false;
		std::ifstream myfile(os.str());
		if (myfile.is_open())
		{
			std::ostringstream oss;

			//so we read the name we want for the state file

			if (std::getline(myfile, line)){
				//we add the prefix
				oss << Globals::data_folder_path << "controllers/bipV2/";
				oss << line;

				if (save_position){

					//and we write it	
					getCharacter()->saveReducedStateToFile(oss.str());
				}
			}

			//we read the name we want for the control file
			if (std::getline(myfile, line)){
				if (save_controller){
					//we add the prefix
					std::ostringstream oss2;
					oss2 << Globals::data_folder_path << "controllers/bipV2/";
					oss2 << line;

					//and we write it	
					getController()->writeToFile(oss2.str(), &oss.str());
				}
			}


			myfile.close();
		}
		else{
			std::cout << "failed save"<<std::endl;
			exit(56);
		}

		//now we check if there is a secondary save config
		if (save_config_idx==0&&!Globals::secondary_save_config.empty()){
			os.clear();
			os.str("");
			os << Globals::data_folder_path;
			os << Globals::secondary_save_config;
			continues = true;
			save_config_idx = 1;
		}


	} while (continues);
}