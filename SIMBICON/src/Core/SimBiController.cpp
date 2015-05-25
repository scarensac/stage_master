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

#include "SimBiController.h"
#include <Utils/Utils.h>
#include "SimGlobals.h"
#include "ConUtils.h"
#include "TwoLinkIK.h"
#include <MathLib/Point3d.h>

SimBiController::SimBiController(Character* b) : PoseController(b){
	if (b == NULL)
		throwError("Cannot create a SIMBICON controller if there is no associated biped!!");


	//characters controlled by a simbicon controller are assumed to have: 2 feet
	lFoot = b->getARBByName("lFoot");
	rFoot = b->getARBByName("rFoot");

	if (rFoot == NULL || lFoot == NULL){
		lFoot = b->getARBByName("lFoot2");
		rFoot = b->getARBByName("rFoot2");
	}

	if (rFoot == NULL || lFoot == NULL)
		throwError("The biped must have the rigid bodies lFoot and rFoot!");
	
	//and two hips connected to the root
	Joint* lHip = b->getJointByName("lHip");
	Joint* rHip = b->getJointByName("rHip");

	lHipIndex = b->getJointIndex("lHip");
	rHipIndex = b->getJointIndex("rHip");

	if (rFoot == NULL || lFoot == NULL)
		throwError("The biped must have the joints lHip and rHip!");

	root = b->getRoot();
	
	if (lHip->getParent() != rHip->getParent() || lHip->getParent() != root)
		throwError("The biped's hips must have a common parent, which should be the root of the figure!");

	setStance(LEFT_STANCE);
	phi = 0;

	setFSMStateTo(-1);

	stanceHipDamping = -1;
	stanceHipMaxVelocity = 4;
	rootPredictiveTorqueScale = 0;

	bodyTouchedTheGround = false;
	
	startingState = -1;
	startingStance = LEFT_STANCE;
	initialBipState[0] = '\0';

	swing_foot_traj = NULL;


	comOffsetSagittal=0;
	comOffsetCoronal=0;

	//init the speed trajecotries
	velDSagittal = 0.7;//0.95 old value
	velDCoronal = 0;


	lKneeIndex = character->getJointIndex("lKnee");
	rKneeIndex = character->getJointIndex("rKnee");
	lAnkleIndex = character->getJointIndex("lAnkle");
	rAnkleIndex = character->getJointIndex("rAnkle");


	panicHeight = 0;
	unplannedForHeight = 0;

	swingLegPlaneOfRotation = Vector3d(-1, 0, 0);

	//those are an init for the foot trajectory
	swingFootTrajectoryCoronal.clear();
	swingFootTrajectorySagittal.clear();




    swingFootTrajectoryCoronal.addKnot(0, 0);
	swingFootTrajectoryCoronal.addKnot(1, 0);

	swingFootTrajectorySagittal.addKnot(0, 0);
	swingFootTrajectorySagittal.addKnot(1, 0);

	//those are an init for the foot trajectory (delta)
	//I must notice that this delta is fully controled by the IHM in the 2010 version of the code.
	//as such this value won't change in this code
	swingFootTrajectoryDeltaCoronal.clear();
	swingFootTrajectoryDeltaHeight.clear();
	swingFootTrajectoryDeltaSagittal.clear();

	swingFootTrajectoryDeltaCoronal.addKnot(0, 0);
	swingFootTrajectoryDeltaCoronal.addKnot(1, 0);

	swingFootTrajectoryDeltaHeight.addKnot(0, 0);
	swingFootTrajectoryDeltaHeight.addKnot(1, 0);

	swingFootTrajectoryDeltaSagittal.addKnot(0, 0);
	swingFootTrajectoryDeltaSagittal.addKnot(1, 0);

	//init the virtula model controler
	vmc = new VirtualModelController(b);


	recovery_step = false;
}

/**
	This method is used to set the current FSM state of the controller to that of the index that
	is passed in.
*/
void SimBiController::setFSMStateTo(int index){
	if (index<0 || (uint)index>=states.size()){
		FSMStateIndex = 0;
		return;
	}
	FSMStateIndex = index;
}

SimBiController::~SimBiController(void){
	for (uint i=0;i<states.size();i++)
		delete states[i];
}

/**
	This method is used to set the stance 
*/
void SimBiController::setStance(int newStance){
	stance = newStance;
	if (stance == LEFT_STANCE){
		stanceFoot = lFoot;
		swingFoot = rFoot;
		swingHipIndex = rHipIndex;
		stanceHipIndex = lHipIndex;
	}else{
		stanceFoot = rFoot;
		swingFoot = lFoot;
		swingHipIndex = lHipIndex;
		stanceHipIndex = rHipIndex;
	}
	stanceFoot->set_mesh_color(0, 0, 1, 1);
	swingFoot->set_mesh_color(1, 1, 1, 1);
}

/**
	This method is used to populate the structure that is passed in with the current state
	of the controller;
*/
void SimBiController::getControllerState(SimBiControllerState* cs){
	cs->stance = this->stance;
	cs->phi = this->phi;
	cs->FSMStateIndex = this->FSMStateIndex;
	cs->bodyGroundContact = this->bodyTouchedTheGround;
}

/**
	This method is used to populate the state of the controller, using the information passed in the
	structure
*/
void SimBiController::setControllerState(const SimBiControllerState &cs){
	this->setStance(cs.stance);
	this->phi = cs.phi;
	this->setFSMStateTo(cs.FSMStateIndex);
	this->bodyTouchedTheGround = cs.bodyGroundContact;
}

/**
	This method should be called when the controller transitions to this state.
*/
void SimBiController::transitionToState(int stateIndex){
	setFSMStateTo(stateIndex);
	setStance(states[FSMStateIndex]->getStateStance(this->stance));
//	tprintf("Transition to state: %d (stance = %s) (phi = %lf)\n", stateIndex, (stance == LEFT_STANCE)?("left"):("right"), phi);
	//reset the phase...
	//I'll backup the phi that was reached
	phi_last_step = phi;
	phi = 0;
}

/**
	This method returns the net force on the body rb, acting from the ground
*/
Vector3d SimBiController::getForceOn(RigidBody* rb, DynamicArray<ContactPoint> *cfs){
	Vector3d fNet = Vector3d();
	for (uint i=0;i<cfs->size();i++){
		if ((*cfs)[i].rb1 == rb)
			fNet += (*cfs)[i].f;
		if ((*cfs)[i].rb2 == rb)
			fNet -= (*cfs)[i].f;
	}
	return fNet;
}

/**
	This method is used to determine if the rigid body that is passed in as a parameter is a
	part of a foot
*/
bool SimBiController::isFoot(RigidBody* rb){
	//check against the feet
	if (rb == lFoot || rb == rFoot)
		return true;
	//and against the toes
	for (uint j=0;j<((ArticulatedRigidBody*)lFoot)->cJoints.size();j++)
		if (((ArticulatedRigidBody*)lFoot)->cJoints[j]->child == rb)
			return true;
	for (uint j=0;j<((ArticulatedRigidBody*)rFoot)->cJoints.size();j++)
		if (((ArticulatedRigidBody*)rFoot)->cJoints[j]->child == rb)
			return true;

	return false;
}

/**
	This method returns true if the rigid body that is passed in as a parameter is a swing foot, false otherwise
*/
bool SimBiController::isSwingFoot(RigidBody* rb){
	//check against the feet
	if (rb == swingFoot)
		return true;
	//and against the toes
	for (uint j=0;j<((ArticulatedRigidBody*)swingFoot)->cJoints.size();j++)
		if (((ArticulatedRigidBody*)swingFoot)->cJoints[j]->child == rb)
			return true;
	return false;
}

/**
	This method returns true if the rigid body that is passed in as a parameter is a swing foot, false otherwise
*/
bool SimBiController::isStanceFoot(RigidBody* rb){
	//check against the feet
	if (rb == stanceFoot)
		return true;
	//and against the toes
	for (uint j=0;j<((ArticulatedRigidBody*)stanceFoot)->cJoints.size();j++)
		if (((ArticulatedRigidBody*)stanceFoot)->cJoints[j]->child == rb)
			return true;
	return false;
}

/**
	This method returns the net force on the body rb, acting from the ground
*/
Vector3d SimBiController::getForceOnFoot(RigidBody* foot, DynamicArray<ContactPoint> *cfs){
	Vector3d fNet = getForceOn(foot, cfs);

	//we will also look at all children of the foot that is passed in (to take care of toes).
	for (uint i=0;i<((ArticulatedRigidBody*)foot)->cJoints.size();i++){
		fNet += getForceOn(((ArticulatedRigidBody*)foot)->cJoints[i]->child, cfs);
	}
	return fNet;
}



/**
	This method is used to advance the controller in time. It takes in a list of the contact points, since they might be
	used to determine when to transition to a new state. This method returns -1 if the controller does not advance to a new state,
	or the index of the state that it transitions to otherwise.
*/
int SimBiController::advanceInTime(double dt, DynamicArray<ContactPoint> *cfs){
	if (FSMStateIndex >= (int)states.size()){
		tprintf("Warning: no FSM state was selected in the controller!\n");
		return -1;
	}

	bodyTouchedTheGround = false;
	//see if anything else other than the feet touch the ground...
	for (uint i=0;i<cfs->size();i++){
		//if neither of the bodies involved are articulated, it means they are just props so we can ignore them
		if ((*cfs)[i].rb1->isArticulated() == false && (*cfs)[i].rb2->isArticulated() == false)
			continue;
			
		if (isFoot((*cfs)[i].rb1) || isFoot((*cfs)[i].rb2))
			continue;

		bodyTouchedTheGround = true;
	}

	//advance the phase of the controller
	advance_phase(dt);

	//see if we have to transition to the next state in the FSM, and do it if so...
	if (states[FSMStateIndex]->needTransition(getPhase(), fabs(getForceOnFoot(swingFoot, cfs).dotProductWith(SimGlobals::up)), fabs(getForceOnFoot(stanceFoot, cfs).dotProductWith(SimGlobals::up)))){
		int newStateIndex = states[FSMStateIndex]->getNextStateIndex();
		transitionToState(newStateIndex);
		return newStateIndex;
	}

	//if we didn't transition to a new state...
	return -1;
}

/**
returns the required stepping location, as predicted by the inverted pendulum model. The prediction is made
on the assumption that the character will come to a stop by taking a step at that location. The step location
is expressed in the character's frame coordinates.
*/
Vector3d SimBiController::computeIPStepLocation(){

	Vector3d step;
	double h = fabs(character->getCOM().y - stanceFoot->getCMPosition().y);
	step.x = v.x * sqrt(h / 9.8 + v.x * v.x / (4 * 9.8*9.8));
	step.z = v.z * sqrt(h / 9.8 + v.z * v.z / (4 * 9.8*9.8));
	step.y = 0;


	/*
	//original equations
	step.x = v.x * sqrt(h / 9.8 + v.x * v.x / (4 * 9.8*9.8)) * 1.3;
	step.z = v.z * sqrt(h / 9.8 + v.z * v.z / (4 * 9.8*9.8)) * 1.1;
	step.y = 0;
	//*/
	return step;
}


/**
	This method is used to return the ratio of the weight that is supported by the stance foot.
	return -1 if there are no vertical forces on the feet (meaning neither of them toutch the ground
*/
double SimBiController::getStanceFootWeightRatio(DynamicArray<ContactPoint> *cfs){
	Vector3d stanceFootForce = getForceOnFoot(stanceFoot, cfs);
	Vector3d swingFootForce = getForceOnFoot(swingFoot, cfs);
	double totalYForce = (stanceFootForce + swingFootForce).dotProductWith(SimGlobals::up);

	if (IS_ZERO(totalYForce))
		return -1;
	else
		return stanceFootForce.dotProductWith(SimGlobals::up) / totalYForce;
}


/**
This function get the desired pose from the last step
*/
void SimBiController::getDesiredPose(DynamicArray<double>& trackingPose){
	trackingPose = desiredPose;
}

/**
	This method makes it possible to evaluate the debug pose at any phase angle
*/
void SimBiController::updateTrackingPose(DynamicArray<double>& trackingPose, double phiToUse){
	if( phiToUse < 0 )
		phiToUse = getPhase();
	if( phiToUse > 1 )
		phiToUse = 1;
	trackingPose.clear();
	this->character->getState(&trackingPose);
	
	ReducedCharacterState debugRS(&trackingPose);

	//always start from a neutral desired pose, and build from there...
	for (int i=0;i<jointCount;i++){
		debugRS.setJointRelativeOrientation(Quaternion(1, 0, 0, 0), i);
		debugRS.setJointRelativeAngVelocity(Vector3d(), i);
		controlParams[i].relToCharFrame = false;
	}

	//and this is the desired orientation for the root
	Quaternion qRootD(1, 0, 0, 0);

	SimBiConState* curState = states[FSMStateIndex];

	for (int i=0;i<curState->getTrajectoryCount();i++){
		//now we have the desired rotation angle and axis, so we need to see which joint this is intended for
		int jIndex = curState->sTraj[i]->getJointIndex(stance);
		
		//if the index is -1, it must mean it's the root's trajectory. Otherwise we should give an error
		if (curState->sTraj[i]->relToCharFrame == true || jIndex == swingHipIndex)
			controlParams[jIndex].relToCharFrame = true;
		Vector3d d0, v0; 
		computeD0( phiToUse, &d0 );
		computeV0( phiToUse, &v0 );
		Quaternion newOrientation = curState->sTraj[i]->evaluateTrajectory(this, character->getJoint(jIndex), stance, phiToUse, d - d0, v - v0);
		if (jIndex == -1){
			qRootD = newOrientation;
		}else{
			debugRS.setJointRelativeOrientation(newOrientation, jIndex);
		}
	}

	debugRS.setOrientation(qRootD);

	//now, we'll make one more pass, and make sure that the orientations that are relative to the character frame are drawn that way
	for (int i=0;i<jointCount;i++){
		if (controlParams[i].relToCharFrame){
			Quaternion temp;
			Joint* j = character->getJoint(i);
			ArticulatedRigidBody* parent = j->getParent();
			while (parent != root){
				j = parent->getParentJoint();
				parent = j->getParent();
				temp = debugRS.getJointRelativeOrientation(character->getJointIndex(j->name)) * temp;
			}
	
			temp = qRootD * temp;
			temp = temp.getComplexConjugate() * debugRS.getJointRelativeOrientation(i);
			debugRS.setJointRelativeOrientation(temp, i);
		}
	}
}



/**
	This method is used to return a pointer to a rigid body, based on its name and the current stance of the character
*/
RigidBody* SimBiController::getRBBySymbolicName(char* sName){
	RigidBody* result;
	char resolvedName[100];
	//deal with the SWING/STANCE_XXX' case
	if (strncmp(sName , "SWING_", strlen("SWING_"))==0){
		strcpy(resolvedName+1, sName + strlen("SWING_"));
		if (stance == LEFT_STANCE)
			resolvedName[0] = 'r';
		else
			resolvedName[0] = 'l';
	}else
		if (strncmp(sName , "STANCE_", strlen("STANCE_"))==0){
			strcpy(resolvedName+1, sName + strlen("STANCE_"));
			if (stance == LEFT_STANCE)
				resolvedName[0] = 'l';
			else
				resolvedName[0] = 'r';
	}else
		strcpy(resolvedName, sName);

	result = character->getARBByName(resolvedName);

	if (result == NULL)
		throwError("Could not find RB \'%s\\%s\'\n", resolvedName, sName);

	return result;

}


/**
updates the indexes of the swing and stance hip, knees and ankles
*/
void SimBiController::updateSwingAndStanceReferences(){
	stanceHipIndex = ((stance == LEFT_STANCE) ? (lHipIndex) : (rHipIndex));
	swingHipIndex = ((stance == LEFT_STANCE) ? (rHipIndex) : (lHipIndex));
	swingKneeIndex = ((stance == LEFT_STANCE) ? (rKneeIndex) : (lKneeIndex));
	stanceKneeIndex = ((stance == LEFT_STANCE) ? (lKneeIndex) : (rKneeIndex));
	stanceAnkleIndex = ((stance == LEFT_STANCE) ? (lAnkleIndex) : (rAnkleIndex));
	swingAnkleIndex = ((stance == LEFT_STANCE) ? (rAnkleIndex) : (lAnkleIndex));

	swingHip = character->joints[swingHipIndex];
	swingKnee = character->joints[swingKneeIndex];
}


/**
	This method is used to compute the torques that are to be applied at the next step.
*/
void SimBiController::computeTorques(DynamicArray<ContactPoint> *cfs, std::map<uint, WaterImpact>& resulting_impact){
	if (FSMStateIndex >= (int)states.size()){
		tprintf("Warning: no FSM state was selected in the controller!\n");
		return;
	}

	//first I check if we are near to fall (or even already on the ground
	double h = character->getRoot()->getCMPosition().y;

	double hMax = 0.4;
	double hMin = 0.2;

	if (h > hMax)
		h = hMax;

	if (h < hMin)
		h = hMin;

	h = (h - hMin) / (hMax - hMin);

	//if we are on the ground I can just skip everything
	if (h <= 0.001){
		for (uint i = 0; i < torques.size(); i++){
			torques[i] = Vector3d(0, 0, 0);
		}
		return;
	}

	//we create the interface to modify the target pose
	ReducedCharacterState poseRS(&desiredPose);
	//and this is the desired orientation for the root
	Quaternion qRootD(1, 0, 0, 0);


	//we compute the basic target
	evaluateJointTargets(poseRS,qRootD);

	//we check the nbr of feet on the ground
	stanceHipToSwingHipRatio = getStanceFootWeightRatio(cfs);
	if (stanceHipToSwingHipRatio < 0){
		stance_mode = -1;
	}
	else if (stanceHipToSwingHipRatio < 0.999){
		stance_mode = 1;
	}
	else{
		stance_mode = 0;
	}

	//since I use the foot trajectory now I have to compute the positions
	//we do it only in the ascendent phase (in the decendent phase we use the IPM result to conserve balance)
	//TODO also use specified result during static phase
	if (!ipm_used()){
		use_specified_swing_foot(SimGlobals::dt);
	}


	//Now we modify the target of the swing leg to follow the IPM (or the specified values)
	computeIKSwingLegTargets(SimGlobals::dt);
	
		

	//compute the torques now, using the desired pose information - the hip torques will get overwritten below
	PoseController::computeTorques(cfs,swingHipIndex,resulting_impact);
	

	//bubble-up the torques computed from the PD controllers
	bubbleUpTorques();


	//we'll also compute the torques that cancel out the effects of gravity, for better tracking purposes
	computeGravityCompensationTorques(resulting_impact);


	//now we can add the torques to control the speed
	Vector3d ffRootTorque(0, 0, 0);

	if (cfs->size() > 0){
		COMJT(cfs,ffRootTorque);
	}

	if (stanceHipToSwingHipRatio < 0)
		rootControlParams.strength = 0;

	//and now separetely compute the torques for the hips - together with the feedback term, this is what defines simbicon
	computeHipTorques(qRootD, poseRS.getJointRelativeOrientation(swingHipIndex), stanceHipToSwingHipRatio,ffRootTorque);

	int stance_factor = 1;
	if (getStance() == RIGHT_STANCE){
		stance_factor = -1;
	}

	//this is a test here I'll try to simulate the foot rotation by adding torques on some joints
	double cur_phi = getPhase();


	//here I'l start the stance foot control system

	
	//this is a ponderation if we are near to fall
	for (uint i=0;i<torques.size();i++){
		torques[i] = torques[i] * h;// +Vector3d(0, 0, 0) * (1 - h);
	}
}


void SimBiController::evaluateJointTargets(ReducedCharacterState& poseRS,Quaternion& qRootD){

	
	//d and v are specified in the rotation (heading) invariant coordinate frame
	//already done before
	//updateDAndV();

	//there are two stages here. First we will compute the pose (i.e. relative orientations), using the joint trajectories for the current state
	//and then we will compute the PD torques that are needed to drive the links towards their orientations - here the special case of the
	//swing and stance hips will need to be considered

	//always start from a neutral desired pose, and build from there...
	for (int i = 0; i<jointCount; i++){
		poseRS.setJointRelativeOrientation(Quaternion(1, 0, 0, 0), i);
		poseRS.setJointRelativeAngVelocity(Vector3d(), i);
		controlParams[i].controlled = true;
		controlParams[i].relToCharFrame = false;
	}

	double phiToUse = getPhase();
	//make sure that we only evaluate trajectories for phi values between 0 and 1
	if (phiToUse>1)
		phiToUse = 1;

	SimBiConState* curState = states[FSMStateIndex];
	Quaternion newOrientation;

	//get the estimated global trajectory 
	//I noticed that those values are always equal to 0 (a least on the forward walk...)
	Vector3d d0, v0;
	computeD0(phiToUse, &d0);
	computeV0(phiToUse, &v0);

	//just precompute the real D and v (couting the base trajectory)
	Vector3d d_d = d - d0;
	Vector3d d_v = v - v0;

	for (int i = 0; i<curState->getTrajectoryCount(); i++){
		//now we have the desired rotation angle and axis, so we need to see which joint this is intended for
		int jIndex = curState->sTraj[i]->getJointIndex(stance);

		//anything loer than -1 is just a trajectory stored here for a simple usage
		//but it's not a real tajectory so I have to ignore them
		if (jIndex < -1){
			continue;
		}

		//get the desired joint orientation to track - include the feedback if necessary/applicable
		newOrientation = curState->sTraj[i]->evaluateTrajectory(this, character->getJoint(jIndex), stance, phiToUse, d_d, d_v);

		//if the index is -1, it must mean it's the root's trajectory. Otherwise we should give an error
		if (jIndex == -1){
			qRootD = newOrientation;
			rootControlParams.strength = curState->sTraj[i]->evaluateStrength(phiToUse);
		}
		else{
			if (curState->sTraj[i]->relToCharFrame == true || jIndex == swingHipIndex){
				controlParams[jIndex].relToCharFrame = true;
				controlParams[jIndex].charFrame = characterFrame;
			}
			poseRS.setJointRelativeOrientation(newOrientation, jIndex);
			controlParams[jIndex].strength = curState->sTraj[i]->evaluateStrength(phiToUse);
		}
	}

}

/**
This method is used to ensure that each RB sees the net torque that the PD controller computed for it.
Without it, an RB sees also the sum of -t of every child.
*/
void SimBiController::bubbleUpTorques(){
	for (int i = character->joints.size() - 1; i >= 0; i--){
		if (i != stanceHipIndex && i != stanceKneeIndex)
			if (character->joints[i]->getParent() != root)
				torques[character->joints[i]->getParent()->getParentJoint()->get_idx()] += torques[i];
	}
}

/**
This method computes the torques that cancel out the effects of gravity,
for better tracking purposes
*/
void SimBiController::computeGravityCompensationTorques(std::map<uint, WaterImpact>& resulting_impact){
	vmc->resetTorques();
	for (uint i = 0; i<character->joints.size(); i++){
		if (i != stanceHipIndex && i != stanceKneeIndex && i != stanceAnkleIndex){
			//I need to diminish the force depending on the water boyancy

			Point3d pt1=character->joints[i]->getChild()->getCMPosition();
			Vector3d F1 = Vector3d(0, character->joints[i]->child->props.mass*9.8, 0);

			if (resulting_impact.find(character->joints[i]->get_idx())!=resulting_impact.end()){
				ForceStruct  boyancy = resulting_impact[character->joints[i]->get_idx()].boyancy;
				if (!boyancy.F.isZeroVector()){
					Vector3d vect_support = pt1 - character->joints[i]->getChild()->getWorldCoordinates(boyancy.pt);
					double L2 = vect_support.length()*boyancy.F.length() / (F1.length() - boyancy.F.length());
					pt1 = pt1 + vect_support / vect_support.length()*L2;
					F1 -= boyancy.F;
				}
			}
			pt1 = character->joints[i]->getChild()->getLocalCoordinates(pt1);

			vmc->computeJointTorquesEquivalentToForce(character->joints[i], pt1, F1, NULL);

		}
	}

	for (uint i = 0; i<character->joints.size(); i++){
		torques[i] += vmc->getTorque(i);
	}
}


void SimBiController::COMJT(DynamicArray<ContactPoint> *cfs, Vector3d& ffRootTorque){
	

	if (stance_mode < 0){
		return;
	}

	//ArticulatedRigidBody* foot = character->joints[stanceAnkleIndex]->child;
	//getForceInfoOn(foot, cfs, &stanceHeelInContact, &stanceToeInContact);

	//in the case of the dual stance mode (I'll apply the torques proportionally to their impact to the ground)
	computeLegTorques(stanceAnkleIndex, stanceKneeIndex, stanceHipIndex, cfs, ffRootTorque, stanceHipToSwingHipRatio);

	//*
	if (stance_mode == 1){
		computeLegTorques(swingAnkleIndex, swingKneeIndex, swingHipIndex, cfs, ffRootTorque, 1.0-stanceHipToSwingHipRatio);
	}//*/
}


/**
This method is used to compute the force that the COM of the character should be applying.
*/
Vector3d SimBiController::computeVirtualForce(){
	//this is the desired acceleration of the center of mass
	Vector3d desA = Vector3d();

	Vector3d vw = characterFrame.rotate(v);


	double eff_velDSagittal = get_effective_desired_sagittal_velocity(phi); 
	double eff_velDCoronal = get_effective_desired_coronal_velocity(phi);

	desA.z = (eff_velDSagittal - vw.z) *30;// +SimGlobals::force_alpha / 1000);
	desA.x = (eff_velDCoronal - vw.x) * 20;// +(comOffsetCoronal - d.x) * 2;
	//desA.x = (-d.x + comOffsetCoronal) * 20 + (velDCoronal - v.x) * 9;

	if (stance == 1){
		//Vector3d errV = characterFrame.inverseRotate(doubleStanceCOMError*-1);
		//desA.x = (-errV.x + comOffsetCoronal) * 20 + (velDCoronal - v.x) * 9;
		//desA.z = (-errV.z + comOffsetSagittal) * 10 + (velDSagittal - v.z) * 150;
	}

	//and this is the force that would achieve that - make sure it's not too large...
	Vector3d fA = (desA)* character->getAF()->getMass(); 
	boundToRange(&fA.x, -100, 100);
	boundToRange(&fA.z, -200, 200);

	//now change this quantity to world coordinates...
	fA = characterFrame.rotate(fA);

	return fA;
}


/**
This method returns performes some pre-processing on the virtual torque. The torque is assumed to be in world coordinates,
and it will remain in world coordinates.
*/
void SimBiController::preprocessAnkleVTorque(int ankleJointIndex, DynamicArray<ContactPoint> *cfs, Vector3d *ankleVTorque){
	ForceStruct heel_force,front_foot_force, toe_force;
	ArticulatedRigidBody* foot = character->joints[ankleJointIndex]->child;
	getForceInfoOn(foot, cfs, heel_force, front_foot_force, toe_force);
	*ankleVTorque = foot->getLocalCoordinates(*ankleVTorque);

	if (front_foot_force.F.isZeroVector() || getPhase() < 0.2 || getPhase() > 0.8){
		ankleVTorque->x = 0;
	}

	Vector3d footRelativeAngularVel = foot->getLocalCoordinates(foot->getAngularVelocity());
	if ((footRelativeAngularVel.z < -0.2 && ankleVTorque->z > 0) || (footRelativeAngularVel.z > 0.2 && ankleVTorque->z < 0)){
		ankleVTorque->z = 0;
	}

	if (fabs(footRelativeAngularVel.z) > 1.0){
		ankleVTorque->z = 0;
	}

	if (fabs(footRelativeAngularVel.x) > 1.0){
		ankleVTorque->x = 0;
	}

	if ((!heel_force.F.isZeroVector())&&(front_foot_force.F.isZeroVector())){
		ankleVTorque->x = 0;
		ankleVTorque->z = 0;
	}

	boundToRange(&ankleVTorque->z, -20, 20);

	*ankleVTorque = foot->getWorldCoordinates(*ankleVTorque);
}

/**
determines if there are any heel/toe forces on the given RB
the rigid body passed in the parameters HAVE to be a foot
*/
void SimBiController::getForceInfoOn(RigidBody* rb, DynamicArray<ContactPoint> *cfs, ForceStruct& heelForce, ForceStruct& frontFeetForce,
												ForceStruct& toeForce){

	//I will compte the resulting force of the ground on the heel, the front foot and the toes
	RigidBody* toe_body = static_cast<ArticulatedRigidBody*>(rb)->getChildJoints().front()->getChild();

	Point3d tmpP;

	for (uint i = 0; i<cfs->size(); i++){
		if (((*cfs)[i].rb1 == rb) || ((*cfs)[i].rb2== rb)){
			tmpP = rb->getLocalCoordinates((*cfs)[i].cp);
			if (tmpP.z < 0){
				heelForce.F += Vector3d(0, std::abs((*cfs)[i].f.y), 0);
			}
			
			if (tmpP.z > 0){
				frontFeetForce.F += Vector3d(0, std::abs((*cfs)[i].f.y), 0);
			}
		}
		else if (((*cfs)[i].rb1 == toe_body) || ((*cfs)[i].rb2 == toe_body)){
			if (toeForce.pt == Point3d(0, 0, 0)){			
				tmpP = rb->getLocalCoordinates((*cfs)[i].cp);
				toeForce.pt = tmpP;
			}
			toeForce.F += (*cfs)[i].f;
		}
	}
}


/**
check to see if rb is the same as whichBody or any of its children
*/
bool SimBiController::haveRelationBetween(RigidBody* rb, RigidBody* whichBody){
	//check against the feet
	if (rb == whichBody)
		return true;
	for (uint j = 0; j<((ArticulatedRigidBody*)whichBody)->cJoints.size(); j++)
		if (((ArticulatedRigidBody*)whichBody)->cJoints[j]->child == rb)
			return true;
	return false;
}


/**
This method is used to compute torques for the stance leg that help achieve a desired speed in the sagittal and lateral planes
*/
void SimBiController::computeLegTorques(int ankleIndex, int kneeIndex, int hipIndex, DynamicArray<ContactPoint> *cfs,Vector3d& ffRootTorque,
			double leg_ratio){
	
	/*bool heelInContact, toeInContact;
	ArticulatedRigidBody* foot = character->joints[ankleIndex]->child;
	getForceInfoOn(foot, cfs, &heelInContact, &toeInContact);
	if (heelInContact && (!toeInContact)){
		return;
	}*/

	//applying a force at the COM induces the force f. The equivalent torques are given by the J' * f, where J' is
	// dp/dq, where p is the COM.

	//int lBackIndex = character->getJointIndex("pelvis_lowerback");
	//int mBackIndex = character->getJointIndex("lowerback_torso");
	int backIndex = character->getJointIndex("pelvis_torso");

	double m = 0;
	ArticulatedRigidBody* tibia = character->joints[ankleIndex]->parent;
	ArticulatedRigidBody* femur = character->joints[kneeIndex]->parent;
	ArticulatedRigidBody* pelvis = character->joints[hipIndex]->parent;
	ArticulatedRigidBody* back = character->joints[backIndex]->child;

	Point3d anklePos = character->joints[ankleIndex]->child->getWorldCoordinates(character->joints[ankleIndex]->cJPos);
	Point3d kneePos = character->joints[kneeIndex]->child->getWorldCoordinates(character->joints[kneeIndex]->cJPos);
	Point3d hipPos = character->joints[hipIndex]->child->getWorldCoordinates(character->joints[hipIndex]->cJPos);
	Point3d backPos = character->joints[backIndex]->child->getWorldCoordinates(character->joints[backIndex]->cJPos);

	//total mass...
	m = tibia->props.mass + femur->props.mass + pelvis->props.mass + back->props.mass;

	Vector3d fA = computeVirtualForce();

	//*
	//this can be used to show the forces
	ForceStruct show_force;
	show_force.F = fA;
	show_force.pt = character->getCOM();
	SimGlobals::vect_forces.push_back(show_force);
	//*/

	Vector3d f1 = Vector3d(anklePos, tibia->state.position) * tibia->props.mass +
		Vector3d(anklePos, femur->state.position) * femur->props.mass +
		Vector3d(anklePos, pelvis->state.position) * pelvis->props.mass;// +
		//Vector3d(anklePos, back->state.position) * back->props.mass;
	f1 /= m;

	Vector3d f2 = Vector3d(kneePos, femur->state.position) * femur->props.mass +
		Vector3d(kneePos, pelvis->state.position) * pelvis->props.mass;// +
		//Vector3d(kneePos, back->state.position) * back->props.mass;
	f2 /= m;

	Vector3d f3 = Vector3d(hipPos, pelvis->state.position) * pelvis->props.mass;
		//Vector3d(hipPos, back->state.position) * back->props.mass;
	f3 /= m;

	Vector3d f4 = Vector3d(backPos, back->state.position) * back->props.mass;
	f4 /= m;


	//I'll separate the 2 component for the force to see if I can learn anything ...
	//I'll start with the z component
	double x_back = fA.x;
	fA.x = 0;

	Vector3d torque = f1.crossProductWith(fA)*leg_ratio;
	preprocessAnkleVTorque(ankleIndex, cfs, &torque);
	torque.y = 0;
	torques[ankleIndex] += torque;

	torque = f2.crossProductWith(fA)*leg_ratio;
	torque.y = 0;
	torques[kneeIndex] += torque;

	torque = f3.crossProductWith(fA)*leg_ratio;
	torque.y = 0;
	torques[hipIndex] += torque;
	//the torque on the stance hip is cancelled out, so pass it in as a torque that the root wants to see!
	ffRootTorque -= torque;

	torque= f4.crossProductWith(fA)*leg_ratio;
	torque.y = 0;
	torques[backIndex] -= torque;
	ffRootTorque -= torque;

	//*
	//Now I'll study the component following x


	fA.z=0;
	fA.x=x_back*SimGlobals::time_factor;

	torque = f1.crossProductWith(fA)*leg_ratio;
	preprocessAnkleVTorque(ankleIndex, cfs, &torque);
	torque.y = 0;
	torques[ankleIndex] += torque;
	
	torque = f2.crossProductWith(fA)*leg_ratio;
	torque.y = 0;
	torques[kneeIndex] += torque;

	torque = f3.crossProductWith(fA)*leg_ratio;
	torque.y = 0;
	torques[hipIndex] += torque;
	//the torque on the stance hip is cancelled out, so pass it in as a torque that the root wants to see!
	ffRootTorque -= torque;

	torque= f4.crossProductWith(fA)*leg_ratio*0.5;
	torque.y = 0;
	torques[backIndex] -= torque;
	ffRootTorque -= torque;
	
	//*/



	
	/*
	//old code
	Vector3d fA = computeVirtualForce();

	Vector3d r;

	Point3d p = character->getCOM();

	r.setToVectorBetween(character->joints[ankleIndex]->child->getWorldCoordinates(character->joints[ankleIndex]->cJPos), p);

	Vector3d ankleTorque = r.crossProductWith(fA);
	preprocessAnkleVTorque(ankleIndex, cfs, &ankleTorque);
	torques[ankleIndex] += ankleTorque;

	r.setToVectorBetween(character->joints[kneeIndex]->child->getWorldCoordinates(character->joints[kneeIndex]->cJPos), p);
	torques[kneeIndex] += r.crossProductWith(fA);

	r.setToVectorBetween(character->joints[hipIndex]->child->getWorldCoordinates(character->joints[hipIndex]->cJPos), p);
	torques[hipIndex] += r.crossProductWith(fA);

	//the torque on the stance hip is cancelled out, so pass it in as a torque that the root wants to see!
	ffRootTorque -= r.crossProductWith(fA);

	//int lBackIndex = character->getJointIndex("pelvis_lowerback");
	//r.setToVectorBetween(character->joints[lBackIndex]->child->getWorldCoordinates(character->joints[lBackIndex]->cJPos), p);
	//torques[lBackIndex] += r.crossProductWith(fA) / 10;

	//int mBackIndex = character->getJointIndex("lowerback_torso");
	//r.setToVectorBetween(character->joints[mBackIndex]->child->getWorldCoordinates(character->joints[mBackIndex]->cJPos), p);
	//torques[mBackIndex] += r.crossProductWith(fA) / 10;

	int backIndex = character->getJointIndex("pelvis_torso");
	r.setToVectorBetween(character->joints[backIndex]->child->getWorldCoordinates(character->joints[backIndex]->cJPos), p);
	torques[backIndex] += r.crossProductWith(fA) / 5;
	//*/
}

/**
this function override the results of the ipm with the specified results
*/
void SimBiController::use_specified_swing_foot(double dt){
	double cur_phi = MIN(getPhase(), 1);
	double future_phi = MIN(cur_phi + dt / get_cur_state_time(), 1);

	//we know that the points what we want to modify are the 2 last that were defined
	//since we specify the position relative to the hip knot and that the system is based on positions relatives to the mass center
	//we need to add the bifference before giving the values to the system
	Point3d com = character->getCOM();
	Point3d hip = swingHip->getParent()->getWorldCoordinates(swingHip->getParentJointPosition());
	double dx = hip.x - com.x;
	double dz = hip.z - com.z;

	int point_count = swingFootTrajectorySagittal.getKnotCount();
	swingFootTrajectorySagittal.setKnotValue(point_count - 2, swing_foot_traj->components[2]->baseTraj.evaluate_catmull_rom(cur_phi)+dz);
	swingFootTrajectorySagittal.setKnotValue(point_count - 1, swing_foot_traj->components[2]->baseTraj.evaluate_catmull_rom(future_phi) + dz);


	double sign = (getStance() == LEFT_STANCE) ? -1.0 : 1.0;
	point_count = swingFootTrajectoryCoronal.getKnotCount();
	swingFootTrajectoryCoronal.setKnotValue(point_count - 2, sign*swing_foot_traj->components[0]->baseTraj.evaluate_catmull_rom(cur_phi) + dx);
	swingFootTrajectoryCoronal.setKnotValue(point_count - 1, sign*swing_foot_traj->components[0]->baseTraj.evaluate_catmull_rom(future_phi) + dx);


	//the height will be handled somewhere else( because it has to be always generated)


}

/**
This method is used to compute the target angles for the swing hip and swing knee that help
to ensure (approximately) precise foot-placement control.
*/
void SimBiController::computeIKSwingLegTargets(double dt){
	Point3d pNow, pFuture;

	Point3d com_pos = character->getCOM();

	//note, V is already expressed in character frame coordinates.
	pNow = getSwingFootTargetLocation(getPhase(), com_pos, characterFrame);
	pFuture = getSwingFootTargetLocation(MIN(getPhase() + dt / get_cur_state_time(), 1), com_pos + character->getCOMVelocity() * dt, characterFrame);


	Vector3d parentAxis(character->joints[swingHipIndex]->cJPos, character->joints[swingKneeIndex]->pJPos);
	Vector3d childAxis(character->joints[swingKneeIndex]->cJPos, character->joints[swingAnkleIndex]->pJPos);

	computeIKQandW(swingHipIndex, swingKneeIndex, parentAxis, swingLegPlaneOfRotation, Vector3d(-1, 0, 0), childAxis, pNow, true, pFuture, dt);
	//	computeIKQandW(swingHipIndex, swingKneeIndex, Vector3d(0, -0.355, 0), Vector3d(1,0,0), Vector3d(1,0,0), Vector3d(0, -0.36, 0), pNow, true, pFuture, dt);
}

/**
This method returns a target for the location of the swing foot, based on some state information. It is assumed that the velocity vel
is expressed in character-relative coordinates (i.e. the sagittal component is the z-component), while the com position, and the
initial swing foot position is expressed in world coordinates. The resulting desired foot position is also expressed in world coordinates.
*/
Point3d SimBiController::getSwingFootTargetLocation(double t, const Point3d& com, const Quaternion& charFrameToWorld){
	Vector3d step;
	step.z = swingFootTrajectorySagittal.evaluate_catmull_rom(t);
	step.x = swingFootTrajectoryCoronal.evaluate_catmull_rom(t);
	//now transform this vector into world coordinates
	step = charFrameToWorld.rotate(step);
	//add it to the com location
	step = com + step;
	//finally, set the desired height of the foot
	//TODO reactivate the panicheight an the unplanned height
	Point3d hip_pos = swingHip->getParent()->getWorldCoordinates(swingHip->getParentJointPosition());
	double delta_y=swing_foot_traj->components[1]->baseTraj.evaluate_catmull_rom(t);
	step.y = hip_pos.y - delta_y; // + panicHeight + unplannedForHeight;
	

	step += computeSwingFootDelta(t);

	return step;
}

Vector3d SimBiController::computeSwingFootDelta(double phiToUse, int stanceToUse) {

	if (phiToUse < 0)
		phiToUse = getPhase();
	if (phiToUse > 1)
		phiToUse = 1;
	if (stanceToUse < 0)
		stanceToUse = stance;
	if (stanceToUse > 1)
		stanceToUse = 1;

	double sign = (stanceToUse == LEFT_STANCE) ? 1.0 : -1.0;
	return Vector3d(
		swingFootTrajectoryDeltaCoronal.evaluate_catmull_rom(phiToUse) * sign,
		swingFootTrajectoryDeltaHeight.evaluate_catmull_rom(phiToUse),
		swingFootTrajectoryDeltaSagittal.evaluate_catmull_rom(phiToUse));
}


/**
This method is used to compute the desired orientation and angular velocity for a parent RB and a child RB, relative to the grandparent RB and
parent RB repsectively. The input is:
- the index of the joint that connects the grandparent RB to the parent RB, and the index of the joint between parent and child

- the distance from the parent's joint with its parent to the location of the child's joint, expressed in parent coordinates

- two rotation normals - one that specifies the plane of rotation for the parent's joint, expressed in grandparent coords,
and the other specifies the plane of rotation between the parent and the child, expressed in parent's coordinates.

- The position of the end effector, expressed in child's coordinate frame

- The desired position of the end effector, expressed in world coordinates

- an estimate of the desired position of the end effector, in world coordinates, some dt later - used to compute desired angular velocities
*/
void SimBiController::computeIKQandW(int parentJIndex, int childJIndex, const Vector3d& parentAxis, const Vector3d& parentNormal, 
											const Vector3d& childNormal, const Vector3d& childEndEffector, const Point3d& wP,
											bool computeAngVelocities, const Point3d& futureWP, double dt){
	//this is the joint between the grandparent RB and the parent
	Joint* parentJoint = character->joints[parentJIndex];
	//this is the grandparent - most calculations will be done in its coordinate frame
	ArticulatedRigidBody* gParent = parentJoint->parent;
	//this is the reduced character space where we will be setting the desired orientations and ang vels.
	ReducedCharacterState rs(&desiredPose);

	//the desired relative orientation between parent and grandparent
	Quaternion qParent;
	//and the desired relative orientation between child and parent
	Quaternion qChild;


	TwoLinkIK::getIKOrientations(parentJoint->getParentJointPosition(), gParent->getLocalCoordinates(wP), parentNormal, parentAxis,
		childNormal, childEndEffector, &qParent, &qChild);

	//TODO activate those 2 lines and disactivate the 2 after
	//I transformed the line so they fit in this code but I dont realy know what the frame is used for in the 2010 version of the code
	//controlParams[parentJIndex].relToFrame = false;
	//controlParams[childJIndex].relToFrame = false;
	controlParams[parentJIndex].relToCharFrame = false;
	controlParams[childJIndex].relToCharFrame = false;
	/*
	controlParams[parentJIndex].controlled = true;
	controlParams[childJIndex].controlled = true;
	controlParams[parentJIndex].strength = 1.0;
	controlParams[childJIndex].strength = 1.0;
	//*/
	
	rs.setJointRelativeOrientation(qChild, childJIndex);
	rs.setJointRelativeOrientation(qParent, parentJIndex);


	Vector3d wParentD(0, 0, 0);
	Vector3d wChildD(0, 0, 0);

	if (computeAngVelocities){
		//the joint's origin will also move, so take that into account, by subbing the offset by which it moves to the
		//futureTarget (to get the same relative position to the hip)
		Vector3d velOffset = gParent->getAbsoluteVelocityForLocalPoint(parentJoint->getParentJointPosition());

		Quaternion qParentF;
		Quaternion qChildF;
		TwoLinkIK::getIKOrientations(parentJoint->getParentJointPosition(), gParent->getLocalCoordinates(futureWP + velOffset * -dt), parentNormal, parentAxis, childNormal, childEndEffector, &qParentF, &qChildF);

		Quaternion qDiff = qParentF * qParent.getComplexConjugate();
		wParentD = qDiff.v * 2 / dt;
		//the above is the desired angular velocity, were the parent not rotating already - but it usually is, so we'll account for that
		wParentD -= gParent->getLocalCoordinates(gParent->getAngularVelocity());

		qDiff = qChildF * qChild.getComplexConjugate();
		wChildD = qDiff.v * 2 / dt;

		//make sure we don't go overboard with the estimates, in case there are discontinuities in the trajectories...
		boundToRange(&wChildD.x, -5, 5); boundToRange(&wChildD.y, -5, 5); boundToRange(&wChildD.z, -5, 5);
		boundToRange(&wParentD.x, -5, 5); boundToRange(&wParentD.y, -5, 5); boundToRange(&wParentD.z, -5, 5);
	}

	rs.setJointRelativeAngVelocity(wChildD, childJIndex);
	rs.setJointRelativeAngVelocity(wParentD, parentJIndex);
}


/**
	This method is used to compute the torques that need to be applied to the stance and swing hips, given the
	desired orientation for the root and the swing hip.
*/
void SimBiController::computeHipTorques(const Quaternion& qRootD, const Quaternion& qSwingHipD, double stanceHipToSwingHipRatio, Vector3d& ffRootTorque){
	//compute the total torques that should be applied to the root and swing hip, keeping in mind that
	//the desired orientations are expressed in the character frame
	Vector3d rootTorque;
	Vector3d swingHipTorque;

	//this is the desired orientation in world coordinates
	Quaternion qRootDW;

	//if (SimGlobals::forceHeadingControl == false){
	//qRootD is specified in the character frame, so just maintain the current heading
	//qRootDW = qRootD;
	//qRootDW = characterFrame * qRootD;
	//}else{
	//qRootDW needs to also take into account the desired heading
	qRootDW = Quaternion::getRotationQuaternion(SimGlobals::desiredHeading, SimGlobals::up) * qRootD;
	//}

	double rootStrength = rootControlParams.strength;
	if (rootStrength < 0)
		rootStrength = 0;
	if (rootStrength > 1)
		rootStrength = 1;

	rootControlParams.strength = 1;

	//so this is the net torque that the root wants to see, in world coordinates
	rootTorque = computePDTorque(root->getOrientation(), qRootDW, root->getAngularVelocity(), Vector3d(0, 0, 0), &rootControlParams);

	rootTorque += ffRootTorque;

	RigidBody* swingHip = character->getJoint(swingHipIndex)->getChild();

	swingHipTorque = torques[swingHipIndex];

	//we need to compute the total torque that needs to be applied at the hips so that the final net torque on the root is rootTorque
	Vector3d rootMakeupTorque;
	for (int i = 0; i < jointCount; i++){
		if (character->getJoint(i)->getParent() == root){
			rootMakeupTorque -= torques[i];
		}
	}
	rootMakeupTorque -= rootTorque;

	//add to the root makeup torque the predictive torque as well (only consider the effect of the torque in the lateral plane).
	Vector3d rootPredictiveTorque(0, 0, rootPredictiveTorqueScale*9.8*d.x);///WARNING seems strange that there is an x here but well...
	rootMakeupTorque += characterFrame.rotate(rootPredictiveTorque);

	//assume the stance foot is in contact...
	Vector3d stanceHipTorque = torques[stanceHipIndex];
	double stance_hip_y = stanceHipTorque.y;

	//now, based on the ratio of the forces the feet exert on the ground, and the ratio of the forces between the two feet, we will compute the forces that need to be applied
	//to the two hips, to make up the root makeup torque - which means the final torque the root sees is rootTorque!
	stanceHipTorque += rootMakeupTorque * stanceHipToSwingHipRatio * rootStrength;
	swingHipTorque += rootMakeupTorque * (1-stanceHipToSwingHipRatio) * rootStrength;


	if( stanceHipDamping > 0 ) {
		//the commented version is the one from the 2007 code. The uncommented one is the one from the 2010 code
		//double wRely = root->getAngularVelocity().y - character->joints[stanceHipIndex]->child->getAngularVelocity().y;
		//if (fabs(wRely) > stanceHipMaxVelocity ) wRely = stanceHipMaxVelocity * SGN(wRely);
		//stanceHipTorque.y -= stanceHipDamping * wRely * wRely * SGN(wRely);

		Vector3d wRel = root->getAngularVelocity() - character->joints[stanceHipIndex]->child->getAngularVelocity();
		double wRelLen = wRel.length();
		if (wRelLen > stanceHipMaxVelocity) wRel = wRel * (stanceHipMaxVelocity / wRelLen);
		stanceHipTorque -= wRel * (stanceHipDamping * wRelLen);
	}

	//the folowing commented code correspond to tries where I amplified the swing hip torque to keep the balance
	//but it don't feel all that correct to do....
	//Vector3d v1 =v;
	//v1.y = 0;
	//if (v1.length() > 0.1){
		//int idx=character->getJointIndex("pelvis_torso");
	
		//Vector3d v2 = Vector3d(0, 1, 0);//y is the vertical vector int this world
		//Vector3d n = v1.crossProductWith(v2);
		//n/=-n.length();
		//n.z= 0;
		//n = Vector3d(1, 0, 0);

		//here I'll try something to counter the effect of the water
		//swingHipTorque += swingHipTorque*SimGlobals::force_alpha / 30000 * SimGlobals::water_level;

		//swingHipTorque.x += swingHipTorque.x*SimGlobals::liquid_density / 1000 * 2;

		
		//stanceHipTorque -= stanceHipTorque*SimGlobals::force_alpha / 30000 * 2;
		//swingHipTorque = Vector3d(0,0,0);
		//Vector3d vv = torques[idx];
		//torques[idx] += -n*SimGlobals::force_alpha / 30000 * 200;
		//stanceHipTorque += n*SimGlobals::force_alpha / 30000 * 200;
		//swingHipTorque += n*SimGlobals::force_alpha / 30000 * 60;
		//	stanceHipTorque = n*0;
	//}


	//now transform the torque to child coordinates, apply torque limits and then change it back to world coordinates
	Quaternion qStanceHip = character->getJoint(stanceHipIndex)->getChild()->getOrientation();
	stanceHipTorque = qStanceHip.getComplexConjugate().rotate(stanceHipTorque);
	limitTorque(&stanceHipTorque, &controlParams[stanceHipIndex]);


	

	/*if (stanceHeelInContact){
		Vector3d ang_speed=character->getJoint(stanceAnkleIndex)->getChild()->getAngularVelocity();
		if (std::abs(ang_speed.y) > 3){
			stanceHipTorque.y = stanceHipTorque.y;
		}
	}*/

	//stanceHipTorque.y *= 0.1;
	stanceHipTorque = qStanceHip.rotate(stanceHipTorque);
	

	Quaternion qSwingHip = character->getJoint(swingHipIndex)->getChild()->getOrientation();
	swingHipTorque = qSwingHip.getComplexConjugate().rotate(swingHipTorque);
	limitTorque(&swingHipTorque, &controlParams[swingHipIndex]);
	swingHipTorque = qSwingHip.rotate(swingHipTorque);

	//and done...
	torques[stanceHipIndex] = stanceHipTorque;
	torques[swingHipIndex] = swingHipTorque;

}

/**
this method us used to control the interaction beetween the stance foot and the ground
it has 2 functionalities.
first it store a curve updating how the interactions beetween the foot and the ground should be
second it make sure that the contact of the foot with the ground is real.
*/
void SimBiController::foot_contact_control(){
	//this function should not do anything if we are not in a phase of foot contact
	if (getPhase() < 0.14){
		return;
	}

	//so first I need the ground force I can attribute with each corner of the foot
}


/**
	This method is used to obtain the d and v parameters, using the current postural information of the biped
*/
void SimBiController::updateDAndV(){
	characterFrame = character->getHeading();


	Vector3d comPosition = character->getCOM();

	d = Vector3d(stanceFoot->getCMPosition(), comPosition);
	//d is now in world coord frame, so we'll represent it in the 'character frame'
	d = characterFrame.getComplexConjugate().rotate(d);
	//compute v in the 'character frame' as well
	v = characterFrame.getComplexConjugate().rotate(character->getCOMVelocity());


	//and now compute the vector from the COM to the center of midpoint between the feet, again expressed in world coordinates
	Point3d feetMidpoint = (stanceFoot->getCMPosition() + swingFoot->getCMPosition());
	feetMidpoint /= 2.0;

	//now we have to compute the difference between the current COM and the desired COMPosition, in world coordinates
	doubleStanceCOMError = Vector3d(comPosition, feetMidpoint);
	//and add the user specified offset
	//	doubleStanceCOMError += characterFrame.rotate(Vector3d(SimGlobals::COMOffsetX, 0, SimGlobals::COMOffsetZ));
	doubleStanceCOMError.y = 0;
}

/**
	This method is used to resolve the names (map them to their index) of the joints.
*/
void SimBiController::resolveJoints(SimBiConState* state){
	char tmpName[100];
	for (uint i=0;i<state->sTraj.size();i++){
		Trajectory* jt = state->sTraj[i];
		//deal with the 'root' special case
		if (strcmp(jt->jName, "root") == 0){
			jt->leftStanceIndex = jt->rightStanceIndex = -1;
			continue;
		}
		//deal with the new swing foot trajectory specification
		if (strcmp(jt->jName, "swing_foot") == 0){
			jt->leftStanceIndex = jt->rightStanceIndex = -2;
			
			//we save a pointer for easy access
			swing_foot_traj = jt;
			continue;
		}

		//deal with the velD trajectory specification
		if (strcmp(jt->jName, "velD") == 0){
			jt->leftStanceIndex = jt->rightStanceIndex = -3;

			//we save a pointer for easy access
			velD_traj = jt;

			continue;
		}

		//deal with the SWING_XXX' case
		if (strncmp(jt->jName, "SWING_", strlen("SWING_"))==0){
			strcpy(tmpName+1, jt->jName + strlen("SWING_"));
			tmpName[0] = 'r';
			jt->leftStanceIndex = character->getJointIndex(tmpName);
			if (jt->leftStanceIndex<0)
				throwError("Cannot find joint %s\n", tmpName);
			tmpName[0] = 'l';
			jt->rightStanceIndex = character->getJointIndex(tmpName);
			if (jt->rightStanceIndex<0)
				throwError("Cannot find joint %s\n", tmpName);
			continue;
		}
		//deal with the STANCE_XXX' case
		if (strncmp(jt->jName, "STANCE_", strlen("STANCE_"))==0){
			strcpy(tmpName+1, jt->jName + strlen("STANCE_"));
			tmpName[0] = 'l';
			jt->leftStanceIndex = character->getJointIndex(tmpName);
			if (jt->leftStanceIndex<0)
				throwError("Cannot find joint %s\n", tmpName);
			tmpName[0] = 'r';
			jt->rightStanceIndex = character->getJointIndex(tmpName);
			if (jt->rightStanceIndex<0)
				throwError("Cannot find joint %s\n", tmpName);
			continue;
		}
		//if we get here, it means it is just the name...
		jt->leftStanceIndex = character->getJointIndex(jt->jName);
		if (jt->leftStanceIndex<0)
			throwError("Cannot find joint %s\n", jt->jName);
		jt->rightStanceIndex = jt->leftStanceIndex;
	}
}


/**
This method is used to write the current controller to a file
*/
void SimBiController::writeToFile(std::string fileName, std::string* stateFileName){
	char fname[100], statename[100];
	strcpy(fname, fileName.c_str());

	if (stateFileName != NULL){
		strcpy(statename, stateFileName->c_str());
		writeToFile(fname, statename);
	}
	else{
		writeToFile(fname);
	}
}

/**
	This method is used to write the details of the current controller to a file
*/
void SimBiController::writeToFile(char* fileName, char* stateFileName){
	FILE* f;
	if (fileName == NULL || (f = fopen(fileName, "w")) == NULL)
		return;

	fprintf( f, "%s\n", getConLineString(CON_PD_GAINS_START) );	
	fprintf( f, "#        joint name              Kp      Kd      MaxTorque    ScaleX        ScaleY        ScaleZ\n" );
	fprintf( f, "    root\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", 
					rootControlParams.kp,
					rootControlParams.kd,
					rootControlParams.maxAbsTorque,
					rootControlParams.scale.x,
					rootControlParams.scale.y,
					rootControlParams.scale.z );
	writeGains(f);

	fprintf( f, "%s\n", getConLineString(CON_PD_GAINS_END) );

	fprintf( f, "\n" );
	
	if( stanceHipDamping > 0 ) {
		fprintf( f, "%s %lf\n", getConLineString(CON_STANCE_HIP_DAMPING), stanceHipDamping );
		fprintf( f, "%s %lf\n", getConLineString(CON_STANCE_HIP_MAX_VELOCITY), stanceHipMaxVelocity );
	}

	fprintf( f, "\n" );
	

	for( uint i=0; i<states.size(); ++i ) {
	
		fprintf( f, "\n\n" );
		states[i]->writeState( f, i );

	}

	fprintf( f, "\n\n" );

	fprintf( f, "%s %d\n", getConLineString(CON_START_AT_STATE), startingState );
	fprintf( f, "%s %s\n", getConLineString(CON_STARTING_STANCE), 
		(stanceFoot == lFoot)?"left":"right" );
	if( stateFileName == NULL )
		fprintf( f, "%s %s\n", getConLineString(CON_CHARACTER_STATE), initialBipState );
	else
		fprintf( f, "%s %s\n", getConLineString(CON_CHARACTER_STATE), stateFileName );

	fclose(f);
}

/**
	This method is used to parse the information passed in the string. This class knows how to read lines
	that have the name of a joint, followed by a list of the pertinent parameters. If this assumption is not held,
	then classes extended this one are required to provide their own implementation of this simple parser
*/
void SimBiController::parseGainLine(char* line){
	double kp, kd, tMax, scX, scY, scZ;
	char jName[100];
	if (sscanf(line, "%s", jName) !=1)
		throwError("To specify the PD gains, you need tp specify the jointName/root --> \'%s\'", line);
	if (strcmp(jName, "root") == 0){
		if (sscanf(line, "%s %lf %lf %lf %lf %lf %lf\n", jName, &kp, &kd, &tMax, &scX, &scY, &scZ) !=7)
			throwError("To specify the PD gains, you need: 'jointName Kp Kd Tmax scaleX scaleY scaleZ'! --> \'%s\'", line);

		rootControlParams.kp = kp;
		rootControlParams.kd = kd;
		rootControlParams.maxAbsTorque = tMax;
		rootControlParams.scale = Vector3d(scX, scY, scZ);
	}else
		PoseController::parseGainLine(line);
}


/**
	This method loads all the pertinent information regarding the simbicon controller from a file.
*/
void SimBiController::loadFromFile(char* fName){
	if (fName == NULL)
		throwError("NULL file name provided.");
	FILE *f = fopen(fName, "r");
	if (f == NULL)
		throwError("Could not open file: %s", fName);

	//to be able to load multiple controllers from multiple files,
	//we will use this offset to make sure that the state numbers
	//mentioned in each input file are updated correctly
	int stateOffset = this->states.size();
	SimBiConState* tempState;
	int tempStateNr = -1;

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
			case CON_PD_GAINS_START:
				readGains(f);
				break;
			case CON_STATE_START:
				tempState = new SimBiConState();
				sscanf(line, "%d", &tempStateNr);
				if (tempStateNr != stateOffset + this->states.size())
					throwError("Inccorect state offset specified: %d", tempStateNr);
				states.push_back(tempState);
				tempState->readState(f, stateOffset);
				//now we have to resolve all the joint names (i.e. figure out which joints they apply to).
				resolveJoints(tempState);
				break;
			case CON_STANCE_HIP_DAMPING:
				sscanf(line, "%lf", &stanceHipDamping);
				break;
			case CON_STANCE_HIP_MAX_VELOCITY:
				sscanf(line, "%lf", &stanceHipMaxVelocity);
				break;
			case CON_ROOT_PRED_TORQUE_SCALE:
				sscanf(line, "%lf", &rootPredictiveTorqueScale);
				break;
			case CON_CHARACTER_STATE:
				//first we must correct the path
				path = std::string(line);
				path = interpret_path(path);
				//and now we can use it
				strcpy(effective_path, path.c_str());

				character->loadReducedStateFromFile(effective_path);
				strcpy(initialBipState, trim(line));
				break;
			case CON_START_AT_STATE:
				if (sscanf(line, "%d", &tempStateNr) != 1)
					throwError("A starting state must be specified!");
				transitionToState(tempStateNr);
				startingState = tempStateNr;
				break;
			case CON_COMMENT:
				break;
			case CON_STARTING_STANCE:
				if (strncmp(trim(line), "left", 4) == 0){
					setStance(LEFT_STANCE);
					startingStance = LEFT_STANCE;
				}
				else if (strncmp(trim(line), "right", 5) == 0){
					setStance(RIGHT_STANCE);
					startingStance = RIGHT_STANCE;
				}
				else 
					throwError("When using the \'reverseTargetOnStance\' keyword, \'left\' or \'right\' must be specified!");
				break;
			case CON_NOT_IMPORTANT:
				tprintf("Ignoring input line: \'%s\'\n", line);
				break;
			default:
				throwError("Incorrect SIMBICON input file: \'%s\' - unexpected line.", buffer);
		}
	}
	fclose(f);

}

/**
	This makes it possible to externally access the states of this controller
	Returns null if the index is out of range
 */
SimBiConState* SimBiController::getState( uint idx ) {
	if( idx >= states.size() ) return NULL;
	return states[idx];
}


/**
this function get the desired sagital velocity (affected by the variation trajectory)
*/
inline double SimBiController::get_effective_desired_sagittal_velocity(double phi){
	return velDSagittal *velD_sagittal_component()->baseTraj.evaluate_catmull_rom(phi);
}

/**
this function get the desired coronal velocity (affected by the variation trajectory)
*/
inline double SimBiController::get_effective_desired_coronal_velocity(double phi){
	double signChange = (getStance() == RIGHT_STANCE) ? 1 : -1;
	return velDCoronal +velD_coronal_component(getStance())->baseTraj.evaluate_catmull_rom(phi)*signChange;
	//return velDCoronal;
}

/**
this function will store the velocities every 0.1 phi when on learning mode
when learning learning mode is desactivated the function will adapt the velD_trajectories so they have a better fit on the movement
this system will also update a bolean signaling if the next step will be a recovery step or if it will be a normal step
*/
void SimBiController::velD_adapter(bool learning_mode, bool* trajectory_modified){
	double variation_moy_limit_z = 0.6;
	double variation_moy_limit_x = 0.6;
	
	
	static std::vector<double> vel_sagittal;
	static std::vector<double> vel_coronal;

	static double avgSpeed_z = 0;
	static double avgSpeed_x_left = 0;
	static double avgSpeed_x_right = 0;
	static uint timesVelSampled = 0;


	static int stance = 0;
	static int stance_var = -1;
	if (stance == 0){
		stance_var = getStance();
		stance= (stance_var == RIGHT_STANCE) ? 1 : -1;
	}

	TrajectoryComponent* sagittal_comp = velD_sagittal_component();
	TrajectoryComponent* coronal_comp = velD_coronal_component(stance_var);

	static double cur_phi_limit_z = sagittal_comp->baseTraj.getMinPosition();
	static int cur_knot_nbr_z = 0;

	static double cur_phi_limit_x = coronal_comp->baseTraj.getMinPosition();
	static int cur_knot_nbr_x = 0;

	if (learning_mode){
		//store the current speed to be able to know the avg speed at the end
		avgSpeed_z += character->getHeading().getComplexConjugate().rotate(character->getRoot()->getCMVelocity()).z;

		if (stance < 0){
			avgSpeed_x_left += character->getHeading().getComplexConjugate().rotate(character->getRoot()->getCMVelocity()).x;
		}
		else{
			avgSpeed_x_right += character->getHeading().getComplexConjugate().rotate(character->getRoot()->getCMVelocity()).x;
		}
		timesVelSampled++;

		//store the actual speed if necessary
		double cur_phi = MIN(getPhase(), 1);

		if (cur_knot_nbr_z != -1 && cur_phi > cur_phi_limit_z){
			//we save the speed for this point
			vel_sagittal.push_back(v.z / velDSagittal);

			//and we look at the next phi
			//if there are no next pt simply set the nbr of the next knot to -1
			cur_knot_nbr_z++;

			if (cur_knot_nbr_z < sagittal_comp->baseTraj.getKnotCount()){
				cur_phi_limit_z = sagittal_comp->baseTraj.getKnotPosition(cur_knot_nbr_z);
			}
			else{
				cur_knot_nbr_z = -1;
				cur_phi_limit_z = INFINITY;
			}

		}


		if (cur_knot_nbr_x != -1 && cur_phi > cur_phi_limit_x){
			//we save the speed for this point
			vel_coronal.push_back(v.x*stance - velDCoronal);

			//and we look at the next phi
			//if there are no next pt simply set the nbr of the next knot to -1
			cur_knot_nbr_x++;

			if (cur_knot_nbr_x < coronal_comp->baseTraj.getKnotCount()){
				cur_phi_limit_x = coronal_comp->baseTraj.getKnotPosition(cur_knot_nbr_x);
			}
			else{
				cur_knot_nbr_x = -1;
				cur_phi_limit_x = INFINITY;
			}
		}
	}
	else{
		//I finish the calculation of the avg speed
		avgSpeed_z /= timesVelSampled;
		avgSpeed_x_left /= timesVelSampled;
		avgSpeed_x_right /= timesVelSampled;

		recovery_step = false;
		TrajectoryComponent* affected_component = NULL;


		//first I handle the sagittal trajectory
		affected_component = sagittal_comp;
		bool need_sup_point = static_cast<int>(vel_sagittal.size()) < affected_component->baseTraj.getKnotCount();

		//So we calculate the moy variation
		double variation_moy = 0;
		std::vector<double> variation_vector;
		for (int i = 0; i < (int)vel_sagittal.size(); ++i){
			variation_vector.push_back(vel_sagittal[i] - affected_component->baseTraj.getKnotValue(i));
			variation_moy += std::abs(variation_vector.back());
		}
		int nbr_values = vel_sagittal.size();

		//this 'if' add a new point in case we haven't reached 1.0
		double sup_point_variation = 0;
		double sup_point_traj_val = 0;
		if (need_sup_point){
			sup_point_traj_val = affected_component->baseTraj.evaluate_catmull_rom(phi_last_step);
			sup_point_variation = v.z / velDSagittal - sup_point_traj_val;
			variation_moy += std::abs(sup_point_variation);
			nbr_values++;
		}

		variation_moy /= nbr_values;

		if (variation_moy < variation_moy_limit_z){

			//we handle the points stored in the buffer
			for (int i = 0; i < (int)vel_sagittal.size(); ++i){
				//we prevent that the variation we are gonna use is based on a value superior to the moy variation
				if (std::abs(variation_vector[i]) > variation_moy){
					variation_vector[i] = variation_moy*variation_vector[i] / std::abs(variation_vector[i]);
				}

				double new_val = affected_component->baseTraj.getKnotValue(i) + variation_vector[i] / 2;
				affected_component->baseTraj.setKnotValue(i, new_val);
			}

			//we handle the supplementary point if one was added
			if (need_sup_point){
				//we limit the variation on the virtual point
				if (std::abs(sup_point_variation) > variation_moy){
					sup_point_variation = variation_moy*sup_point_variation / std::abs(sup_point_variation);
				}

				double new_val = sup_point_traj_val + sup_point_variation / 2;

				//I now need to calculate the value for the next point in the trajectory
				// I'll use a linear interpolation for it
				int pt_nbr = nbr_values - 2;
				double val_previous_pt = affected_component->baseTraj.getKnotValue(pt_nbr);
				double pos_previous_pt = affected_component->baseTraj.getKnotPosition(pt_nbr);
				double pos_next_pt = affected_component->baseTraj.getKnotPosition(pt_nbr + 1);

				double val_next_pt = val_previous_pt + (pos_next_pt - pos_previous_pt)*(new_val - val_previous_pt) / (phi_last_step - pos_previous_pt);

				//we now limit the variation
				double val_next_variation = val_next_pt - affected_component->baseTraj.getKnotValue(pt_nbr + 1);
				if (std::abs(val_next_variation) > variation_moy){
					val_next_variation = variation_moy*val_next_variation / std::abs(val_next_variation);
				}

				//and we set the value in the curve
				val_next_pt = affected_component->baseTraj.getKnotValue(pt_nbr + 1) + val_next_variation / 2;
				affected_component->baseTraj.setKnotValue(pt_nbr + 1, val_next_pt);


			}


			//now we need to translate the curve depending on the speed it result in and the speed we want
			double evo_speed = 1;

			//I'l simply divide by the ratio between the avg_speed and the velD
			double traj_delta = (avgSpeed_z / velDSagittal - 1)* evo_speed;
			
			//I'll limit the possible translation to 0.25
			if (traj_delta>0.25){
				traj_delta = 0.25;
			}
			else if (traj_delta < -0.25){
				traj_delta = -0.25;
			}
			traj_delta += 1;

			for (int i = 0; i < nbr_values; ++i){
				affected_component->baseTraj.setKnotValue(i, affected_component->baseTraj.getKnotValue(i) / traj_delta);
			}


			//I'll now handle the other points (the ones even after the point I just created
			//simply using a linear interpolation to know how they should be updated (limiting the possible variation the the variation_moy/2
			//and preventing them to go over the current min and max of the trajectory
			if (nbr_values < affected_component->baseTraj.getKnotCount()){

				double min_z = affected_component->baseTraj.getKnotValue(0);
				double max_z = affected_component->baseTraj.getKnotValue(0);

				for (int i = 1; i < nbr_values; ++i){
					double val = affected_component->baseTraj.getKnotValue(i);
					if (val < min_z){
						min_z = val;
					}
					else if (val > max_z){
						max_z = val;
					}
				}

				//compute the step and the starting point
				double a = affected_component->baseTraj.getKnotValue(nbr_values - 1) -
					affected_component->baseTraj.getKnotValue(nbr_values - 2);
				a /= (affected_component->baseTraj.getKnotPosition(nbr_values - 1) -
					affected_component->baseTraj.getKnotPosition(nbr_values - 2));

				for (int i = nbr_values; i < (int)(affected_component->baseTraj.getKnotCount()); ++i){
					//we compute the value with linear prolongation
					double next_val = affected_component->baseTraj.getKnotValue(i - 1) + a*
						(affected_component->baseTraj.getKnotPosition(i) - affected_component->baseTraj.getKnotPosition(i - 1));

					//now we apply the limits
					if (next_val > max_z){
						next_val = max_z;
					}
					else if (next_val < min_z){
						next_val = min_z;
					}

					//we now compute the variation we created in the point position and limit it
					double point_variation = next_val - affected_component->baseTraj.getKnotValue(i);
					if (std::abs(point_variation) > variation_moy){
						point_variation = variation_moy*point_variation / std::abs(point_variation);
					}
					next_val = affected_component->baseTraj.getKnotValue(i) + point_variation / 2;


					//and we can set the value in the curve
					affected_component->baseTraj.setKnotValue(i, next_val);
				}
			}


		}
		else{
			//this should mean the caracter is currenlty in an unstable state
			recovery_step = true;
		}
		//we are finished with the sagittal trajectory

		//now I need to handle the coronal trajectory
		//I won't make the trajectory evolve for the first 2 steps (because we are not sure we have a complete trajaectory and real avgspeed
		//*
		static int nbr_steps=0;
		nbr_steps++;
		if (nbr_steps>2){

			affected_component = coronal_comp;
			need_sup_point = static_cast<int>(vel_coronal.size()) < affected_component->baseTraj.getKnotCount();

			//So we calculate the moy variation
			variation_moy = 0;
			variation_vector.clear();
			for (int i = 0; i < (int)vel_coronal.size(); ++i){
				variation_vector.push_back(vel_coronal[i] - affected_component->baseTraj.getKnotValue(i));
				variation_moy += std::abs(variation_vector.back());
			}
			nbr_values = vel_coronal.size();

			//this 'if' add a new point in case we haven't reached 1.0
			sup_point_variation = 0;
			sup_point_traj_val = 0;
			if (need_sup_point){
				sup_point_traj_val = affected_component->baseTraj.evaluate_catmull_rom(phi_last_step);
				sup_point_variation = v.x*stance - velDCoronal - sup_point_traj_val;
				variation_moy += std::abs(sup_point_variation);
				nbr_values++;
			}

			variation_moy /= nbr_values;

			if (variation_moy < variation_moy_limit_x){

				//we handle the points stored in the buffer
				for (int i = 0; i < (int)vel_coronal.size(); ++i){
					double cur_phi = i*0.1;
					//we prevent that the variation we are gonna use is based on a value superior to the moy variation
					if (std::abs(variation_vector[i]) > variation_moy){
						variation_vector[i] = variation_moy*variation_vector[i] / std::abs(variation_vector[i]);
					}

					double new_val = affected_component->baseTraj.getKnotValue(i) + variation_vector[i] / 2;
					affected_component->baseTraj.setKnotValue(i, new_val);
				}

				//we handle the supplementary point if one was added
				if (need_sup_point){
					//we limit the variation on the virtual point
					if (std::abs(sup_point_variation) > variation_moy){
						sup_point_variation = variation_moy*sup_point_variation / std::abs(sup_point_variation);
					}

					double new_val = sup_point_traj_val + sup_point_variation / 2;

					//I now need to calculate the value for the next point in the trajectory
					// I'll use a linear interpolation for it
					int pt_nbr = nbr_values - 2;
					double val_previous_pt = affected_component->baseTraj.getKnotValue(pt_nbr);
					double pos_previous_pt = affected_component->baseTraj.getKnotPosition(pt_nbr);
					double pos_next_pt = affected_component->baseTraj.getKnotPosition(pt_nbr + 1);

					double val_next_pt = val_previous_pt + (pos_next_pt - pos_previous_pt)*(new_val - val_previous_pt) / (phi_last_step - pos_previous_pt);

					//we now limit the variation
					double val_next_variation = val_next_pt - affected_component->baseTraj.getKnotValue(pt_nbr + 1);
					if (std::abs(val_next_variation) > variation_moy){
						val_next_variation = variation_moy*val_next_variation / std::abs(val_next_variation);
					}

					//and we set the value in the curve
					val_next_pt = affected_component->baseTraj.getKnotValue(pt_nbr + 1) + val_next_variation / 2;
					affected_component->baseTraj.setKnotValue(pt_nbr + 1, val_next_pt);


				}

				//*
				//now we need to translate the curve depending on the speed it result in and the speed we want
				double evo_speed = 0.1;

				//I'l simply divide by the ratio between the avg_speed and the velD
				double traj_delta = ((avgSpeed_x_left + avgSpeed_x_right) - velDCoronal)* evo_speed;
				for (int i = 0; i < nbr_values; ++i){
					affected_component->baseTraj.setKnotValue(i, affected_component->baseTraj.getKnotValue(i) - stance*traj_delta);
				}
				//*/

				//I'll now handle the other points (the ones even after the point I just created
				//simply using a linear interpolation to know how they should be updated (limiting the possible variation the the variation_moy/2
				//and preventing them to go over the current min and max of the trajectory
				//*
				if (nbr_values < affected_component->baseTraj.getKnotCount()){

					double min_z = affected_component->baseTraj.getKnotValue(0);
					double max_z = affected_component->baseTraj.getKnotValue(0);

					for (int i = 1; i < nbr_values; ++i){
						double val = affected_component->baseTraj.getKnotValue(i);
						if (val < min_z){
							min_z = val;
						}
						else if (val > max_z){
							max_z = val;
						}
					}

					//compute the step and the starting point
					double a = affected_component->baseTraj.getKnotValue(nbr_values - 1) -
						affected_component->baseTraj.getKnotValue(nbr_values - 2);
					a /= (affected_component->baseTraj.getKnotPosition(nbr_values - 1) -
						affected_component->baseTraj.getKnotPosition(nbr_values - 2));

					for (int i = nbr_values; i < (int)(affected_component->baseTraj.getKnotCount()); ++i){
						//we compute the value with linear prolongation
						double next_val = affected_component->baseTraj.getKnotValue(i - 1) + a*
							(affected_component->baseTraj.getKnotPosition(i) - affected_component->baseTraj.getKnotPosition(i - 1));

						//now we apply the limits
						if (next_val > max_z){
							next_val = max_z;
						}
						else if (next_val < min_z){
							next_val = min_z;
						}

						//we now compute the variation we created in the point position and limit it
						double point_variation = next_val - affected_component->baseTraj.getKnotValue(i);
						if (std::abs(point_variation) > variation_moy){
							point_variation = variation_moy*point_variation / std::abs(point_variation);
						}
						next_val = affected_component->baseTraj.getKnotValue(i) + point_variation / 2;


						//and we can set the value in the curve
						affected_component->baseTraj.setKnotValue(i, next_val);
					}
				}
				//*/

			}
			else{
				//this should mean the caracter is currenlty in an unstable state
				recovery_step = true;
			}
		}





		//*/
	
	
		//we reinitialize the system
		vel_sagittal.clear();
		vel_coronal.clear();
		cur_phi_limit_z = 0;
		cur_knot_nbr_z = 0;
		cur_phi_limit_x = 0;
		cur_knot_nbr_x = 0;
		avgSpeed_z = 0;
		timesVelSampled = 0;
		
		if (stance < 0){
			avgSpeed_x_right = 0;
		}
		else{
			avgSpeed_x_left = 0;
		}
		stance = 0;
	}

}
