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

#include "PoseController.h"
#include "BaseControlFramework.h"
#include <Utils/Utils.h>
#include <Physics/RigidBody.h>
#include "SimBiConState.h"
#include "Trajectory.h"
#include "VirtualModelController.h"


/**
	This structure is used to store the state of a simbicon controller. It can be used to save/load a controller's
	states, where the state here does not refer to the states in the Finite State Machine. The state refers to the
	phase in the current FSM state, and also the stance.
*/
typedef struct {
	int stance;
	double phi;
	int FSMStateIndex;
	bool bodyGroundContact;
} SimBiControllerState;


/**
 * A simbicon controller is a fancy PoseController. The root (i.e. pelvis or torso), as well as the two hips are controlled
 * relative to a semi-global coordinate frame (it needs only have the y-axis pointing up), but all other joints are controlled
 * exactely like in a normal PoseController. The SimBiController operates on a biped, which means that we can make special
 * assumptions about it: it must have two joints, lHip and rHip, connecting the root to the left and right upper-legs,
 * and it must also have two feet (lFoot and rFoot) as rigid bodies in the articulated linkage.
 */

class SimBiController : public PoseController{
friend class ConCompositionFramework;
friend class SimbiconPlayer;
friend class ControllerEditor;
private:
/**
	These are quantities that are set only once
*/
	//we will keep a reference to the left and right feet to be able to determine when the stance switches
	RigidBody* lFoot;
	RigidBody* rFoot;
	//we will also keep a reference to the root of the figure, to be able to identify the semi-global coordinate frame quickly
	RigidBody* root;

	//this is a collection of the states that are used in the controller
	DynamicArray<SimBiConState*> states;

	//the root is not directly controlled by any joint, so we will store its Kp, Kd and maxTorque separated here.
	//while the pose controller does not use these values, other types of controllers may need this information
	ControlParams rootControlParams;

	double stanceHipDamping;
	double stanceHipMaxVelocity;
	double rootPredictiveTorqueScale;


/**
	these are quantities that get updated throughout the simulation
*/
	//this value indicates which side is the stance side. 
	int stance;

	//this is the index of the controller that is currently active
	int FSMStateIndex;

	//this is the vector from the cm of the stance foot to the cm of the character
	Vector3d d;
	//this is the velocity of the cm of the character
	Vector3d v;

	//this is the distance between the COM and the midpoint between the feet
	Vector3d doubleStanceCOMError;

	//the phase parameter, phi must have values between 0 and 1, and it indicates the progress through the current state.
	double phi;
	double phi_last_step;

	//this quaternion gives the current heading of the character. The complex conjugate of this orientation is used
	//to transform quantities from world coordinates into a rotation/heading-independent coordinate frame (called the character frame).
	//I will make the asumption that the character frame is a coordinate frame that is aligned with the vertical axis, but has 0 heading, and
	//the characterFrame quaternion is used to rotate vectors from the character frame to the real world frame
	Quaternion characterFrame;

	//this variable, updated everytime the controller state is advanced in time, is set to true if any body other than the feet are in contact
	//with the ground, false otherwise. A higer level process can determine if the controller failed or not, based on this information.
	bool bodyTouchedTheGround;

	//this is a controller that we will be using to compute gravity-cancelling torques
	VirtualModelController* vmc;

	//this trajectory contains the way time is distorded during the phase (it's used to dynamicaly adapt the time that the controler can use)
	Trajectory1D time_distortion_trajectory;

public:
	//keep a copy of the initial character state, starting stance and file that contains the character's initial state
	int startingState;
	int startingStance;
	char initialBipState[100];

	//the next members are for the implementation of the IPM

	//desired velocity in the sagittal plane
	double velDSagittal;
	//desired velocity in the coronal plane...
	double velDCoronal;


	//this is a desired foot trajectory that we may wish to follow, expressed separately, for the 3 components,
	//and relative to the current location of the CM
	Trajectory1D swingFootTrajectorySagittal;
	Trajectory1D swingFootTrajectoryCoronal;
	Trajectory1D swingFootTrajectoryDeltaSagittal;
	Trajectory1D swingFootTrajectoryDeltaCoronal;
	Trajectory1D swingFootTrajectoryDeltaHeight;

	//this is the vector that specifies the plane of rotation for the swing leg, relative to the root...
	Vector3d swingLegPlaneOfRotation;

	//desired offset of the CM relative to the stance foot/midpoint of the feet
	double comOffsetSagittal;
	double comOffsetCoronal;

	//this variable can be used to quickly alter the desired height, if panic ensues...
	double panicHeight;
	//and this should be used to add height for the leg (i.e. if it needs to step over an obstacle that wasn't planned for).
	double unplannedForHeight;

	//a pointer to the swing and stance feet
	RigidBody* stanceFoot;
	RigidBody* swingFoot;


	//keep track of the legs
	int swingHipIndex, swingKneeIndex, swingAnkleIndex;
	int stanceHipIndex, stanceAnkleIndex, stanceKneeIndex;

	Joint *swingKnee, *swingHip;

	//we also keep the indices (useless and should be removed when refactoring)
	int lHipIndex;
	int rHipIndex;
	int lKneeIndex, rKneeIndex, lAnkleIndex, rAnkleIndex;

	//I'll store the stance I am in so i don't have to do it endlessly
	int stance_mode;//-1 no feet on ground, 0 one foot on ground, 1 both feet on ground
	double stanceHipToSwingHipRatio;
	bool stanceHeelInContact, stanceToeInContact;


	//I'll store the swing foot trajectory here before I'll use it often
	Trajectory* swing_foot_traj;

	//I'll store the velD trajectory here before I'll use it often
	Trajectory* velD_traj;

	//this bolean indicate if the next step should be a step where we focus on trying to get back in a stable state
	bool recovery_step;

protected:




	/**
		This method is used to parse the information passed in the string. This class knows how to read lines
		that have the name of a joint, followed by a list of the pertinent parameters. If this assumption is not held,
		then classes extended this one are required to provide their own implementation of this simple parser
	*/
	virtual void parseGainLine(char* line);

	/**
		This method is used to set the current FSM state of the controller to that of the index that
		is passed in.
	*/
	void setFSMStateTo(int index);

	/**
		This method should be called when the controller transitions to this state.
	*/
	void transitionToState(int stateIndex);

	/**
		This method returns the net force on the body rb, acting from the ground
	*/
	Vector3d getForceOn(RigidBody* rb, DynamicArray<ContactPoint> *cfs);

	/**
		This method returns the net force on the body rb, acting from the ground
	*/
	Vector3d getForceOnFoot(RigidBody* foot, DynamicArray<ContactPoint> *cfs);

	/**
		This method is used to determine if the rigid body that is passed in as a parameter is a
		part of a foot
	*/
	bool isFoot(RigidBody* rb);

	/**
		This method returns true if the rigid body that is passed in as a parameter is a swing foot, false otherwise
	*/
	bool isStanceFoot(RigidBody* rb);

	/**
		This method returns true if the rigid body that is passed in as a parameter is a swing foot, false otherwise
	*/
	bool isSwingFoot(RigidBody* rb);

	/**
		This method is used to return the ratio of the weight that is supported by the stance foot.
	*/
	double getStanceFootWeightRatio(DynamicArray<ContactPoint> *cfs);

	/**
	this function override the results of the ipm with the specified results
	*/
	void use_specified_swing_foot(double dt);

	/**
	This method is used to compute the target angles for the swing hip and swing knee that help
	to ensure (approximately) precise foot-placement control.
	*/
	void computeIKSwingLegTargets(double dt);

	/**
	This method returns a target for the location of the swing foot, based on some state information. It is assumed that the velocity v
	is expressed in character-relative coordinates (i.e. the sagittal component is the z-component), while the com position, and the
	initial swing foot position is expressed in world coordinates. The resulting desired foot position is also expressed in world coordinates.
	*/
	Point3d getSwingFootTargetLocation(double t, const Point3d& com, const Quaternion& charFrameToWorld);

	Vector3d computeSwingFootDelta(double phiToUse = -1, int stanceToUse = -1);

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
	void computeIKQandW(int parentJIndex, int childJIndex, const Vector3d& parentAxis, const Vector3d& parentNormal, const Vector3d& childNormal, const Vector3d& childEndEffector, const Point3d& wP, bool computeAngVelocities, const Point3d& futureWP, double dt);



	/**
		This method is used to compute the torques that need to be applied to the stance and swing hips, given the
		desired orientation for the root and the swing hip. The coordinate frame that these orientations are expressed
		relative to is computed in this method. It is assumed that the stanceHipToSwingHipRatio variable is
		between 0 and 1, and it corresponds to the percentage of the total net vertical force that rests on the stance
		foot.
	*/
	void computeHipTorques(const Quaternion& qRootD, const Quaternion& qSwingHipD, double stanceHipToSwingHipRatio, Vector3d& ffRootTorque);

	/**
		This method is used to resolve the names (map them to their index) of the joints
	*/
	void resolveJoints(SimBiConState* state);

	/**
		This method is used to set the stance 
	*/
	void setStance(int newStance);


	/**
		This method is used to return a pointer to a rigid body, based on its name and the current stance of the character
	*/
	RigidBody* getRBBySymbolicName(char* sName);



public:
	/**
		Default constructor
	*/
	SimBiController(Character* b);

	/**
		Destructor
	*/
	virtual ~SimBiController(void);

	/*
	this func is useless I just needed to output those value to see them
	*/
	void read_target_swing_hip_knee(std::vector<double>& hip_target, std::vector<double>& knee_target){
		SimBiConState* curState= getState(getFSMState());
		for (int i = 0; i < curState->getTrajectoryCount(); i++){
			//now we have the desired rotation angle and axis, so we need to see which joint this is intended for
			int jIndex = curState->sTraj[i]->getJointIndex(stance);

			if (jIndex == stanceKneeIndex){
				for (int k = 0; k < (int)11; ++k){
					knee_target.push_back(curState->sTraj[i]->components[0]->baseTraj.evaluate_catmull_rom(k*0.1));
				}

			}
			else if	(jIndex == swingHipIndex){
				for (int k = 0; k < (int)11; ++k){
					hip_target.push_back(curState->sTraj[i]->components[1]->baseTraj.evaluate_catmull_rom(k*0.1));
				}
			}
		}
	}
	
	inline Vector3d get_d(){
		return d;
	}
	
	inline Vector3d get_v(){
		return v;
	}

	/*
	This function update the velD.
	*/
	inline void calc_desired_velocities(){
		//read the parameters from the gui
		velDSagittal = SimGlobals::velDSagittal;
		velDCoronal = SimGlobals::velDCoronal;

	}
	
	Vector3d get_step_size(){
		if (rFoot != NULL && lFoot != NULL){
			return Vector3d(getSwingFootPos(), getStanceFootPos());
		}
	}

	/**
	updates the indexes of the swing and stance hip, knees and ankles
	*/
	void updateSwingAndStanceReferences();


	/**
		This method is used to compute the torques that are to be applied at the next step.
	*/
	virtual void computeTorques(DynamicArray<ContactPoint> *cfs, std::map<uint, WaterImpact>& resulting_impact);

	/**
	This method is used to compute the target orientations using the current FSM
	*/
	void evaluateJointTargets(ReducedCharacterState& poseRS, Quaternion& qRootD);

	/**
	This method is used to ensure that each RB sees the net torque that the PD controller computed for it.
	Without it, an RB sees also the sum of -t of every child.
	*/
	void bubbleUpTorques();

	/**
	This method computes the torques that cancel out the effects of gravity,
	for better tracking purposes
	*/
	void computeGravityCompensationTorques(std::map<uint, WaterImpact>& resulting_impact);

	/**
	This function simulate a force on the COM to achieve the desired speed in the sagittal and corronal pane
	*/
	void COMJT(DynamicArray<ContactPoint> *cfs, Vector3d& ffRootTorque);

	/**
	This method is used to compute the force that the COM of the character should be applying.
	*/
	Vector3d computeVirtualForce();

	/**
	This method returns performes some pre-processing on the virtual torque. The torque is assumed to be in world coordinates,
	and it will remain in world coordinates.
	*/
	void preprocessAnkleVTorque(int ankleJointIndex, DynamicArray<ContactPoint> *cfs, Vector3d *ankleVTorque);

	/**
	determines if there are any heel/toe forces on the given RB
	*/
	void getForceInfoOn(RigidBody* rb, DynamicArray<ContactPoint> *cfs, ForceStruct& heelForce, ForceStruct& frontFeetForce,ForceStruct& toeForce);
	/**
	check to see if rb is the same as whichBody or any of its children
	*/
	bool haveRelationBetween(RigidBody* rb, RigidBody* whichBody);

	/**
	This method is used to compute torques for the stance leg that help achieve a desired speed in the sagittal and lateral planes
	*/
	void computeLegTorques(int ankleIndex, int kneeIndex, int hipIndex, DynamicArray<ContactPoint> *cfs, Vector3d& ffRootTorque,double leg_ratio);


	/**
		This method is used to advance the controller in time. It takes in a list of the contact points, since they might be
		used to determine when to transition to a new state. This method returns -1 if the controller does not advance to a new state,
		or the index of the state that it transitions to otherwise.
	*/
	int advanceInTime(double dt, DynamicArray<ContactPoint> *cfs);

	/*
	this function is here to make phi advance following the specifyed dt
	*/
	void advance_phase(double dt){

		this->phi += dt / get_cur_state_time();
	}

	/**
	this funtion is just an easy way to now the current state time length
	*/
	inline double get_cur_state_time(){
		return states[FSMStateIndex]->getStateTime();
	}

	/**
	returns the required stepping location, as predicted by the inverted pendulum model. The prediction is made
	on the assumption that the character will come to a stop by taking a step at that location. The step location
	is expressed in the character's frame coordinates.
	*/
	Vector3d computeIPStepLocation();

	/**
		This method is used to populate the structure that is passed in with the current state
		of the controller;
	*/
	void getControllerState(SimBiControllerState* cs);

	/**
		This method is used to populate the state of the controller, using the information passed in the
		structure
	*/
	void setControllerState(const SimBiControllerState &cs);
	
	/**
		This method loads all the pertinent information regarding the simbicon controller from a file.
	*/
	void loadFromFile(char* fName);

	/**
		This method is used to return the value of bodyGroundContact
	*/
	inline bool isBodyInContactWithTheGround(){
		return bodyTouchedTheGround;
	}

	/**
		This method is used to return the value of the phase (phi) in the current FSM state.
	*/
	inline double getPhase(){
		return phi*(SimGlobals::force_alpha/100000);
	}

	/**
		This method returns the position of the CM of the stance foot, in world coordinates
	*/
	inline Point3d getStanceFootPos(){
		if (stanceFoot)
			return stanceFoot->getCMPosition();
		return Point3d(0,0,0);
	}

	/**
		This method returns the position of the CM of the swing foot, in world coordinates
	*/
	inline Point3d getSwingFootPos(){
		if (swingFoot)
			return swingFoot->getCMPosition();
		return Point3d(0,0,0);
	}

	/**
	This method is used to write the current controller to a file
	*/
	void writeToFile(char* fileName, char* stateFileName = NULL);

	/**
	This method is used to write the current controller to a file
	*/
	void writeToFile(std::string fileName, std::string* stateFileName = NULL);

	/**
		This method is used to return the current state number
	*/
	inline int getFSMState(){
		return this->FSMStateIndex;
	}

	/**
		This method returns the character frame orientation
	*/
	Quaternion getCharacterFrame(){
		return characterFrame;
	}

	/**
		This method is used to update the d and v parameters, as well as recompute the character coordinate frame.
	*/
	void updateDAndV();

	/**
		This makes it possible to externally access the states of this controller
		Returns null if the index is out of range
	 */
	SimBiConState* getState( uint idx );

	/**
	This function get the desired pose from the last step
	*/
	void getDesiredPose(DynamicArray<double>& trackingPose);

	/**
		This method makes it possible to evaluate the debug pose at any phase angle
		Negative phase angle = Use the current controller phase angle
	*/
	void updateTrackingPose(DynamicArray<double>& trackingPose, double phiToUse = -1);

	/**
		this method returns the stance of the character
	*/
	inline int getStance(){
		return stance;
	}

	// Evaluate the D trajectory
	inline void computeD0( double phi, Vector3d* d0 ) {
		SimBiConState* currState = states[getFSMState()];
		computeDorV( phi, currState->dTrajX, currState->dTrajZ, stance, d0 );
	}


	// Evaluate the V trajectory 
	inline void computeV0( double phi, Vector3d* v0 ) {
		SimBiConState* currState = states[getFSMState()];
		computeDorV( phi, currState->vTrajX, currState->vTrajZ, stance, v0 );
	}


	// Evaluate the V trajectory 
	inline static void computeDorV( double phi, Trajectory1D* trajX, Trajectory1D* trajZ, int stance, Vector3d* result ) {
		result->y = 0;
		double signReverse = (stance == RIGHT_STANCE)?-1:1;
		if( trajX == NULL )
			result->x = 0;
		else
			result->x = trajX->evaluate_catmull_rom( phi ) * signReverse;
		if( trajZ == NULL )
			result->z = 0;
		else
			result->z = trajZ->evaluate_catmull_rom( phi );
	}

	/**
		This function compute the torques that have the same impact as the specified force if it's applyed at the
		specified point
		The force and the application point have to be specified in the olrd coordinates
		the affected_joints parameter let the use choose the joints on which the force impact have to be calculated.
	*/
	void compute_virtual_force(Vector3d F, Point3d p, std::vector<Joint*> affected_joints){
		for (uint i = 0; i < affected_joints.size(); ++i){
			Vector3d tmpV = Vector3d(affected_joints[i]->parent->getWorldCoordinates(affected_joints[i]->pJPos), p);
			Vector3d tmpT = tmpV.crossProductWith(F);
			torques[character->getJointIndex(affected_joints[i]->name)] += tmpT;
		}
	}
	
	/**
	those functions are here to make access to the diferent component of the velD easier
	*/
	inline TrajectoryComponent* velD_sagittal_component() {
		return velD_traj->components[2];
	}
	inline TrajectoryComponent* velD_coronal_component(int stance) {
		if (stance == RIGHT_STANCE){
			return velD_traj->components[0];
		}
		else{
			return velD_traj->components[1];
		}
	}

	/**
	this function get the desired sagital velocity (affected by the variation trajectory)
	*/
	inline double get_effective_desired_sagittal_velocity(double phi);

	/**
	this function get the desired coronal velocity (affected by the variation trajectory)
	*/
	inline double get_effective_desired_coronal_velocity(double phi);

	/**
	this function will store the velocities every for every phi specified on the velD curve when on learning mode
	when learning learning mode is desactivated the function will adapt the velD_trajectories so they have a better fit on the movement
	this system will also update a bolean signaling if the next step will be a recovery step or if it will be a normal step
	*/
	void velD_adapter(bool learning_mode=true, bool* trajectory_modified = NULL);

	/*
	this funtion will return a simple bolean telling us if the IPM result will be used or if they will be overriden
	*/
	bool ipm_used(){
		if (recovery_step){ return true; }

		if (getPhase() < 0.2){ return false; }

		if (v.y < 0){ return true; }
		
		return false;
	}
};
