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

#include "ODEWorld.h"
#include <Utils/utils.h>
#include <Physics/Joint.h>
#include <Physics/HingeJoint.h>
#include <Physics/UniversalJoint.h>
#include <Physics/BallInSocketJoint.h>
#include <Core/SimGlobals.h>
#include <Utils/Timer.h>
#include "Core\Character.h"

/**
	Default constructor
*/
ODEWorld::ODEWorld() : AbstractRBEngine(){
	int maxCont = 4;

	//Initialize the world, simulation space and joint groups
    worldID = dWorldCreate();
    spaceID = dHashSpaceCreate(0);
	contactGroupID = dJointGroupCreate(0);

	//make sure that when we destroy the space group, we destroy all the geoms inside it
	dSpaceSetCleanup(spaceID, 1);

	//set a few of the constants that ODE needs to be aware of
	dWorldSetContactSurfaceLayer(worldID,0.001);							// the ammount of interpenetration allowed between objects
	dWorldSetContactMaxCorrectingVel(worldID, 1.0);							// maximum velocity that contacts are allowed to generate  

	//set the gravity...
	Vector3d gravity = SimGlobals::up * SimGlobals::gravity;
	dWorldSetGravity(worldID, gravity.x, gravity.y, gravity.z);

	//allocate the space for the contacts;
	maxContactCount = maxCont;
	cps = new dContact[maxContactCount];

	pcQuery = NULL;

	pcQuery = new PreCollisionQuery();
}

/**
	destructor
*/
ODEWorld::~ODEWorld(void){
	//destroy the ODE physical world, simulation space and joint group
	delete cps;
	dJointGroupDestroy(contactGroupID);
	dSpaceDestroy(spaceID);
	dWorldDestroy(worldID);
	dCloseODE();
}


/**
	this method is used to copy the state of the ith rigid body to its ode counterpart.
*/
void ODEWorld::setODEStateFromRB(int i){
	if (i<0 || (uint)i>=odeToRbs.size())
		return;

	//if it is a locked object, we won't do anything about it
	if (odeToRbs[i].rb->isLocked() == true)
		return;

	dQuaternion tempQ;
	tempQ[0] = odeToRbs[i].rb->state.orientation.s;
	tempQ[1] = odeToRbs[i].rb->state.orientation.v.x;
	tempQ[2] = odeToRbs[i].rb->state.orientation.v.y;
	tempQ[3] = odeToRbs[i].rb->state.orientation.v.z;
	
	dBodySetPosition(odeToRbs[i].id, odeToRbs[i].rb->state.position.x, odeToRbs[i].rb->state.position.y, odeToRbs[i].rb->state.position.z);
	dBodySetQuaternion(odeToRbs[i].id, tempQ);
	dBodySetLinearVel(odeToRbs[i].id, odeToRbs[i].rb->state.velocity.x, odeToRbs[i].rb->state.velocity.y, odeToRbs[i].rb->state.velocity.z);
	dBodySetAngularVel(odeToRbs[i].id, odeToRbs[i].rb->state.angularVelocity.x, odeToRbs[i].rb->state.angularVelocity.y, odeToRbs[i].rb->state.angularVelocity.z);
}

/**
	this method is used to copy the state of the ith rigid body, from the ode object to its rigid body counterpart 
*/
void ODEWorld::setRBStateFromODE(int i){
	const dReal *tempData;

	//if it is a locked object, we won't do anything about it
	if (odeToRbs[i].rb->isLocked() == true)
		return;

	//if the objects is supposed to be planar, make sure we don't let drift accumulate
	if (odeToRbs[i].rb->props.isPlanar){
	   const dReal *rot = dBodyGetAngularVel(odeToRbs[i].id);
	   const dReal *quat_ptr;
	   dReal quat[4], quat_len;
	   quat_ptr = dBodyGetQuaternion( odeToRbs[i].id );
	   quat[0] = quat_ptr[0];
	   quat[1] = quat_ptr[1];
	   quat[2] = 0; 
	   quat[3] = 0; 
	   quat_len = sqrt( quat[0] * quat[0] + quat[1] * quat[1] );
	   quat[0] /= quat_len;
	   quat[1] /= quat_len;
	   dBodySetQuaternion( odeToRbs[i].id, quat );
	   dBodySetAngularVel( odeToRbs[i].id, rot[0], 0, 0);
	}


	tempData = dBodyGetPosition(odeToRbs[i].id);
	odeToRbs[i].rb->state.position.x = tempData[0];
	odeToRbs[i].rb->state.position.y = tempData[1];
	odeToRbs[i].rb->state.position.z = tempData[2];

	tempData = dBodyGetQuaternion(odeToRbs[i].id);
	odeToRbs[i].rb->state.orientation.s = tempData[0];
	odeToRbs[i].rb->state.orientation.v.x = tempData[1];
	odeToRbs[i].rb->state.orientation.v.y = tempData[2];
	odeToRbs[i].rb->state.orientation.v.z = tempData[3];

	tempData = dBodyGetLinearVel(odeToRbs[i].id);
	odeToRbs[i].rb->state.velocity.x = tempData[0];
	odeToRbs[i].rb->state.velocity.y = tempData[1];
	odeToRbs[i].rb->state.velocity.z = tempData[2];

	tempData = dBodyGetAngularVel(odeToRbs[i].id);
	odeToRbs[i].rb->state.angularVelocity.x = tempData[0];
	odeToRbs[i].rb->state.angularVelocity.y = tempData[1];
	odeToRbs[i].rb->state.angularVelocity.z = tempData[2];
}


/**
	this method is used to set up an ODE sphere geom. NOTE: ODE only allows planes to
	be specified in world coordinates, not attached to a body, so we need to fix it once and
	for all.
*/
dGeomID ODEWorld::getPlaneGeom(PlaneCDP* p, RigidBody* parent){
	//and create the ground plane
	Vector3d n = parent->getWorldCoordinates(p->getNormal());
	Vector3d o = Vector3d(parent->getWorldCoordinates(p->getPointOnPlane()));
	dGeomID g = dCreatePlane(spaceID, n.x, n.y, n.z, o.dotProductWith(n));
	return g;
}

/**
	this method is used to set up an ODE sphere geom. It is properly placed in body coordinates.
*/
dGeomID ODEWorld::getSphereGeom(SphereCDP* s){
	dGeomID g = dCreateSphere(0, s->getRadius());
	Point3d c = s->getCenter();
	dGeomSetPosition(g, c.x, c.y, c.z);
	return g;
}


/**
	this method is used to set up an ODE box geom. It is properly placed in body coordinates.
*/
dGeomID ODEWorld::getBoxGeom(BoxCDP* b){
	dGeomID g = dCreateBox(0, b->getXLen(), b->getYLen(), b->getZLen());
	Point3d c = b->getCenter();
	dGeomSetPosition(g, c.x, c.y, c.z);
	return g;
}

/**
	this method is used to set up an ODE sphere geom. It is properly placed in body coordinates.
*/
dGeomID ODEWorld::getCapsuleGeom(CapsuleCDP* c){
	Point3d a = c->getA();
	Point3d b = c->getB();
	Vector3d ab(a, b);
	dGeomID g = dCreateCCylinder(0, c->getRadius(), ab.length());
	
	Point3d cen = a + ab/2.0;
	dGeomSetPosition(g, cen.x, cen.y, cen.z);


	//now, the default orientation for this is along the z-axis. We need to rotate this to make it match the direction
	//of ab, so we need an angle and an axis...
	Vector3d defA(0, 0, 1);

	Vector3d axis = defA.crossProductWith(ab);
	axis.toUnit();
	double rotAngle = defA.angleWith(ab);

	Quaternion relOrientation = Quaternion::getRotationQuaternion(rotAngle, axis);
	
	dQuaternion q;
	q[0] = relOrientation.s;
	q[1] = relOrientation.v.x;
	q[2] = relOrientation.v.y;
	q[3] = relOrientation.v.z;

	dGeomSetQuaternion(g, q);

	return g;
}


/**
	This method is used to set up an ode hinge joint, based on the information in the hinge joint passed in as a parameter
*/
void ODEWorld::setupODEHingeJoint(HingeJoint* hj){
	dJointID j = dJointCreateHinge(worldID, 0);
	dJointAttach(j, odeToRbs[(int)(hj->child->id)].id, odeToRbs[(int)(hj->parent->id)].id);
	Point3d p = hj->child->getWorldCoordinates(hj->cJPos);
	dJointSetHingeAnchor(j, p.x, p.y, p.z);
	Vector3d a = hj->parent->getWorldCoordinates(hj->a);
	dJointSetHingeAxis(j, a.x, a.y, a.z);

	//now set the joint limits
	if (hj->useJointLimits == false)
		return;

	dJointSetHingeParam(j, dParamLoStop, hj->minAngle);
	dJointSetHingeParam(j, dParamHiStop, hj->maxAngle);
}

/**
	This method is used to set up an ode universal joint, based on the information in the universal joint passed in as a parameter
*/
void ODEWorld::setupODEUniversalJoint(UniversalJoint* uj){
	dJointID j = dJointCreateUniversal(worldID, 0);
	dJointAttach(j, odeToRbs[(int)(uj->child->id)].id, odeToRbs[(int)(uj->parent->id)].id);
	Point3d p = uj->child->getWorldCoordinates(uj->cJPos);
	dJointSetUniversalAnchor(j, p.x, p.y, p.z);

	Vector3d a = uj->parent->getWorldCoordinates(uj->a);
	Vector3d b = uj->child->getWorldCoordinates(uj->b);

	dJointSetUniversalAxis1(j, a.x, a.y, a.z);
	dJointSetUniversalAxis2(j, b.x, b.y, b.z);

	//now set the joint limits
	if (uj->useJointLimits == false)
		return;

	dJointSetUniversalParam(j, dParamLoStop, uj->minAngleA);
	dJointSetUniversalParam(j, dParamHiStop, uj->maxAngleA);
	dJointSetUniversalParam(j, dParamLoStop2, uj->minAngleB);
	dJointSetUniversalParam(j, dParamHiStop2, uj->maxAngleB);
}

/**
	This method is used to set up an ode ball-and-socket joint, based on the information in the ball in socket joint passed in as a parameter
*/
void ODEWorld::setupODEBallAndSocketJoint(BallInSocketJoint* basj){
	dJointID j = dJointCreateBall(worldID, 0);
	dJointAttach(j, odeToRbs[(int)(basj->child->id)].id, odeToRbs[(int)(basj->parent->id)].id);
	Point3d p = basj->child->getWorldCoordinates(basj->cJPos);
	//now we'll set the world position of the ball-and-socket joint. It is important that the bodies are placed in the world
	//properly at this point
	dJointSetBallAnchor(j, p.x, p.y, p.z);

	//now deal with the joint limits
	if (basj->useJointLimits == false)
		return;

	Vector3d a = basj->parent->getWorldCoordinates(basj->swingAxis1);
	Vector3d b =  basj->child->getWorldCoordinates(basj->twistAxis);

	//we'll assume that:
	//b is the twisting axis of the joint, and the joint limits will be (in magnitude) less than 90 degrees, otherwise
	//the simulation will go unstable!!!


	dJointID aMotor = dJointCreateAMotor(worldID, 0);
	dJointAttach(aMotor, odeToRbs[(int)(basj->parent->id)].id, odeToRbs[(int)(basj->child->id)].id);
	dJointSetAMotorMode(aMotor, dAMotorEuler);

	dJointSetAMotorParam(aMotor, dParamStopCFM, 0.1);
	dJointSetAMotorParam(aMotor, dParamStopCFM2, 0.1);
	dJointSetAMotorParam(aMotor, dParamStopCFM3, 0.1);


	dJointSetAMotorAxis (aMotor, 0, 1, a.x, a.y, a.z);
	dJointSetAMotorAxis (aMotor, 2, 2, b.x, b.y, b.z);

	dJointSetAMotorParam(aMotor, dParamLoStop, basj->minSwingAngle1);
	dJointSetAMotorParam(aMotor, dParamHiStop, basj->maxSwingAngle1);
	
	dJointSetAMotorParam(aMotor, dParamLoStop2, basj->minSwingAngle2);
	dJointSetAMotorParam(aMotor, dParamHiStop2, basj->maxSwingAngle1);

	dJointSetAMotorParam(aMotor, dParamLoStop3, basj->minTwistAngle);
	dJointSetAMotorParam(aMotor, dParamHiStop3, basj->maxTwistAngle);
}

/**
	this method is used to create ODE geoms for all the collision primitives of the rigid body that is passed in as a paramter
*/
void ODEWorld::createODECollisionPrimitives(RigidBody* body){
	//now we'll set up the body's collision detection primitives
	for (uint j=0;j<body->cdps.size();j++){
		int cdpType = body->cdps[j]->getType();

		//depending on the type of collision primitive, we'll now create g.
		dGeomID g;

		switch (cdpType){
			case SPHERE_CDP:
				g = getSphereGeom((SphereCDP*)body->cdps[j]);
				break;
			case CAPSULE_CDP:
				g = getCapsuleGeom((CapsuleCDP*)body->cdps[j]);
				break;
			case BOX_CDP:
				g = getBoxGeom((BoxCDP*)body->cdps[j]);
				break;
			case PLANE_CDP:
				//NOTE: only static objects can have planes as their collision primitives - if this isn't static, force it!!
				g = getPlaneGeom((PlaneCDP*)body->cdps[j], body);
				break;
			default:
				throwError("Ooppps... No collision detection primitive was created rb: %s, cdp: %d", body->name, j);
		}

		//now associate the geom to the rigid body that it belongs to, so that we can look up the properties we need later...
		dGeomSetData(g, body);

		//if it's a plane, it means it must be static, so we can't attach a transform to it...
		if (cdpType == PLANE_CDP)
			continue;

		//now we've created a geom for the current body. Note: g will be rotated relative to t, so that it is positioned
		//well in body coordinates, and then t will be attached to the body.
		dGeomID t = dCreateGeomTransform(spaceID);
		//make sure that when we destroy the transfromation, we destroy the encapsulated objects as well.
		dGeomTransformSetCleanup(t, 1);

		//associate the transform geom with the body as well
		dGeomSetData(t, body);

		//if the object is fixed, then we want the geometry to take into account the initial position and orientation of the rigid body
		if (body->isLocked() == true){
			dGeomSetPosition(t, body->state.position.x, body->state.position.y, body->state.position.z);
			dQuaternion q;
			q[0] = body->state.orientation.s;
			q[1] = body->state.orientation.v.x;
			q[2] = body->state.orientation.v.y;
			q[3] = body->state.orientation.v.z;
			dGeomSetQuaternion(t, q);
		}

		dGeomTransformSetGeom(t, g);
		//now add t (which contains the correctly positioned geom) to the body, if we do really have an ODE body for it
		if (body->isLocked() == false)
			dGeomSetBody(t, odeToRbs[body->id].id);
	}
}


/**
	This method reads a list of rigid bodies from the specified file.
*/
void ODEWorld::loadRBsFromFile(char* fName){
	//make sure we don't go over the old articulated figures in case this method gets called multiple times.
	int index = objects.size();
	int index_afs = AFs.size();

	AbstractRBEngine::loadRBsFromFile(fName);

	//now we'll make sure that the joint constraints are satisfied
	for (uint i=index;i<objects.size();i++){
		
		//CREATE AND LINK THE ODE BODY WITH OUR RIGID BODY

		//if the body is fixed, we'll only create the colission detection primitives
		if (objects[i]->isLocked() == true){
			//push in a dummy - never to be used - mapping!!!
			odeToRbs.push_back(ODE_RB_Map(0, objects[i]));
		}else{
			odeToRbs.push_back(ODE_RB_Map(dBodyCreate(worldID), objects[i]));
			//the ID of this rigid body will be its index in the 
			objects[i]->setBodyID(i);
			//we will use the user data of the object to store the index in this mapping as well, for easy retrieval
			dBodySetData(odeToRbs[i].id, (void*)i);
		}

		//if this is a planar object, make sure we constrain it to always stay planar
		if (objects[i]->props.isPlanar){
			dJointID j = dJointCreatePlane2D(worldID, 0);
			dJointAttach(j, odeToRbs[(int)(objects[i]->id)].id, 0);
		}

		//PROCESS THE COLLISION PRIMITIVES OF THE BODY
		createODECollisionPrimitives(objects[i]);

		//SET THE INERTIAL PARAMETERS

		if (objects[i]->isLocked() == false){
			dMass m;

			//set the mass and principal moments of inertia for this object
			m.setZero();
			Vector3d principalMoments = odeToRbs[i].rb->getPMI();
			m.setParameters(odeToRbs[i].rb->getMass(), 0, 0, 0, 
				principalMoments.x, 
				principalMoments.y, 
				principalMoments.z, 
				0, 0, 0);

			dBodySetMass(odeToRbs[i].id, &m);

			setODEStateFromRB(i);
		}

	}

	DynamicArray<Joint*> joints;

	//now we will go through all the new joints, and create and link their ode equivalents
	for (uint i=index_afs;i<AFs.size();i++){
		joints.clear();
		AFs[i]->addJointsToList(&joints);
		for (uint j=0;j<joints.size();j++){
			//connect the joint to the two bodies
			int jointType = joints[j]->getJointType();
			switch (jointType){
				case BALL_IN_SOCKET_JOINT:
					setupODEBallAndSocketJoint((BallInSocketJoint*)joints[j]);
					break;
				case HINGE_JOINT:
					setupODEHingeJoint((HingeJoint*)joints[j]);
					break;
				case UNIVERSAL_JOINT:
					setupODEUniversalJoint((UniversalJoint*)joints[j]);
					break;
				default:
					throwError("Ooops.... Only BallAndSocket, Hinge and Universal joints are currently supported.\n");
			}
		}
	}
}

/**
	this method is used to process the collision between the two objects passed in as parameters. More generally,
	it is used to determine if the collision should take place, and if so, it calls the method that generates the
	contact points.
*/
void ODEWorld::processCollisions(dGeomID o1, dGeomID o2){
    dBodyID b1, b2;
	RigidBody *rb1, *rb2;
    b1 = dGeomGetBody(o1);
    b2 = dGeomGetBody(o2);
    rb1 = (RigidBody*) dGeomGetData(o1);
    rb2 = (RigidBody*) dGeomGetData(o2);

	bool joined = b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact);

	if (pcQuery)
		if (pcQuery->shouldCheckForCollisions(rb1, rb2, joined) == false)
			return;

	//we'll use the minimum of the two coefficients of friction of the two bodies.
	double mu1 = rb1->getFrictionCoefficient();
	double mu2 = rb2->getFrictionCoefficient();
	double mu_to_use = min(mu1, mu2);
	double eps1 = rb1->getRestitutionCoefficient();
	double eps2 = rb2->getRestitutionCoefficient();
	double eps_to_use = min(eps1, eps2);

	int num_contacts = dCollide(o1,o2,maxContactCount,&(cps[0].geom), sizeof(dContact));

	double groundSoftness = 0, groundPenalty = 0;
	if (rb1){
		groundSoftness = rb1->props.groundSoftness;
		groundPenalty = rb1->props.groundPenalty;
	}else{
		groundSoftness = rb2->props.groundSoftness;
		groundPenalty = rb2->props.groundPenalty;
	}

	//fill in the missing properties for the contact points
	for (int i=0;i<num_contacts;i++){
		cps[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactBounce;
		cps[i].surface.mu = mu_to_use;
		cps[i].surface.bounce = eps_to_use;
		cps[i].surface.bounce_vel = 0.00001;

		cps[i].surface.soft_cfm = groundSoftness;
		cps[i].surface.soft_erp = groundPenalty;
	}

	// and now add them contact points to the simulation
	for (int i=0;i<num_contacts;i++){
		//create a joint, and link the two geometries.
		dJointID c = dJointCreateContact(worldID, contactGroupID, &cps[i]);
		dJointAttach(c, b1, b2);

		if (jointFeedbackCount >= MAX_CONTACT_FEEDBACK)
			tprintf("Warning: too many contacts are established. Some of them will not be reported.\n");
		else{
			if (contactPoints.size() != jointFeedbackCount){
				tprintf("Warning: Contact forces need to be cleared after each simulation, otherwise the results are not predictable.\n");
			}
			contactPoints.push_back(ContactPoint());
			//now we'll set up the feedback for this contact joint
			contactPoints[jointFeedbackCount].rb1 = rb1;
			contactPoints[jointFeedbackCount].rb2 = rb2;
			contactPoints[jointFeedbackCount].cp = Point3d(cps[i].geom.pos[0], cps[i].geom.pos[1], cps[i].geom.pos[2]);
			dJointSetFeedback(c,&(jointFeedback[jointFeedbackCount]));
			jointFeedbackCount++;
		}
	}
}

/**
	This method is used to set the state of all the rigid body in this collection.
*/
void ODEWorld::setState(DynamicArray<double>* state, int start){
	AbstractRBEngine::setState(state, start);
}

/**
	This method is a simple call back function that passes the message to the world whose objects are being acted upon. 
*/
void collisionCallBack(void* odeWorld, dGeomID o1, dGeomID o2){
	((ODEWorld*)odeWorld)->processCollisions(o1, o2);
}

void runTestStep(dWorldID w, dReal stepsize);


/**
	run a testing method...
*/
void ODEWorld::runTest(){
	//make sure that the state of the RB's is synchronized with the engine...
	setEngineStateFromRB();

	//restart the counter for the joint feedback terms
	jointFeedbackCount = 0;

	Timer t;
	t.restart();
	for (int i=0;i<10000000;i++){
		//and run the step...
		runTestStep(worldID, 1/2000.0);
	}
	logPrint("ODE: This took %lf s\n", t.timeEllapsed());
}

/**
	This method is used to integrate the forward simulation in time.
*/

void ODEWorld::advanceInTime(double deltaT){
	//make sure that the state of the RB's is synchronized with the engine...
	setEngineStateFromRB();

	//restart the counter for the joint feedback terms
	jointFeedbackCount = 0;

	//go through all the joints in the world, and apply their torques to the parent and child rb's
	for (uint j = 0; j<jts.size(); j++){
		Vector3d t = jts[j]->torque;
		//we will apply to the parent a positive torque, and to the child a negative torque
		dBodyAddTorque(odeToRbs[jts[j]->parent->id].id, t.x, t.y, t.z);
		dBodyAddTorque(odeToRbs[jts[j]->child->id].id, -t.x, -t.y, -t.z);
	}

	//clear the previous list of contact forces
	contactPoints.clear();

	//we need to determine the contact points first - delete the previous contacts
	dJointGroupEmpty(contactGroupID);
	//initiate the collision detection
	dSpaceCollide(spaceID, this, &collisionCallBack);

	//advance the simulation
	dWorldStep(worldID, deltaT);
//	runTestStep(worldID, deltaT);

	//copy over the state of the ODE bodies to the rigid bodies...
	setRBStateFromEngine();

	//copy over the force information for the contact forces
	for (int i=0;i<jointFeedbackCount;i++){
		contactPoints[i].f = Vector3d(jointFeedback[i].f1[0], jointFeedback[i].f1[1], jointFeedback[i].f1[2]);
		//make sure that the force always points away from the static objects
		if (contactPoints[i].rb1->isLocked() && !contactPoints[i].rb2->isLocked()){
			contactPoints[i].f = contactPoints[i].f * (-1);
			RigidBody* tmpBdy = contactPoints[i].rb1;
			contactPoints[i].rb1 = contactPoints[i].rb2;
			contactPoints[i].rb2 = tmpBdy;
		}
	}
}

/**
	This method is used to integrate the forward simulation in time.
*/
void ODEWorld::testAdvanceInTime(double deltaT){
	//make sure that the state of the RB's is synchronized with the engine...
	setEngineStateFromRB();

	//restart the counter for the joint feedback terms
	jointFeedbackCount = 0;

	//go through all the joints in the world, and apply their torques to the parent and child rb's
	for (uint j=0;j<jts.size();j++){
		Vector3d t = jts[j]->torque;
		//we will apply to the parent a positive torque, and to the child a negative torque
		dBodyAddTorque(odeToRbs[jts[j]->parent->id].id, t.x, t.y, t.z);
		dBodyAddTorque(odeToRbs[jts[j]->child->id].id, -t.x, -t.y, -t.z);
	}

	//clear the previous list of contact forces
	contactPoints.clear();

	//we need to determine the contact points first - delete the previous contacts
	dJointGroupEmpty(contactGroupID);
	//initiate the collision detection
	dSpaceCollide(spaceID, this, &collisionCallBack);

	//advance the simulation
//	dWorldStep(worldID, deltaT);
	runTestStep(worldID, deltaT);


	//copy over the state of the ODE bodies to the rigid bodies...
	setRBStateFromEngine();

	//copy over the force information for the contact forces
	for (int i=0;i<jointFeedbackCount;i++){
		contactPoints[i].f = Vector3d(jointFeedback[i].f1[0], jointFeedback[i].f1[1], jointFeedback[i].f1[2]);
		//make sure that the force always points away from the static objects
		if (contactPoints[i].rb1->isLocked() && !contactPoints[i].rb2->isLocked()){
			contactPoints[i].f = contactPoints[i].f * (-1);
			RigidBody* tmpBdy = contactPoints[i].rb1;
			contactPoints[i].rb1 = contactPoints[i].rb2;
			contactPoints[i].rb2 = tmpBdy;
		}
	}
}


/**
	this method is used to transfer the state of the rigid bodies, from ODE to the rigid body wrapper
*/
void ODEWorld::setRBStateFromEngine(){
	//now update all the rigid bodies...
	for (uint i=0;i<objects.size();i++){
		setRBStateFromODE(i);
//		objects[i]->updateToWorldTransformation();
	}
}

/**
	this method is used to transfer the state of the rigid bodies, from the rigid body wrapper to ODE's rigid bodies
*/
void ODEWorld::setEngineStateFromRB(){
	//now update all the rigid bodies...
	for (uint i=0;i<objects.size();i++){
		setODEStateFromRB(i);
	}
}

/**
	this method applies a force to a rigid body, at the specified point. The point is specified in local coordinates,
	and the force is also specified in local coordinates.
*/
void ODEWorld::applyRelForceTo(RigidBody* b, const Vector3d& f, const Point3d& p){
	if (!b)
		return;
	dBodyAddRelForceAtRelPos(odeToRbs[b->id].id, f.x, f.y, f.z, p.x, p.y, p.z);
}

/**
	this method applies a force to a rigid body, at the specified point. The point is specified in local coordinates,
	and the force is specified in world coordinates.
*/
void ODEWorld::applyForceTo(RigidBody* b, const Vector3d& f, const Point3d& p){
	if (!b)
		return;
	dBodyAddForceAtRelPos(odeToRbs[b->id].id, f.x, f.y, f.z, p.x, p.y, p.z);
}


/**
	this method applies a torque to a rigid body. The torque is specified in world coordinates.
*/
void ODEWorld::applyTorqueTo(RigidBody* b, const Vector3d& t){
	if (!b)
		return;
	dBodyAddTorque(odeToRbs[b->id].id, t.x, t.y, t.z);
}




/**
	this method is used to compute the effect of water (it convert a level of water into the induced forces
	this version only work for the bipV2 caracter
*/
void ODEWorld::compute_water_impact(Character* character, float water_level, std::map<uint, WaterImpact>& resulting_impact){
	//first I check if the water have any density (if not it's useless to try anything)
	if (IS_ZERO(SimGlobals::liquid_density)){
		return;
	}

	//*
	static std::vector<Joint*> lower_body;
	if (lower_body.empty()){
		character->getCharacterBottom(lower_body);
	}
	
	
	for (uint i = 0; i < lower_body.size(); ++i){
		Joint* joint = lower_body[i];
		RigidBody* body = joint->getChild();

		//F.x = 0;
		//F = -Vector3d(0, 0, 1)*SimGlobals::force_alpha / 3000;
		bool on_full_body = false;
		if (on_full_body){
			if (strcmp(body->name, "torso") == 0){
				//compute_liquid_drag_on_toes(i, water_level);

				//applyForceTo(body, F*20, body->getLocalCoordinates(body->getCMPosition()));

			}
			//applyForceTo(body, F, body->getLocalCoordinates(body->getCMPosition()));
		}
		else{
			WaterImpact water_impact;
			if (strcmp(body->name, "rToes") == 0){
				water_impact.drag_torque = compute_liquid_drag_on_toes(joint, water_level);
				water_impact.boyancy = compute_buoyancy(joint, water_level);
			}
			else if (strcmp(body->name, "lToes") == 0){
				water_impact.drag_torque = compute_liquid_drag_on_toes(joint, water_level);
				water_impact.boyancy = compute_buoyancy(joint, water_level);
			}
			else if (strcmp(body->name, "rFoot") == 0){
				water_impact.drag_torque = compute_liquid_drag_on_feet(joint, water_level);
				water_impact.boyancy = compute_buoyancy(joint, water_level);
			}
			else if (strcmp(body->name, "lFoot") == 0){
				water_impact.drag_torque = compute_liquid_drag_on_feet(joint, water_level);
				water_impact.boyancy = compute_buoyancy(joint, water_level);
			}
			else if (strcmp(body->name, "lLowerleg") == 0){
				water_impact.drag_torque = compute_liquid_drag_on_legs(joint, water_level);
				water_impact.boyancy = compute_buoyancy(joint, water_level);
			}
			else if (strcmp(body->name, "rLowerleg") == 0){
				water_impact.drag_torque = compute_liquid_drag_on_legs(joint, water_level);
				water_impact.boyancy = compute_buoyancy(joint, water_level);
			}
			else if (strcmp(body->name, "lUpperleg") == 0){
				water_impact.drag_torque = compute_liquid_drag_on_legs(joint, water_level);
				water_impact.boyancy = compute_buoyancy(joint, water_level);
			}
			else if (strcmp(body->name, "rUpperleg") == 0){
				water_impact.drag_torque = compute_liquid_drag_on_legs(joint, water_level);
				water_impact.boyancy = compute_buoyancy(joint, water_level);
			}
			else{
				//should be impossible with the current system
				//I'll just shut down the application if this happens bouahahahahha
				exit(0);
			}

			//Now I need to limit the drag torque (depending on the type of join)
			//there are 3 types of join with only one having a restriction (the HingeJoint)
			HingeJoint* hj = dynamic_cast<HingeJoint*>(joint);
			if (hj != NULL){
				Vector3d axis = body->getWorldCoordinates(hj->getRotAxisA());
				axis /= axis.length();
				water_impact.drag_torque = axis*water_impact.drag_torque.dotProductWith(axis);
			}
			resulting_impact[joint->get_idx()] = water_impact;
		}
	}
	//*/
	
}



/**
this function is a children function of the above one (it prevent mass duplication of code for similar body parts
this function handle the toes
*/
Vector3d ODEWorld::compute_liquid_drag_on_toes(Joint* joint, float water_level){

	RigidBody* body = static_cast<RigidBody*>(joint->getChild());

	CollisionDetectionPrimitive* cdp = body->cdps.front();
	SphereCDP* sphere = dynamic_cast<SphereCDP*>(cdp);

	if (sphere == NULL){
		throwError("the toes should only have a sphere primitive...");
		return Vector3d();
	}

	double dy = water_level - body->getCMPosition().getY();

	Vector3d drag_torque = Vector3d(0, 0, 0);

	//we vrify that the water hit the ball before doing anything
	if (dy + sphere->getRadius()>0){
		//now I want to determine the surface that face the speed

		int nbr_interval_r = 3;
		double dr = sphere->getRadius() / nbr_interval_r;

		int nbr_interval_t = 10;
		double dt = 2 * PI / nbr_interval_t;


		//we need the speed in world coordinates
		//since the sphere is realy small for the toes we approxiate the speed as beeing constant through the object
		///TODO remove that approximation
		Vector3d V = body->getCMVelocity();
		Vector3d v_norm = V / V.length();

		Vector3d u, v;//those will be the 2 unitary vector of the disc plane
		v_norm.getOrthogonalVectors(&u, &v);//get the value of the 2 unitary vector

		//since idk if they are normed I'll norm them to be sure
		u /= u.length();
		v /= v.length();

		Point3d center = body->getWorldCoordinates(sphere->getCenter());

		double cur_r = dr;
		double cur_t = 0;

		double S = 0;

		for (int i = 0; i < nbr_interval_r; ++i){
			cur_t = 0;

			//we now calculate the alpha for the current r
			double alpha = std::sqrt(sphere->getRadius()*sphere->getRadius() - cur_r*cur_r);
			alpha /= (v_norm.x + v_norm.y + v_norm.z);


			for (int j = 0; j < nbr_interval_t; ++j){
				//now we need to test to see if we have to consider this segment for the calculation

				//first we determine the wolrld coordinate of the current point
				Point3d p = center + u*std::cos(cur_t)*cur_r + v*std::sin(cur_t)*cur_r;

				//now I need to know if the corresponding point on the spere is affected by the water
				//since the water level only depnds on the y coordinate I only need to compute it (nvm the x and z)
				double y = p.y + alpha*v_norm.y;

				if (y < water_level){
					//I need to calculate the area and add count it 
					double ds = cur_r*dr*dt;
					S += ds;
				}
			}
		}

		//now that we have the surface we can compute the resulting force
		Vector3d F = -V*V.length() * 1 / 2 * SimGlobals::liquid_density*S;


		//if we remove th e approximation of constant speed on the whole toes we need to stop doing the integral
		//and apply the force on every ds
		applyForceTo(body, F, sphere->getCenter());




		/*
		//this can be used to show the forces
		ForceStruct cp;
		cp.F = F;
		cp.pt = body->getWorldCoordinates(sphere->getCenter());
		SimGlobals::vect_forces.push_back(cp);
		//*/

		
		Vector3d op = sphere->getCenter()-joint->getChildJointPosition();
		drag_torque= body->getWorldCoordinates(op).crossProductWith(F);

		
	}

	return drag_torque;
}

/**
this function is a children function of the above one (it prevent mass duplication of code for similar body parts)
this function handle the feet
*/
Vector3d ODEWorld::compute_liquid_drag_on_feet(Joint* joint, float water_level){
	
	RigidBody* body = static_cast<RigidBody*>(joint->getChild());



	CollisionDetectionPrimitive* cdp = body->cdps.front();
	BoxCDP* box = dynamic_cast<BoxCDP*>(cdp);

	if (box == NULL){
		//throwError("the toes should only have a sphere primitive...");
		return Vector3d();
	}

	//I want the lower points to find out if the box is in the water
	//I call Z the vertical axis but in this world representation the vertical axis is actualy Y...

	Point3d center = box->getCenter();
	Point3d corners[8];

	corners[0] = center + Point3d(box->getXLen() / 2, box->getYLen() / 2, -box->getZLen() / 2);
	corners[1] = center + Point3d(box->getXLen() / 2, -box->getYLen() / 2, -box->getZLen() / 2);
	corners[2] = center + Point3d(-box->getXLen() / 2, box->getYLen() / 2, -box->getZLen() / 2);
	corners[3] = center + Point3d(-box->getXLen() / 2, -box->getYLen() / 2, -box->getZLen() / 2);
	corners[4] = center + Point3d(box->getXLen() / 2, box->getYLen() / 2, box->getZLen() / 2);
	corners[5] = center + Point3d(box->getXLen() / 2, -box->getYLen() / 2, box->getZLen() / 2);
	corners[6] = center + Point3d(-box->getXLen() / 2, box->getYLen() / 2, box->getZLen() / 2);
	corners[7] = center + Point3d(-box->getXLen() / 2, -box->getYLen() / 2, box->getZLen() / 2);
	
	double miny = body->getWorldCoordinates(corners[0]).getY();

	for (int i = 0; i < 8; ++i){
		corners[i] = body->getWorldCoordinates(corners[i]);
		if (corners[i].getY() < miny){
			miny = corners[i].getY();
		}
	}
	
	Vector3d drag_torque=Vector3d(0,0,0);
	
	//we vrify that the water hit the ball before doing anything
	if (miny<water_level){
		//now I'll subdivide the faces in smaller surfaces and apply the necessary force on each of them

		//I'll have to handle each face in a diffenrent way
		//but i'll use these variable (well 2 of them) in every face so I'll create them here
		int nbr_interval_x = 3;
		int nbr_interval_y = 2;
		int nbr_interval_z = 9;

		double d_x = box->getXLen() / nbr_interval_x;
		double d_y = box->getYLen() / nbr_interval_y;
		double d_z = box->getZLen() / nbr_interval_z;
		Point3d cur_pos,cur_normal;

		///TODO optimize this so we star with the lowest corner

		//first let's handle the back face
		cur_pos = center + Point3d(-box->getXLen() / 2 + d_x/2, -box->getYLen() / 2 + d_y/2, -box->getZLen() / 2);
		cur_normal = Point3d(0, 0, -1);
		drag_torque+=compute_liquid_drag_on_plane(joint, box->getXLen(), box->getYLen(), box->getZLen(),
			cur_pos, cur_normal, water_level, nbr_interval_x, nbr_interval_y, 0);

		//now the front face
		cur_pos = center + Point3d(-box->getXLen() / 2 + d_x / 2, -box->getYLen() / 2 + d_y / 2, box->getZLen() / 2);
		cur_normal = Point3d(0, 0, 1);
		drag_torque+=compute_liquid_drag_on_plane(joint, box->getXLen(), box->getYLen(), box->getZLen(),
			cur_pos, cur_normal, water_level, nbr_interval_x, nbr_interval_y, 0);

		//now the left face
		cur_pos = center + Point3d(box->getXLen() / 2, -box->getYLen() / 2 + d_y / 2, -box->getZLen() / 2 + d_z / 2);
		cur_normal = Point3d(1, 0, 0);
		drag_torque+=compute_liquid_drag_on_plane(joint, box->getXLen(), box->getYLen(), box->getZLen(),
			cur_pos, cur_normal, water_level, 0, nbr_interval_y, nbr_interval_z);
		
		//now the right face
		cur_pos = center + Point3d(-box->getXLen() / 2, -box->getYLen() / 2 + d_y / 2, -box->getZLen() / 2 + d_z / 2);
		cur_normal = Point3d(-1, 0, 0);
		drag_torque+=compute_liquid_drag_on_plane(joint, box->getXLen(), box->getYLen(), box->getZLen(),
			cur_pos, cur_normal, water_level, 0, nbr_interval_y, nbr_interval_z);

		//now the top face
		cur_pos = center + Point3d(-box->getXLen() / 2 + d_x / 2, box->getYLen() / 2, -box->getZLen() / 2 + d_z / 2);
		cur_normal = Point3d(0, 1, 0);
		drag_torque+=compute_liquid_drag_on_plane(joint, box->getXLen(), box->getYLen(), box->getZLen(),
			cur_pos, cur_normal, water_level, nbr_interval_x, 0, nbr_interval_z);

		//now the top face to finish
		cur_pos = center + Point3d(-box->getXLen() / 2 + d_x / 2, -box->getYLen() / 2, -box->getZLen() / 2 + d_z / 2);
		cur_normal = Point3d(0, -1, 0);
		drag_torque+=compute_liquid_drag_on_plane(joint, box->getXLen(), box->getYLen(), box->getZLen(),
			cur_pos, cur_normal, water_level, nbr_interval_x, 0, nbr_interval_z);


		//*/
	}

	return drag_torque;
}
	
/**
	Compute and affect to force on a face 
*/
Vector3d ODEWorld::compute_liquid_drag_on_plane(Joint* joint, double l_x, double l_y, double l_z, Point3d pos,
	Vector3d normal, float water_level,	int nbr_interval_x, int nbr_interval_y, int nbr_interval_z){


	RigidBody* body = static_cast<RigidBody*>(joint->getChild());


	Vector3d drag_torque = Vector3d(0, 0, 0);

	double d_x = 0, d_y = 0, d_z = 0;
	if (nbr_interval_x > 0){
		d_x = l_x / nbr_interval_x;
	}

	if (nbr_interval_y > 0){
		d_y = l_y / nbr_interval_y;
	}

	if (nbr_interval_z > 0){
		d_z = l_z / nbr_interval_z;
	}

	if ((nbr_interval_x > 0) && (nbr_interval_y > 0)){
		double d_S = d_x*d_y;
		double init_y = pos.y;


		for (int i = 0; i < nbr_interval_x; ++i){
			for (int j = 0; j < nbr_interval_y; ++j){
				//here we calculate the force and applys it
				Vector3d V = body->getLocalCoordinates(body->getAbsoluteVelocityForLocalPoint(pos));
				Vector3d V_norm = V / V.length();

				//we the center of the fragment is not underater we skipp it
				if (body->getWorldCoordinates(pos).y > water_level){
					continue;
				}

				//we check if we are a facing the movement 
				double V_eff = V.z*normal.z;
				if (V_eff > 0){
					double S = d_S;

					//now that we have the surface we can compute the resulting force
					Vector3d F = -normal*V_eff*V_eff * 1 / 2 * SimGlobals::liquid_density*S;
					F = body->getWorldCoordinates(F);

					//if we remove th e approximation of constant speed on the whole toes we need to stop doing the integral
					//and apply the force on every ds
					applyForceTo(body,F, pos);

					Vector3d op = pos - joint->getChildJointPosition();
					drag_torque += body->getWorldCoordinates(op).crossProductWith(F);

					/*
					//this can be used to show the forces
					ForceStruct cp;
					cp.F = body->getWorldCoordinates(F);
					cp.cp = body->getWorldCoordinates(pos);
					SimGlobals::vect_forces.push_back(cp);
					//*/

				}
				pos = pos + Point3d(0, d_y, 0);
			}
			pos = Point3d(pos.x + d_x, init_y, pos.z);
		}
	}

	if ((nbr_interval_x > 0) && (nbr_interval_z > 0)){
		double d_S = d_x*d_z;
		double init_z = pos.z;

		for (int i = 0; i < nbr_interval_x; ++i){
			for (int j = 0; j < nbr_interval_z; ++j){
				//here we calculate the force and applys it
				Vector3d V = body->getLocalCoordinates(body->getAbsoluteVelocityForLocalPoint(pos));
				Vector3d V_norm = V / V.length();

				//we the center of the fragment is not underater we skipp it
				if (body->getWorldCoordinates(pos).y > water_level){
					continue;
				}

				//we check if we are afacing the movement 
				double V_eff = V.y*normal.y;
				if (V_eff > 0){
					double S = d_S;

					//now that we have the surface we can compute the resulting force
					Vector3d F = -normal*V_eff*V_eff * 1 / 2 * SimGlobals::liquid_density*S;

					//if we remove th e approximation of constant speed on the whole toes we need to stop doing the integral
					//and apply the force on every ds
					F = body->getWorldCoordinates(F);

					//if we remove th e approximation of constant speed on the whole toes we need to stop doing the integral
					//and apply the force on every ds
					applyForceTo(body, F, pos);

					Vector3d op = pos - joint->getChildJointPosition();
					drag_torque += body->getWorldCoordinates(op).crossProductWith(F);

					/*
					//this can be used to show the forces
					ForceStruct cp;
					cp.F = body->getWorldCoordinates(F);
					cp.cp = body->getWorldCoordinates(pos);
					SimGlobals::vect_forces.push_back(cp);
					//*/

				}
				pos = pos + Point3d(0, 0, d_z);
			}
			pos = Point3d(pos.x + d_x, pos.y, init_z);
		}
	}

	if ((nbr_interval_y > 0) && (nbr_interval_z > 0)){
		double d_S = d_z*d_y;
		double init_y = pos.y;

		for (int i = 0; i < nbr_interval_z; ++i){
			for (int j = 0; j < nbr_interval_y; ++j){
				//here we calculate the force and applys it
				Vector3d V = body->getLocalCoordinates(body->getAbsoluteVelocityForLocalPoint(pos));
				Vector3d V_norm = V / V.length();

				//we the center of the fragment is not underater we skipp it
				if (body->getWorldCoordinates(pos).y > water_level){
					continue;
				}

				//we check if we are afacing the movement 
				double V_eff = V.x*normal.x;
				if (V_eff > 0){
					double S = d_S;
					
					//now that we have the surface we can compute the resulting force
					Vector3d F = -normal*V_eff*V_eff * 1 / 2 * SimGlobals::liquid_density*S;
					F = body->getWorldCoordinates(F);

					//if we remove th e approximation of constant speed on the whole toes we need to stop doing the integral
					//and apply the force on every ds
					applyForceTo(body, F, pos);

					Vector3d op = pos - joint->getChildJointPosition();
					drag_torque += body->getWorldCoordinates(op).crossProductWith(F);

					/*
					//this can be used to show the forces
					ForceStruct cp;
					cp.F = body->getWorldCoordinates(F);
					cp.cp = body->getWorldCoordinates(pos);
					SimGlobals::vect_forces.push_back(cp);
					//*/
				}
				pos = pos + Point3d(0, d_y, 0);
			}
			pos = Point3d(pos.x , init_y, pos.z + d_z);
		}
	}

	return drag_torque;
}

/**
this function is a children function of the above one (it prevent mass duplication of code for similar body parts)
this function handle the legs and arms
*/
Vector3d ODEWorld::compute_liquid_drag_on_legs(Joint* joint, float water_level){
	RigidBody* body = static_cast<RigidBody*>(joint->getChild());

	CollisionDetectionPrimitive* cdp = body->cdps.front();
	CapsuleCDP* capsule = dynamic_cast<CapsuleCDP*>(cdp);

	if (capsule == NULL){
		//throwError("the toes should only have a sphere primitive...");
		return Vector3d();
	}

	//I want the lower points to find out if the capsule is in the water
	//it's easy it's forced that the lowest point is the extremity of the cylinder minus the radius
	//I call Z the vertical axis but in this world representation the vertical axis is actualy Y...	
	Point3d wA = body->getWorldCoordinates(capsule->getA());
	Point3d wB = body->getWorldCoordinates(capsule->getB());
	double miny = std::fmin(wA.y, wB.y);
	miny -= capsule->getRadius();


	Vector3d drag_torque = Vector3d(0, 0, 0);


	//we vrify that the water hit the capsule before doing anything
	if (miny<water_level){
		//now I'll subdivide the faces in smaller surfaces and apply the necessary force on each of them
		//so I'll first concider the cylindric part of the capsule.
		
		//let's say we will consider 100 intervals on the axis of the cylinder
		//and we will consider 5 intervals on the facet for each interval on the axis
		int axis_intervals = 20;
		int facet_intervals = 3;

		//I precalculate some information on the axis and the facet so the algorithm will be faster
		double facet_interval_length = capsule->getRadius() * 2 / facet_intervals;
		Vector3d axis_unit_vector = capsule->getB()-capsule->getA();
		if (wA.y> wB.y){
			axis_unit_vector *= -1;
		}

		double axis_length = axis_unit_vector.length();
		axis_unit_vector /= axis_length;
		double axis_interval_length = axis_length / axis_intervals;
		Vector3d axis_interval_vect = axis_unit_vector*axis_interval_length;
		double facet_interval_area=axis_interval_length*facet_interval_length;

		//position at the start
		Point3d axis_cur_pos;
		if (wA.y> wB.y){
			axis_cur_pos = capsule->getB() + axis_interval_vect / 2;
		}
		else{
			axis_cur_pos = capsule->getA() + axis_interval_vect / 2;
		}
		 
		//so now we start to iterate along the axis
		for (int i = 0; i < axis_intervals; ++i){
			//here is an approximation (comming from the fact that the facet are likely to be horizontal (meaning the height))
			//of each subfacet is likely to be the same as the on of the central point
			//if (body->getWorldCoordinates(axis_cur_pos).y > water_level){
			//	continue;
			//}

			//we read the spead on the axis
			Vector3d axis_speed = body->getLocalCoordinates(body->getAbsoluteVelocityForLocalPoint(axis_cur_pos));

			//to create the facet I must finds the vector that have the same direction as the speed but face the axis
			Vector3d n = axis_speed - axis_unit_vector*(axis_unit_vector.dotProductWith(axis_speed));
			n /= n.length();

			//I just compute the last vector of the basis
			Vector3d v2 = n.crossProductWith(axis_unit_vector);
			v2 /= v2.length();

			//now I have to iterate on each interval of the facet
			//so i position myself at the first interval
			Point3d cur_pos = axis_cur_pos + v2*(capsule->getRadius() - facet_interval_length / 2);

			bool force_applied = false;
			for (int j = 0; j < facet_intervals; ++j){
				if (body->getWorldCoordinates(cur_pos).y < water_level){
					Vector3d local_speed = body->getLocalCoordinates(body->getAbsoluteVelocityForLocalPoint(cur_pos));
				
					//now i want to ponderate the area by the orientation along the axis
					//but what I realy want is the sinus (since the ponderation is of 1 if the two vectors are perpendicular)
					double S = facet_interval_area*
						((local_speed / local_speed.length()).crossProductWith(axis_unit_vector)).length();

					//now I can compute the force and apply it
					Vector3d F = -local_speed*local_speed.length() * 1 / 2 * SimGlobals::liquid_density*S;
					F = body->getWorldCoordinates(F);

					//if we remove th e approximation of constant speed on the whole toes we need to stop doing the integral
					//and apply the force on every ds
					applyForceTo(body, F, cur_pos);

					Vector3d op = cur_pos - joint->getChildJointPosition();
					drag_torque += body->getWorldCoordinates(op).crossProductWith(F);

					/*
					//this can be used to show the forces
					ForceStruct cp;
					cp.F = body->getWorldCoordinates(F);
					cp.cp = body->getWorldCoordinates(cur_pos);
					SimGlobals::vect_forces.push_back(cp);
					//*/

					force_applied = true;
				}
				cur_pos = cur_pos - v2*facet_interval_length;
			}

			//we stop if we realise that no forces were applied on a whole row
			if (!force_applied){
				break;
			}

			axis_cur_pos += axis_interval_vect;
		}
	}
	return drag_torque;
}


ForceStruct ODEWorld::compute_buoyancy(Joint* joint, float water_level){
	RigidBody* body = static_cast<RigidBody*>(joint->getChild());

	CollisionDetectionPrimitive* cdp = body->cdps.front();
	SphereCDP* sphere = dynamic_cast<SphereCDP*>(cdp);
	BoxCDP* box = dynamic_cast<BoxCDP*>(cdp);
	CapsuleCDP* capsule = dynamic_cast<CapsuleCDP*>(cdp);

	ForceStruct result_force;
	if (sphere != NULL){
		result_force = compute_buoyancy_on_sphere(body, water_level, -SimGlobals::gravity, SimGlobals::liquid_density);// +SimGlobals::force_alpha);
	}
	else if (box != NULL){
		result_force = compute_buoyancy_on_box(body, water_level, -SimGlobals::gravity, SimGlobals::liquid_density);// +SimGlobals::force_alpha);
	}
	else if (capsule != NULL){
		result_force = compute_buoyancy_on_capsule(body, water_level, -SimGlobals::gravity, SimGlobals::liquid_density);// +SimGlobals::force_alpha);
	}

	if (!result_force.F.isZeroVector()){
		applyForceTo(body, result_force.F, result_force.pt);
		/*
		//this can be used to show the forces
		ForceStruct show_force = result_force;
		result_force.pt = body->getWorldCoordinates(result_force.pt);
		SimGlobals::vect_forces.push_back(result_force);
		//*/
	}
	
	return result_force;
}

ForceStruct ODEWorld::compute_buoyancy_on_sphere(RigidBody* body, float water_level, double gravity, double density){

	CollisionDetectionPrimitive* cdp = body->cdps.front();
	SphereCDP* sphere = dynamic_cast<SphereCDP*>(cdp);

	if (sphere == NULL){
		throwError("the toes should only have a sphere primitive...");
		return ForceStruct();
	}

	double r = sphere->getRadius();
	double h = water_level - body->getCMPosition().getY() - r;

	ForceStruct result;

	//we vrify that the water hit the ball before doing anything
	if (h>0){
		//I first eliminate the case where the spere is fully in the water (for efficiency purposes)
		if (h > (2 * r)){
			double V = 4/3 * PI* r*r*r;
			Vector3d F = Vector3d(0, 1, 0)*V*density*gravity;
			//applyForceTo(body, F, sphere->getCenter());
			result.F = F;
			result.pt = sphere->getCenter();

			return result;
		}

		double V = PI / 3 * h*h* (3 * r - h);
		Vector3d F = Vector3d(0, 1, 0)*V*density*gravity;

		double dy = 3/4 * (2 * r - h)*(2 * r - h) / (3 * r - h);
		Point3d pt = body->getLocalCoordinates(body->getCMPosition() - Point3d(0, dy, 0));

		//applyForceTo(body, F, pt);

		result.F = F;
		result.pt = pt;

		return result;

	}
	return ForceStruct();
}

ForceStruct ODEWorld::compute_buoyancy_on_box(RigidBody* body, float water_level, double gravity, double density){
	
	CollisionDetectionPrimitive* cdp = body->cdps.front();
	BoxCDP* box = dynamic_cast<BoxCDP*>(cdp);

	if (box == NULL){
		//throwError("the toes should only have a sphere primitive...");
		return ForceStruct();
	}

	//I want the lower points to find out if the box is in the water
	double lx = box->getXLen(), ly = box->getYLen(), lz = box->getZLen();
	Point3d center = box->getCenter();
	Point3d corners[8], wcorners[8];

	corners[0] = center + Point3d(lx / 2, ly / 2, -lz / 2);
	corners[1] = center + Point3d(lx / 2, -ly / 2, -lz / 2);
	corners[2] = center + Point3d(-lx / 2, ly / 2, -lz / 2);
	corners[3] = center + Point3d(-lx / 2, -ly / 2, -lz / 2);
	corners[4] = center + Point3d(lx / 2, ly / 2, lz / 2);
	corners[5] = center + Point3d(lx / 2, -ly / 2, lz / 2);
	corners[6] = center + Point3d(-lx / 2, ly / 2, lz / 2);
	corners[7] = center + Point3d(-lx / 2, -ly / 2, lz / 2);

	double miny = body->getWorldCoordinates(corners[0]).getY();
	int idx_min = 0;

	for (int i = 0; i < 8; ++i){
		wcorners[i] = body->getWorldCoordinates(corners[i]);
		if (wcorners[i].getY() < miny){
			miny = wcorners[i].getY();
			idx_min = i;
		}
	}


	//we vrify that the water hit the box before doing anything
	if (miny<water_level){
		ForceStruct result_force;

		// for all that is after it ill be easier if we know the basis formed by a corner
		// please note that the condition are set like that because of the way I built the corner structure
		Vector3d vx(1,0,0), vy(0,1,0), vz(0,0,1);
		if(idx_min > 3){
			vz *= -1;
		}
		if ((idx_min & 1) == 0){
			vy *= -1;
		}
		if (idx_min % 4 < 2){
			vx *= -1;
		}

		// I want to check some easy to compute cases.
		// First if the object is fully immersed (easy to find the highest point if we know the positions 
		//of the lower int the box) 
		//*
		Point3d upper_corner = corners[idx_min] + vx*lx + vy*ly + vz*lz;
		if (body->getWorldCoordinates(upper_corner).y < water_level){
			double V = lx*ly*lz;
			Vector3d F = Vector3d(0, 1, 0)*V*density*gravity;
			//applyForceTo(body, F, center);

			result_force.F = F;
			result_force.pt = center;

			return result_force;
		}//*/

		//I'll initialize some variable since i'll need them for every following tests
		int nbr_interval_x = 3;
		int nbr_interval_y = 3;
		int nbr_interval_z = 7;


		//second test is to check if e could not simplify the plroblem by a prism
		Vector3d wvx = body->getWorldCoordinates(vx);
		Vector3d wvy = body->getWorldCoordinates(vy);
		Vector3d wvz = body->getWorldCoordinates(vz);

		int count_alligned = 0;
		double epsilon = 1.0/(float)nbr_interval_y;

		if (wvx.y < epsilon){
			++count_alligned;
		}
		if (wvy.y < epsilon){
			++count_alligned;
		}
		if (wvz.y < epsilon){
			++count_alligned;
		}

		if (count_alligned>0){
			//so we have in a configuration were I simplify by a prism calculation

			/*if (count_alligned>1){
				//so we have a vertical prism calculationare simple so I won't comment it
				//the calculations 
				double V, h;
				Point3d pt;
				h = (water_level - wcorners[idx_min].y);
				if (wvx.y < epsilon){
					if (wvy.y < epsilon){
						V = lx*ly;
						pt = corners[idx_min] + vx*(lx / 2) + vy*(ly / 2) + vz*(h / 2);
					}
					else{
						V = lx*lz;
						pt = corners[idx_min] + vx*(lx / 2) + vy*(h / 2) + vz*(lz / 2);
					}
				}
				else{
					V = ly*lz;
					pt = corners[idx_min] + vx*(h / 2) + vy*(ly / 2) + vz*(lz / 2);
				}
				V *= h;
				Vector3d F = Vector3d(0, 1, 0)*V*density*gravity;
				applyForceTo(body, F, pt);
				return;
			}

			//so we have an inclined prism
			//here it's more tricky be cause I need the immerged surface on the side face.
			//so first I'll find the idxs of the corners of the face and i'll order them from
			//the lowest to the highest
			//seeing the code of that optimisation (which is not finished btw) I think it may take more time than simply 
			//doing an integral...
			//so i'll disactivate it for now
			/*

			double V,h; 
			int face[4];
			face[0] = idx_min;
			if (wvx.y < epsilon){
				int fz = 4 * vz.z;
				int fy = 1 * vy.y;

				if (wvy.y < wvz.y){
					face[1] = idx_min + fy;
					face[2] = idx_min + fz;
				}
				else {
					face[1] = idx_min + fz;
					face[2] = idx_min + fy;
				}
				face[3] = idx_min+fz+fy;
				h= lx;
			}
			if (wvy.y < epsilon){
				int fz = 4 * vx.x;
				int fx = 2 * vy.y;

				if (wvx.y < wvz.y){
					face[1] = idx_min + fx;
					face[2] = idx_min + fz;
				}
				else {
					face[1] = idx_min + fz;
					face[2] = idx_min + fx;
				}
				face[3] = idx_min + fz + fx;
				h= ly;
			}
			else{
				int fx = 2 * vx.x;
				int fy = 1 * vy.y;

				if (wvx.y < wvy.y){
					face[1] = idx_min + fx;
					face[2] = idx_min + fy;
				}
				else {
					face[1] = idx_min + fy;
					face[2] = idx_min + fx;
				}
				face[3] = idx_min + fy + fx;

				h= lz;
			}
			//no that we have the ordored face corner we can calculate the surface
			//there are 3 cases :

			//the first one is when the water level is above the 3rd point. In that cas e we have a partial
			//triangle (it's a trapezoid) and a second trapezoid under it

			

			if (water_level>wcorners[face[2]].y){
				//first I need the last point for the base of the upper triangle
				double ht = wcorners[face[3]].y - wcorners[face[2]].y;
				double dht = (wcorners[face[3]].y - water_level) / ht;
				Point3d triangle_pt = wcorners[face[3]] + (wcorners[face[2]] - wcorners[face[3]])*dht;
					
				double A_triangle = ht* (triangle_pt - wcorners[face[2]]).length();
				A_triangle -= A_triangle*dht;

				//and now the area of the trapezoid
			}
			else if (water_level > wcorners[face[2]].y){

			}
			else{

			}
			
			Vector3d F = Vector3d(0, 1, 0)*V*SimGlobals::liquid_density*SimGlobals::gravity;
			applyForceTo(body, F, center);
			return;
			//*/
		}
		
		//I we reach here it mean we are not a in a case where I can do a trivial calculation
		//now I'll subdivide the box in smaller boxes and simply count the ones underwater
		double d_x = lx / nbr_interval_x;
		double d_y = ly / nbr_interval_y;
		double d_z = lz / nbr_interval_z;
		double dV = d_x*d_y*d_z;

		//I'll have to work in word coordinate else I'd have to convert everytime I check with the water level
		Vector3d vxp = body->getWorldCoordinates(vx*d_x);
		Vector3d vyp = body->getWorldCoordinates(vy*d_y);
		Vector3d vzp = body->getWorldCoordinates(vz*d_z);


		Vector3d result = Vector3d(0, 0, 0);
		int count=0;
		int i, j, k;
		Vector3d cur_pos=wcorners[idx_min]+vxp/2+vyp/2+vzp/2;
		Vector3d cur_plane,cur_box;
		double V = 0;
		for (i = 0; i < nbr_interval_x; ++i){
			cur_plane = cur_pos;
			for (j = 0; j < nbr_interval_y; ++j){
				cur_box = cur_plane;
				for (k = 0; k < nbr_interval_z; ++k){
					if (cur_box.y>water_level){
						break;
					}
					//here it mean the box if underwater
					result += cur_box;
					V+=dV;
					count++;

					cur_box += vzp;
				}
				if (k == 0){
					break;
				}
				cur_plane += vyp;
			}
			if (j == 0){
				break;
			}
			cur_pos += vxp;
		}

		//and now we apply the force if it is possible
		if (count != 0){
			Vector3d F = Vector3d(0, 1, 0)*V*density*gravity;
			Point3d inter = result / count;//forced to use an inter var or it does realy strange things...
			Point3d pt = body->getLocalCoordinates(inter);

			//applyForceTo(body, F, pt);

			result_force.F = F;
			result_force.pt = pt;

			return result_force;
		}
	}
	return ForceStruct();
}

ForceStruct ODEWorld::compute_buoyancy_on_capsule(RigidBody* body, float water_level, double gravity, double density){
	CollisionDetectionPrimitive* cdp = body->cdps.front();
	CapsuleCDP* capsule = dynamic_cast<CapsuleCDP*>(cdp);

	if (capsule == NULL){
		//throwError("the toes should only have a sphere primitive...");
		return ForceStruct();
	}

	//I want the lower points to find out if the capsule is in the water
	//it's easy it's forced that the lowest point is the extremity of the cylinder minus the radius
	//I call Z the vertical axis but in this world representation the vertical axis is actualy Y...	
	Point3d wA = body->getWorldCoordinates(capsule->getA());
	Point3d wB = body->getWorldCoordinates(capsule->getB());
	double miny = std::fmin(wA.y, wB.y);
	miny -= capsule->getRadius();


	//we vrify that the water hit the capsule before doing anything
	if (miny < water_level){
		ForceStruct result_force;

		double r = capsule->getRadius();

		//there are 3 parts: lower sphere, upper sphere and cylinder
		//FOr each part I'll tryto handle easy cases then use a general method in case none of the optimisation are possible

		//First I handle the lower half sphere

		//then I handle the cylindric part
		//that for is just an easy way to skip the complex calculation in the case the simplification works
		for (int uselessvar = 0; uselessvar < 1; ++uselessvar){
			//first I need the lowest point

			//I need the normal of the vertica plane to create the rotation to get the lowest point
			Vector3d axis_vector = wA - wB;
			Point3d axis_lowest_pt = wB;
			Point3d axis_upper_pt = wA;
			if (axis_vector.y < 0){
				axis_vector *= -1;
				axis_lowest_pt = wA;
				axis_upper_pt = wB;
			}

			double axis_len = axis_vector.length();
			axis_vector /= axis_len;
		
			Vector3d n = Vector3d(0, 1, 0).crossProductWith(axis_vector);
			double sin_angle = n.length();
			n /= sin_angle;
		
		
			//and now I calculate the lowest point
			Quaternion quat = Quaternion::getRotationQuaternion(PI / 2, n);
			Vector3d vh = quat.rotate(axis_vector)*r;
			if (vh.y > 0){
				vh *= -1;
				sin_angle *= -1;
			}
			Point3d lowest_pt = axis_lowest_pt + vh;

		
			if (lowest_pt.y < water_level){
				//now I check if the full cylinder is 
				Point3d highest_pt = axis_upper_pt - vh;
				if (highest_pt.y < water_level){
					double V = PI*r*r*axis_len;
					Vector3d F = Vector3d(0, 1, 0)*V*density*gravity;
					Point3d inter = Vector3d(wA + wB) / 2;
					Point3d pt = body->getLocalCoordinates(inter);
					Point3d pt2 = body->getLocalCoordinates(body->getCMPosition());

					//applyForceTo(body, F, pt);

					result_force.F = F;
					result_force.pt = pt;

					break;
				}

				//another easy case is when the cilinder is near vertical
				//I will supose that as long as the angle is less than 5° it is vertical 
				//trully I conpare with 5.73 so that the sin is near 0.1 
				if (std::abs(sin_angle) < 0.01){
					double h = water_level - lowest_pt.y + vh.y / 2;
					if (h > axis_len){
						h = axis_len;
					}
					if (h > 0){
						double V = PI*r*r*h;
						Vector3d F = Vector3d(0, 1, 0)*V*density*gravity;
						Point3d inter = axis_lowest_pt + axis_vector*h/ 2 +vh*sin_angle;
						Point3d pt = body->getLocalCoordinates(inter);

						//applyForceTo(body, F, pt);

						result_force.F = F;
						result_force.pt = pt;


					}
					break;
				}

				//on the same logic I'll handle the cases where the cylinder is near horizontal
				//my goal is to eliminate the case of the near horizontal cylinder. But it is revelant to do it
				//only if the 2 bases are affected by water
				Point3d low_high_pt = lowest_pt + axis_vector*axis_len;
				if (((1 - std::abs(sin_angle)) < 0.001) && (low_high_pt.y < water_level)){
					double H = water_level - lowest_pt.y;
					if (H > r * 2){
						H = r * 2;
					}
					if (H > 0){
						//now I want to conform the notation used by wolfram
						//using their formula I can only achieve a result if the immersed part is lower than
						//half of the circle
						double h=H;
						if (H > r){
							h = 2 * r - H;
						}
						double theta = std::acos((r - h) / r);
						double circle_segment_area = r*r*(theta - std::sin(theta))/2;
						double area = circle_segment_area;
						if (axis_lowest_pt.y<water_level){
							area = PI*r*r - area;
						}
						Vector3d v_inter(vh.x, 0, vh.z);
						double cylinder_length = axis_len - v_inter.length();
						double V = cylinder_length*area;
						if (V > 0.00001){
							Vector3d F = Vector3d(0, 1, 0)*V*density*gravity;

							double sin_inter = std::sin(theta / 2);
							double dy=0;
							if ((theta - std::sin(theta)) > 0.0000001){
								dy= 4 * r*sin_inter*sin_inter*sin_inter / (3 * (theta - std::sin(theta)));
								if (H>r){
									dy = -dy*circle_segment_area/area;
								}
							}

							Point3d inter = Vector3d(axis_lowest_pt + Point3d(0, dy, 0) + axis_vector*(cylinder_length-v_inter.length()) / 2);
							Point3d pt = body->getLocalCoordinates(inter);

							//applyForceTo(body, F, pt);

							result_force.F = F;
							result_force.pt = pt;

						}	
					}
					break;
				}

				//now I have 2 case left. depending on the water level we have either a cylindrical wegde or a cylindrical segment
				//so I need to check in which case we are
				Point3d high_low_pt = axis_lowest_pt - vh;

				if (high_low_pt.y < water_level){
					//this mean we have a cylindrical segment
					double h1 = (axis_vector*((water_level - high_low_pt.y) / axis_vector.y)).length();
					double h2 = (axis_vector*((water_level - lowest_pt.y) / axis_vector.y)).length();//possible to change this to rmv the /

					double V = PI*r*r*(h1 + h2) / 2;
					Vector3d F = Vector3d(0, 1, 0)*V*density*gravity;

					//to compute the center of mass I use a formula from wolfram cilynder segment page
					//in their coordinate the cylinder axis is following Z and the  base f the cylinder is in the plane (xOy)
					double dx = r*(h2 - h1) / 4 * (h1 + h2);
					double dz = (5*h1*h1+6*h1*h2+5*h2*h2)/(16*(h1+h2));

					Point3d inter = Vector3d(axis_lowest_pt + axis_vector*dz + vh / vh.length()*dx);
					Point3d pt = body->getLocalCoordinates(inter);
					//applyForceTo(body, F, pt);

					result_force.F = F;
					result_force.pt = pt;

				}
				else{
					//this mean we have a cylindrical wedge
					//formula found on the wolfram page for cylindrical wedge
					double h = (axis_vector*((water_level - lowest_pt.y) / axis_vector.y)).length();
					double b = (-vh *((water_level - lowest_pt.y) / (-vh.y))).length();
					double a = std::sqrt(b*(2 * r - b));

					double V = (h / (6 * b))*(2 * a*(3 * r*r - 2 * a*b + b*b) - 3 * PI*r*r*(r - b) +
						6 * r*r*(r - b)*std::asin((r - b) / r));
					Vector3d F = Vector3d(0, 1, 0)*V*density*gravity;

					//since the calculation of the real application point seems to be pretty hard
					//I'll use a simplification conidering that the point is on the triangle centroid
					Point3d inter = Vector3d(lowest_pt - vh / vh.length()*b + lowest_pt + lowest_pt + axis_vector*h) / 3;
					Point3d pt = body->getLocalCoordinates(inter);
					//applyForceTo(body, F, pt);

					result_force.F = F;
					result_force.pt = pt;

				}


				//now I have to compute a negative weight to handle the case where the cylinder is horizontal enougth
				//to let the water go above the upper face. So that negative weight will correspond to the cylindrical wedge
				//above that face.
				if (low_high_pt.y < water_level){
					double h = (axis_vector*((water_level - low_high_pt.y) / axis_vector.y)).length();
					double b = (-vh *((water_level - low_high_pt.y) / (-vh.y))).length();
					double a = std::sqrt(b*(2 * r - b));

					double V = (h / (6 * b))*(2 * a*(3 * r*r - 2 * a*b + b*b) - 3 * PI*r*r*(r - b) +
						6 * r*r*(r - b)*std::asin((r - b) / r));
					Vector3d F = Vector3d(0, -1, 0)*V*density*gravity;

					//since the calculation of the real application point seems to be pretty hard
					//I'll use a simplification conidering that the point is on the triangle centroid
					Point3d pt = Vector3d(low_high_pt - vh / vh.length()*b + low_high_pt + low_high_pt + axis_vector*h) / 3;
					//applyForceTo(body, F, pt);

					//here getting the result force is more tricky 
					//so the effective application point is the point where the sum of the moment is null.
					Point3d pt2 = body->getWorldCoordinates(result_force.pt);
					Vector3d vect_support = pt2 - pt;
					double L2 = vect_support.length()*F.length() / (result_force.F.length() - F.length());
					result_force.pt = pt2 + vect_support / vect_support.length()*L2;
					result_force.pt = body->getLocalCoordinates(result_force.pt);
					result_force.F += F;
				}
			}
		}
		//applyForceTo(body, result_force.F, result_force.pt);

		return result_force;
	}
	return ForceStruct();
}